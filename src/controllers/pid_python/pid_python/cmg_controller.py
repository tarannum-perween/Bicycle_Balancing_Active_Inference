"""MuJoCo CMG balance controller for the bicycle-rider model."""

import argparse
import math
import os
import sys
import time


def parse_bool(value):
    """Parse common command-line boolean strings."""
    return str(value).lower() in ('1', 'true', 'yes', 'on')


def resolve_model_path(model_arg):
    """Resolve an absolute, relative, or package-share MJCF path."""
    if os.path.isabs(model_arg):
        return model_arg

    candidates = [
        os.path.abspath(model_arg),
        os.path.abspath(os.path.join('src', 'bicycle', 'model', model_arg)),
        os.path.abspath(os.path.join('bicycle', 'model', model_arg)),
    ]
    for local_path in candidates:
        if os.path.exists(local_path):
            return local_path

    try:
        from ament_index_python.packages import get_package_share_directory
    except ImportError:
        return candidates[0]

    share = get_package_share_directory('bicycle')
    return os.path.join(share, 'model', model_arg)


def get_joint_ids(mujoco, model, joint_name):
    """Return qpos/qvel/dof addresses for a MuJoCo joint."""
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    if joint_id < 0:
        raise ValueError(f'Joint not found in model: {joint_name}')

    return {
        'joint_id': joint_id,
        'qpos': int(model.jnt_qposadr[joint_id]),
        'qvel': int(model.jnt_dofadr[joint_id]),
        'dof': int(model.jnt_dofadr[joint_id]),
    }


def get_optional_joint_ids(mujoco, model, joint_name):
    """Return joint ids when present, otherwise None."""
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    if joint_id < 0:
        return None

    return {
        'joint_id': joint_id,
        'qpos': int(model.jnt_qposadr[joint_id]),
        'qvel': int(model.jnt_dofadr[joint_id]),
        'dof': int(model.jnt_dofadr[joint_id]),
    }


def get_body_id(mujoco, model, body_name):
    """Return a MuJoCo body id by name."""
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    if body_id < 0:
        raise ValueError(f'Body not found in model: {body_name}')
    return body_id


def get_actuator_id(mujoco, model, actuator_name):
    """Return a MuJoCo actuator id by name, or -1 if it does not exist."""
    return mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_ACTUATOR,
        actuator_name,
    )


def body_roll(model, data, body_id):
    """
    Estimate body roll angle from the body orientation matrix.

    This assumes the bicycle's forward axis is local X, lateral axis is local Y,
    and vertical axis is local Z. That matches the current bicycle MJCF layout.
    """
    del model
    xmat = data.xmat[body_id].reshape(3, 3)
    return math.atan2(xmat[2, 1], xmat[2, 2])


def clamp(value, lower, upper):
    """Clamp a control value to a finite interval."""
    return max(lower, min(upper, value))


def build_arg_parser():
    """Build CLI arguments for the controller."""
    parser = argparse.ArgumentParser(
        description='Run a CMG balance controller on a MuJoCo bicycle.'
    )
    parser.add_argument(
        '--model',
        default='mjcf_bicycle_with_rider/mujoco_description_formatted.xml',
        help='MJCF path, relative path, or filename under bicycle/share/model.',
    )
    parser.add_argument('--duration', type=float, default=10.0)
    parser.add_argument('--headless', default='false')
    parser.add_argument('--balance-body', default='frame')
    parser.add_argument('--drive-body', default='frame')
    parser.add_argument('--steering-joint', default='steering_joint')
    parser.add_argument('--rear-wheel-joint', default='wheel1_joint')
    parser.add_argument('--torso-joint', default='torso_roll_joint')
    parser.add_argument('--cmg-gimbal-joint', default='cmg_roll_moment_joint')
    parser.add_argument('--cmg-flywheel-joint', default='cmg_flywheel_joint')
    parser.add_argument('--target-speed', type=float, default=1.0)
    parser.add_argument(
        '--kinematic-drive',
        default='false',
        help='Set the free-base forward velocity directly for early balance tuning.',
    )
    parser.add_argument(
        '--pose-drive',
        default='false',
        help='Move the floating base position directly along drive-axis.',
    )
    parser.add_argument(
        '--drive-axis',
        choices=('x', 'y', 'z'),
        default='x',
        help='World axis used for temporary kinematic/pose drive.',
    )
    parser.add_argument(
        '--initial-speed',
        type=float,
        default=0.0,
        help='Initial forward speed applied to the floating base in m/s.',
    )
    parser.add_argument('--wheel-radius', type=float, default=0.333)
    parser.add_argument('--kp-roll', type=float, default=40.0)
    parser.add_argument('--kd-roll', type=float, default=8.0)
    parser.add_argument(
        '--kp-roll-assist',
        type=float,
        default=0.0,
        help='Optional direct roll stabilizer gain for comparison/debug only.',
    )
    parser.add_argument(
        '--kd-roll-assist',
        type=float,
        default=0.0,
        help='Optional direct roll damping gain for comparison/debug only.',
    )
    parser.add_argument('--kp-steer', type=float, default=2.0)
    parser.add_argument('--kd-steer', type=float, default=0.7)
    parser.add_argument('--kp-speed', type=float, default=1.5)
    parser.add_argument(
        '--flywheel-speed',
        type=float,
        default=500.0,
        help='Target CMG flywheel speed in rad/s.',
    )
    parser.add_argument(
        '--flywheel-inertia',
        type=float,
        default=0.004912,
        help='CMG flywheel polar inertia in kg*m^2.',
    )
    parser.add_argument(
        '--kp-cmg-rate',
        type=float,
        default=6.0,
        help='Gimbal-rate tracking gain for the CMG roll-moment motor.',
    )
    parser.add_argument(
        '--kd-cmg-angle',
        type=float,
        default=1.0,
        help='Gimbal centering gain that keeps the CMG away from its limits.',
    )
    parser.add_argument(
        '--kp-forward-force',
        type=float,
        default=30.0,
        help='Temporary forward force gain on the free base, useful while tire contact is being tuned.',
    )
    parser.add_argument('--kp-torso', type=float, default=25.0)
    parser.add_argument('--kd-torso', type=float, default=8.0)
    parser.add_argument('--max-steer-torque', type=float, default=20.0)
    parser.add_argument('--max-drive-torque', type=float, default=8.0)
    parser.add_argument('--max-cmg-roll-torque', type=float, default=22.6)
    parser.add_argument('--max-cmg-gimbal-rate', type=float, default=8.0)
    parser.add_argument('--max-cmg-motor-torque', type=float, default=22.6)
    parser.add_argument('--max-forward-force', type=float, default=30.0)
    parser.add_argument('--max-roll-assist-torque', type=float, default=120.0)
    parser.add_argument('--max-torso-torque', type=float, default=80.0)
    parser.add_argument(
        '--invert-cmg',
        default='false',
        help='Flip CMG reaction direction if it amplifies lean instead of opposing it.',
    )
    return parser


def axis_index(axis_name):
    """Return the world vector index for an axis name."""
    return {'x': 0, 'y': 1, 'z': 2}[axis_name]


def warn_if_fixed_base(model):
    """Warn when the model cannot physically roll because it has no free joint."""
    has_free_joint = any(model.jnt_type[i] == 0 for i in range(model.njnt))
    if not has_free_joint:
        print(
            'Warning: this MJCF has no free joint, so the bicycle base appears '
            'fixed to the world. Steering torque can move the fork, but the '
            'whole bicycle cannot fall/balance until the root body is free.',
            file=sys.stderr,
        )


def run_controller(args):
    """Load the model and run the CMG controller."""
    try:
        import mujoco
    except ImportError:
        print(
            'Python package "mujoco" is not installed. Install it with:\n'
            '  python3 -m pip install mujoco',
            file=sys.stderr,
        )
        return 1

    model_path = resolve_model_path(args.model)
    if not os.path.exists(model_path):
        print(f'MJCF model not found: {model_path}', file=sys.stderr)
        return 1

    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    steer = get_joint_ids(mujoco, model, args.steering_joint)
    rear_wheel = get_joint_ids(mujoco, model, args.rear_wheel_joint)
    torso = get_joint_ids(mujoco, model, args.torso_joint)
    cmg_gimbal = get_optional_joint_ids(mujoco, model, args.cmg_gimbal_joint)
    cmg_flywheel = get_optional_joint_ids(mujoco, model, args.cmg_flywheel_joint)
    base_free = get_joint_ids(mujoco, model, 'base_free_joint')
    balance_body_id = get_body_id(mujoco, model, args.balance_body)
    drive_body_id = get_body_id(mujoco, model, args.drive_body)
    steering_actuator_id = get_actuator_id(mujoco, model, 'steering_motor')
    rear_wheel_actuator_id = get_actuator_id(mujoco, model, 'rear_wheel_motor')
    torso_actuator_id = get_actuator_id(mujoco, model, 'torso_roll_motor')
    cmg_gimbal_actuator_id = get_actuator_id(
        mujoco,
        model,
        'cmg_roll_moment_motor',
    )
    cmg_flywheel_actuator_id = get_actuator_id(
        mujoco,
        model,
        'cmg_flywheel_velocity',
    )
    warn_if_fixed_base(model)

    target_wheel_speed = args.target_speed / args.wheel_radius
    cmg_sign = -1.0 if parse_bool(args.invert_cmg) else 1.0
    drive_axis_id = axis_index(args.drive_axis)
    previous_roll = None

    if cmg_gimbal is None or cmg_flywheel is None:
        print(
            'Warning: CMG joints were not found in this MJCF. The controller '
            'will keep drive/torso/steering-centering active, but CMG balance '
            'requires cmg_roll_moment_joint and cmg_flywheel_joint.',
            file=sys.stderr,
        )
    if cmg_gimbal_actuator_id < 0:
        print(
            'Warning: cmg_roll_moment_motor actuator was not found.',
            file=sys.stderr,
        )
    if cmg_flywheel_actuator_id < 0:
        print(
            'Warning: cmg_flywheel_velocity actuator was not found.',
            file=sys.stderr,
        )

    if args.initial_speed:
        data.qvel[base_free['qvel'] + drive_axis_id] = args.initial_speed

    def apply_control():
        nonlocal previous_roll

        roll = body_roll(model, data, balance_body_id)
        if previous_roll is None:
            roll_rate = 0.0
        else:
            roll_rate = (roll - previous_roll) / model.opt.timestep
        previous_roll = roll

        steer_angle = data.qpos[steer['qpos']]
        steer_rate = data.qvel[steer['qvel']]
        rear_wheel_rate = data.qvel[rear_wheel['qvel']]
        torso_angle = data.qpos[torso['qpos']]
        torso_rate = data.qvel[torso['qvel']]
        cmg_gimbal_angle = 0.0
        cmg_gimbal_rate = 0.0
        cmg_flywheel_rate = args.flywheel_speed
        if cmg_gimbal is not None:
            cmg_gimbal_angle = data.qpos[cmg_gimbal['qpos']]
            cmg_gimbal_rate = data.qvel[cmg_gimbal['qvel']]
        if cmg_flywheel is not None:
            cmg_flywheel_rate = data.qvel[cmg_flywheel['qvel']]
        forward_speed = data.qvel[base_free['qvel'] + drive_axis_id]
        if parse_bool(args.kinematic_drive):
            data.qvel[base_free['qvel'] + drive_axis_id] = clamp(
                args.target_speed,
                -abs(args.target_speed),
                abs(args.target_speed),
            )
            forward_speed = args.target_speed
        if parse_bool(args.pose_drive):
            data.qpos[base_free['qpos'] + drive_axis_id] += (
                args.target_speed * model.opt.timestep
            )
            forward_speed = args.target_speed

        corrective_roll_torque = -cmg_sign * (
            args.kp_roll * roll + args.kd_roll * roll_rate
        )
        corrective_roll_torque = clamp(
            corrective_roll_torque,
            -args.max_cmg_roll_torque,
            args.max_cmg_roll_torque,
        )
        flywheel_momentum = args.flywheel_inertia * max(
            abs(cmg_flywheel_rate),
            1.0,
        )
        desired_gimbal_rate = corrective_roll_torque / flywheel_momentum
        desired_gimbal_rate = clamp(
            desired_gimbal_rate,
            -args.max_cmg_gimbal_rate,
            args.max_cmg_gimbal_rate,
        )
        cmg_motor_torque = args.kp_cmg_rate * (
            desired_gimbal_rate - cmg_gimbal_rate
        )
        cmg_motor_torque -= args.kd_cmg_angle * cmg_gimbal_angle

        steering_torque = -args.kp_steer * steer_angle
        steering_torque -= args.kd_steer * steer_rate
        drive_torque = args.kp_speed * (target_wheel_speed - rear_wheel_rate)
        forward_force = args.kp_forward_force * (
            args.target_speed - forward_speed
        )
        roll_assist_torque = -args.kp_roll_assist * roll
        roll_assist_torque -= args.kd_roll_assist * roll_rate
        torso_torque = -args.kp_torso * torso_angle - args.kd_torso * torso_rate

        steering_torque = clamp(
            steering_torque,
            -args.max_steer_torque,
            args.max_steer_torque,
        )
        drive_torque = clamp(
            drive_torque,
            -args.max_drive_torque,
            args.max_drive_torque,
        )
        cmg_motor_torque = clamp(
            cmg_motor_torque,
            -args.max_cmg_motor_torque,
            args.max_cmg_motor_torque,
        )
        torso_torque = clamp(
            torso_torque,
            -args.max_torso_torque,
            args.max_torso_torque,
        )
        forward_force = clamp(
            forward_force,
            -args.max_forward_force,
            args.max_forward_force,
        )
        roll_assist_torque = clamp(
            roll_assist_torque,
            -args.max_roll_assist_torque,
            args.max_roll_assist_torque,
        )

        data.xfrc_applied[drive_body_id, drive_axis_id] = forward_force
        data.xfrc_applied[drive_body_id, 3] = roll_assist_torque

        if steering_actuator_id >= 0:
            data.ctrl[steering_actuator_id] = steering_torque
        else:
            data.qfrc_applied[steer['dof']] = steering_torque

        if rear_wheel_actuator_id >= 0:
            data.ctrl[rear_wheel_actuator_id] = drive_torque
        else:
            data.qfrc_applied[rear_wheel['dof']] = drive_torque

        if torso_actuator_id >= 0:
            data.ctrl[torso_actuator_id] = torso_torque
        else:
            data.qfrc_applied[torso['dof']] = torso_torque

        if cmg_flywheel_actuator_id >= 0:
            data.ctrl[cmg_flywheel_actuator_id] = args.flywheel_speed
        elif cmg_flywheel is not None:
            data.qvel[cmg_flywheel['qvel']] = args.flywheel_speed

        if cmg_gimbal is not None:
            if cmg_gimbal_actuator_id >= 0:
                data.ctrl[cmg_gimbal_actuator_id] = cmg_motor_torque
            else:
                data.qfrc_applied[cmg_gimbal['dof']] = cmg_motor_torque

    if parse_bool(args.headless):
        while data.time < args.duration:
            data.qfrc_applied[:] = 0.0
            data.xfrc_applied[:] = 0.0
            apply_control()
            mujoco.mj_step(model, data)
        print(f'Finished {data.time:.3f} seconds with CMG controller.')
        return 0

    try:
        import mujoco.viewer
    except ImportError:
        print('MuJoCo viewer support is unavailable.', file=sys.stderr)
        return 1

    with mujoco.viewer.launch_passive(model, data) as viewer:
        start = time.monotonic()
        while viewer.is_running():
            if args.duration > 0.0 and time.monotonic() - start >= args.duration:
                break
            step_start = time.monotonic()
            data.qfrc_applied[:] = 0.0
            data.xfrc_applied[:] = 0.0
            apply_control()
            mujoco.mj_step(model, data)
            viewer.sync()
            sleep_time = model.opt.timestep - (time.monotonic() - step_start)
            if sleep_time > 0.0:
                time.sleep(sleep_time)

    return 0


def main():
    """CLI entry point."""
    parser = build_arg_parser()
    return run_controller(parser.parse_args())


if __name__ == '__main__':
    sys.exit(main())
