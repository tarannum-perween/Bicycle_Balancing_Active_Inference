#!/usr/bin/env python3
"""Load a MuJoCo MJCF model and optionally run the interactive viewer."""

import argparse
import os
import sys
import time

from ament_index_python.packages import get_package_share_directory


def parse_bool(value):
    return str(value).lower() in ('1', 'true', 'yes', 'on')


def resolve_model_path(model_arg):
    if os.path.isabs(model_arg):
        return model_arg

    local_path = os.path.abspath(model_arg)
    if os.path.exists(local_path):
        return local_path

    share = get_package_share_directory('bicycle')
    return os.path.join(share, 'model', model_arg)


def main():
    parser = argparse.ArgumentParser(description='Run a bicycle MJCF model in MuJoCo.')
    parser.add_argument('--model', default='bicycle.xml',
                        help='MJCF filename in bicycle/model, or an absolute/relative path.')
    parser.add_argument('--duration', type=float, default=0.0,
                        help='Seconds to simulate. Use 0 to run until the viewer closes.')
    parser.add_argument('--headless', default='false',
                        help='Set true to step the simulation without opening the MuJoCo viewer.')
    args = parser.parse_args()

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

    if parse_bool(args.headless):
        end_time = args.duration if args.duration > 0.0 else 1.0
        while data.time < end_time:
            mujoco.mj_step(model, data)
        print(f'Simulated {data.time:.3f} seconds from {model_path}')
        return 0

    try:
        import mujoco.viewer
    except ImportError:
        print('MuJoCo viewer support is unavailable in this Python environment.', file=sys.stderr)
        return 1

    with mujoco.viewer.launch_passive(model, data) as viewer:
        start = time.monotonic()
        while viewer.is_running():
            if args.duration > 0.0 and time.monotonic() - start >= args.duration:
                break
            step_start = time.monotonic()
            mujoco.mj_step(model, data)
            viewer.sync()
            sleep_time = model.opt.timestep - (time.monotonic() - step_start)
            if sleep_time > 0.0:
                time.sleep(sleep_time)

    return 0


if __name__ == '__main__':
    sys.exit(main())
