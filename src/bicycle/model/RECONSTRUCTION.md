# Approximate bicycle reconstruction

`bicycle.urdf.xacro` is a new model, not the missing original CAD assembly.
It can be expanded alone or included by `sim_main.urdf.xacro`.

## Provenance and assumptions

- Existing STL meshes supply visual geometry. Their dimensions indicate mm;
  scale 0.001 converts them to metres. CAD coordinates map to ROS (-X, Z, Y).
- The supplied PDF reports give frame mass 3.07549 kg, steering mass 1.55391 kg,
  and tire mass 0.45288 kg. The tire value is used per wheel and may omit hub/spokes.
  Those reports warn that their mass properties were copied from an assembly.
- Mesh/report origins differ, so CAD centers of mass and inertia tensors are
  deliberately NOT transplanted. Frame and steering use box inertia estimates;
  wheels use thin-hoop estimates. The IMU mass is an assumed 0.01 kg.
- Estimated rear/front axle positions are (0, 0, 0.333) and (0.99, 0, 0.333) m.
  Wheel radius is 0.333 m and width is 0.05 m, based on mesh bounds.
- Steering pivot (0.88, 0, 0.80) m, axis tilt, mesh offsets, collision proxies,
  joint limits, and damping are approximate. Check against the physical bicycle.
- Existing CMG attachment position is preserved. Its clearance is not certified.
- base_link is a freely moving reference, not a joint fastening the bike to ground.
- Legacy bicycle.gazebo is not invoked: sensor YAML and simulator plugins need
  separate repair/migration. No sensor topic or balancing controller is created.

## Preview

From the workspace root:

```bash
source setup_ros.bash
colcon build --symlink-install --packages-select cmgdevice_description bicycle
source setup_ros.bash
ros2 launch bicycle display.launch.py
# Bicycle alone:
ros2 launch bicycle display.launch.py model:=bicycle.urdf.xacro
```

RViz provides visual inspection and joint sliders, not physics simulation.
A successful Xacro/URDF check does not validate balance dynamics or Gazebo support.

## Seated rider

The display launch now shows a seated rider by default. Use `add_rider:=false`
to hide it. Its pelvis is attached to `frame` at (0.29, 0, 1.01) metres, near
the reconstructed saddle. Adjust using `rider_xyz:="0.29 0 1.01"`.
Direct Xacro expansion defaults to no rider; enable with `add_rider:=true`.

The uploaded `src/humanSubject01/humanSubject01_66dof_colored.urdf` is preserved.
`scripts/generate_seated_rider.py` creates `model/seated_rider.urdf.xacro` with
prefixed names and fixed joints embedding a seated pose. It retains body geometry
and nonzero masses/inertias, removes zero-sized helper visuals and zero-mass
inertials, and fits the arms toward estimated handlebar positions. Regeneration
requires NumPy and SciPy; the preview does not.

This is a static rider: the legs bend, the torso leans forward, and the hands
reach toward the handlebars. There is no pedaling, grip/contact constraint,
steering-following hand motion, or validated rider/bicycle collision behavior.
The uploaded model's colors and primitive body shapes are retained.
