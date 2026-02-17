# MSBRDM: Final Project Guide

## 1. Overview

`image_to_traj`:
- Takes an input image.
- Extracts the skeleton and splits it into multiple strokes.
- Fits each stroke with polynomials and outputs XML.
- Publishes the XML to ROS topic `/ur10/planar_polynomial_trajectories`.

`tum_ics_ur10_controller_tutorial`:
- Subscribes to `/ur10/planar_polynomial_trajectories` (`std_msgs/String`, XML content).
- Uses a controller state machine: `SAFE -> MOVE -> DRAW`.
- Executes each stroke in order and publishes debug/visualization topics.

## 2. End-to-End Workflow

1. Start the UR10 environment (model/state publisher/RViz).
2. Start `operational_space_controller`.
3. Use `gen_traj_node.py` to generate and publish trajectory XML from an image.
4. After receiving XML, the controller resets and starts from `SAFE`.

## 3. Build

Run in the workspace root:

```bash
catkin build -DTUM_ICS_USE_QT5=1
source devel/setup.bash
```

## 4. Launch Commands (4 terminals recommended)

Terminal A:

```bash
source devel/setup.bash
roscore
```

Terminal B:

```bash
source devel/setup.bash
roslaunch tum_ics_ur10_bringup bringUR10-FT-wsg-50.launch
```

Terminal C:

```bash
source devel/setup.bash
roslaunch tum_ics_ur10_controller_tutorial operational_space_controller.launch
```

Terminal D (publish trajectory):

```bash
source devel/setup.bash
python3 src/image_to_traj/scripts/gen_traj_node.py \
  --image src/image_to_traj/scripts/img/test.jpg \
  --print_lengths
```

Optional arguments:
- `--xml_out /tmp/strokes.xml`: save XML to file.
- `--save_fig /tmp/fit.png`: save fitted-curve visualization.
- `--show`: show a plot window.

## 5. XML Format from `image_to_traj`

Expected core structure (ordered by `stroke index`):

```xml
<trajectory>
  <strokes count="N">
    <stroke index="0" deg_used="10" n_points="...">
      <length_px>...</length_px>
      <polyfit>
        <x>...</x>
        <y>...</y>
      </polyfit>
    </stroke>
  </strokes>
</trajectory>
```

Notes:
- `x/y` coefficients in XML are in descending power order (`a_n ... a_0`, `np.polyfit` default).
- The controller converts them to ascending order internally before evaluation.

## 6. Key Controller Parameters

File: `src/tum_ics_ur10_controller_tutorial/launch/configs/operationalSpaceCtrl.yaml`

Important fields:
- `operational_space_ctrl/pixel_to_world/*`: pixel-to-world mapping.
- `operational_space_ctrl/move/*`: MOVE timing, speed, convergence thresholds.
- `operational_space_ctrl/draw/*`: DRAW timing, speed, fixed Z, blend ratio.
- `operational_space_ctrl/trajectory/topic`: XML trajectory topic (default `/ur10/planar_polynomial_trajectories`).

If drawing is shifted or the start point is wrong, tune `pixel_to_world` (`scale/offset/rotation_deg`) first.

## 7. Useful Topics

- Trajectory input:
  - `/ur10/planar_polynomial_trajectories` (`std_msgs/String`)
- Controller visualization/debug:
  - `/ur10/desired_trajectory` (`visualization_msgs/Marker`)
  - `/ur10/actual_trajectory` (`visualization_msgs/Marker`)
  - `/ur10/task_space_error` (`std_msgs/Float64MultiArray`)
  - `/ur10/joint_effort_debug` (`std_msgs/Float64MultiArray`)
  - `/ur10/joint_effort_state` (`sensor_msgs/JointState`)

## 8. Troubleshooting

1. Controller does not enter DRAW:
- Verify XML is received: `rostopic echo /ur10/planar_polynomial_trajectories -n 1`
- Check whether `move/pos_tol`, `move/time`, or `move/speed` is too strict.

2. XML is received but robot does not move:
- Check `enable_safe/enable_move/enable_draw` in `operationalSpaceCtrl.yaml` are `true`.
- Check if SAFE convergence takes too long due to a large gap to `q_safe`.

3. Trajectory direction or scale is wrong:
- Tune sign and magnitude of `pixel_to_world/scale_x` and `scale_y`.
- Tune `pixel_to_world/rotation_deg` (implemented as clockwise rotation).
