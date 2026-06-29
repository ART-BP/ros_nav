# neupan_service

Standalone ROS1 service wrapper for NeuPAN.

This package is intended to be copied as a small, migration-friendly catkin
package. It contains:

- `scripts/neupan_service_server.py`: persistent NeuPAN server node.
- `srv/SetPath.srv`: set/update the reference path.
- `srv/ComputeVelocity.srv`: compute one velocity command.
- `src/neupan_local_planner_ros.cpp`: `move_base` local planner plugin that
  adapts `nav_core::BaseLocalPlanner` to the NeuPAN services.
- `config/neupan_planner_limo.yaml`: default LIMO planner config.
- `weights/limo/model_5000.pth`: default LIMO DUNE checkpoint.
- `launch/neupan_service_limo.launch`: Gazebo LIMO wiring.

The NeuPAN Python package is still a runtime dependency. On the target machine,
install the Python package first, then build this catkin package.

## Build

```bash
cd <catkin_ws>/src
ln -s /path/to/neupan_service neupan_service
cd <catkin_ws>
catkin_make
```

## Run For LIMO Gazebo

Start Gazebo separately, then run:

```bash
source ~/anaconda3/etc/profile.d/conda.sh
conda activate neupan38
source /opt/ros/noetic/setup.bash
source <catkin_ws>/devel/setup.bash

roslaunch neupan_service neupan_service_limo.launch
```

The node publishes:

- `/cmd_vel`
- `/neupan_plan`
- `/neupan_ref_state`
- `/neupan_initial_path`

It subscribes to:

- `/limo/scan`
- `/move_base_simple/goal`
- `/initial_path`

Services:

- `/neupan_service/set_path`
- `/neupan_service/compute_velocity`

## Use As A move_base Local Planner

The plugin class is:

```yaml
base_local_planner: neupan_service/NeuPANLocalPlannerROS
```

`NeuPANLocalPlannerROS` calls `set_path` from `setPlan()` and
`compute_velocity` from `computeVelocityCommands()`. When used under
`move_base`, keep the NeuPAN service node running with:

```yaml
run_control_loop: false
publish_cmd_vel: false
```

so that only `move_base` publishes `/cmd_vel`.
