# AGENTS.md

## Repo shape

- ROS 2 Humble `ament_python` package `turtlebot3_nav2`; repo root is the colcon workspace (`src/turtlebot3_nav2/...`). Single package, no monorepo tooling.
- Runtime entrypoint: `src/turtlebot3_nav2/turtlebot3_nav2/nav2_controller.py` (`main()`), exposed via `setup.py` as console script `nav2_controller`.
- Python modules must go in the nested `turtlebot3_nav2/turtlebot3_nav2/` package (packaged by `find_packages`), not the package root.
- No maps, Nav2 params, or robot description live here. They come from installed Nav2/TurtleBot3 packages.
- `launch/turtlebot3_nav2_launch.py` includes `nav2_bringup/launch/tb3_simulation_launch.py` and then starts the controller. `nav2_bringup` and its launch arguments must exist in the sourced ROS installation.

## Environment

- Target: Ubuntu 22.04 + ROS 2 Humble + Gazebo/TurtleBot3 (see `README.md`). The current checkout is on macOS with no ROS installed, so `colcon`, `ros2`, and the ROS linters cannot run here. Pure Python tests can.
- Keep code Python 3.9-compatible so the pure tests run on the current macOS dev machine; Humble itself uses Python 3.10.

## Commands (Ubuntu/Humble)

```sh
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select turtlebot3_nav2   # repo root
source install/setup.bash
ros2 launch turtlebot3_nav2 turtlebot3_nav2_launch.py headless:=False
# or, with Nav2 already running:
ros2 run turtlebot3_nav2 nav2_controller --ros-args -p use_sim_time:=true
```

- Full check: `colcon test --packages-select turtlebot3_nav2 && colcon test-result --verbose`
- Pure tests without ROS: from `src/turtlebot3_nav2/`, run `python3 -m pytest test/test_geometry.py test/test_goal_sampler.py`.
- Single ROS lint file: from `src/turtlebot3_nav2/`, run `python3 -m pytest test/test_flake8.py` after sourcing ROS (tests import the `ament_*` linters and lint the current directory).
- Tests: `test_geometry.py` and `test_goal_sampler.py` are real unit tests; `test_flake8.py`/`test_pep257.py` are linters; `test_copyright.py` is skipped. No CI.

## Runtime prerequisites

- `nav2_controller` blocks in `waitUntilNav2Active()` until the Nav2 lifecycle nodes and AMCL are up. Standalone it appears to hang; start the simulation first.
- The initial pose is published before waiting; `waitUntilNav2Active()` republishes it until `amcl_pose` arrives. Do not move that call back into `__init__`.
- Gazebo runs on simulation time. Always pass `-p use_sim_time:=true` when running the node against Gazebo, or goal timestamps and TF lookups can fail.

## Controller behavior

- Parameters (defaults preserve the demo): `frame_id`, `initial_pose.{x,y,z,yaw}`, `goal_area.{min_x,max_x,min_y,max_y}` (±1.5), `navigation_timeout_sec` (600). They are declared read-only after startup; restart the node to change them.
- `run()` loops forever, samples one random goal at a time, and logs the actual `TaskResult`; it is no longer recursive. Keep it iterative.
- Random yaw uses a normalized quaternion in `geometry.py`; goal sampling/validation is in `goal_sampler.py`. Keep pure logic there so it stays testable without ROS.
- `Nav2Controller` owns a single `BasicNavigator` node named `nav2_controller`; it is not itself an rclpy Node. Destroy the navigator in `main()`'s `finally` block.
