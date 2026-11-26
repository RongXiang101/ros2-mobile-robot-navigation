# ROS 2 Autonomous Vacuum Robot

> This README was rewritten for the modified version of Samer Odeh’s ROS 2 Autonomous Robot project.

## Project Overview

A simulated differential-drive robot builds an occupancy map from a 2D lidar, navigates around static obstacles with Nav2, and either explores frontiers on a live map or follows a fixed waypoint patrol on a saved map. The target environment is Ubuntu 22.04, ROS 2 Humble, and Gazebo Fortress.

The upstream project already supplied the robot model and test room, simulation bridge, SLAM and Nav2 bring-up, DWB tuning, a baseline frontier explorer, and waypoint patrol. This version adds simulated IMU input and EKF-based odometry TF, a single-pass patrol, and path feasibility and goal cooldown logic for exploration.

## System Architecture

```text
Gazebo Fortress → ros_gz_bridge → /scan → SLAM Toolbox or AMCL → map → odom
                                → /odom, /imu/data → covariance relay → EKF → odom → base_link
                                → /joint_states → robot_state_publisher → robot link transforms
Live /map → Frontier Explorer → ComputePathToPose → NavigateToPose → Nav2
Saved map + pose estimate → Patrol → NavigateToPose → Nav2
Nav2 → /cmd_vel → ros_gz_bridge → Gazebo differential drive
```

The relay publishes `/wheel/odom` and `/imu/data_cov`. The EKF publishes `/odometry/filtered` and owns `odom → base_link`. Gazebo still supplies raw `/odom`, which the current Nav2 BT navigator uses. The bridge does not forward Gazebo TF. SLAM Toolbox and AMCL each provide `map → odom` in their respective modes.

| Path | Contents |
| --- | --- |
| `config/` | Nav2, DWB override, EKF and SLAM Toolbox parameters |
| `description/`, `worlds/` | Xacro robot, simulated sensors and Fortress worlds |
| `launch/` | Simulation, mapping, localization, navigation, patrol and combined exploration |
| `scripts/` | Frontier explorer, one-pass patrol and covariance relay |
| `maps/` | Paired PGM and YAML maps, including the recorded exploration map |
| `rviz/`, `docs/` | RViz configurations and upstream documentation assets |
| `experiments/` | Historical experiment records and provenance |

## Dependencies

Use Python 3 and the ROS 2 Humble packages listed in [package.xml](package.xml), including SLAM Toolbox, Nav2 and its Simple Commander, `robot_localization`, RViz 2, Xacro, `ros_gz_sim` and `ros_gz_bridge` for Fortress. The simulation launch uses the Gazebo GUI and needs a working display with Ogre2 rendering. Check `ign gazebo --versions` for Gazebo Sim 6.x (Fortress).

## Installation

Install ROS 2 Humble, Fortress, `rosdep`, and `colcon` first. Initialize `rosdep` once if needed (`sudo rosdep init`, then `rosdep update`). Set `REPO_URL` to the HTTPS or SSH clone URL of this repository.

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
git clone "$REPO_URL" src/articubot_one
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build --packages-select articubot_one
source install/setup.bash
```

## Simulation Setup

Source the ROS underlay and workspace overlay in each terminal. Use the same ROS domain for all processes. Run the test room with its static obstacles and IMU system.

```bash
PKG_SHARE="$(ros2 pkg prefix --share articubot_one)"
ros2 launch articubot_one launch_sim.launch.py world:="$PKG_SHARE/worlds/slam_test.sdf"
```

The bridge declarations live in `launch/launch_sim.launch.py`. Check `/clock`, `/scan`, `/odom`, `/imu/data`, `/wheel/odom` and `/odometry/filtered` before launching higher-level components. An EKF parameter file can be selected with `ekf_config:=/path/to/ekf.yaml`.

## SLAM Mapping

With simulation running, start SLAM Toolbox in a separate terminal. It consumes `/scan`, publishes `/map`, and supplies `map → odom`.

```bash
ros2 launch articubot_one online_async_launch.py
```

## Localization

For saved-map navigation, run the simulation and start AMCL in a separate terminal with SLAM Toolbox stopped. The default map is `slam_test_map_v2.yaml` from the installed package. The `map` launch argument also accepts another map, such as `frontier_explore_v1.yaml`. Use RViz’s **2D Pose Estimate** to set the initial pose and check that the scan aligns with the map.

```bash
PKG_SHARE="$(ros2 pkg prefix --share articubot_one)"
ros2 launch articubot_one localization_launch.py use_sim_time:=true \
  map:="$PKG_SHARE/maps/slam_test_map_v2.yaml"
```

Open the Nav2 RViz configuration in another terminal for **2D Pose Estimate** and **Nav2 Goal**.

```bash
rviz2 -d "$(ros2 pkg prefix --share nav2_bringup)/rviz/nav2_default_view.rviz" \
  --ros-args -p use_sim_time:=true
```

## Nav2 Navigation

Start navigation with either live SLAM or saved-map localization. The stack uses Navfn (`GridBased`), the DWB controller with the bundled override, and lidar-fed costmaps. Check that `/compute_path_to_pose` and `/navigate_to_pose` are available and that the lifecycle nodes are active. RViz’s **Nav2 Goal** can be used for manual navigation goals.

```bash
ros2 launch articubot_one navigation_launch.py use_sim_time:=true autostart:=true
```

## Patrol

After AMCL and Nav2 are active, run the patrol below. It sends three fixed map-frame poses, in order, at `(1, 0)`, `(2, 1)`, and `(0, 1)` metres. It exits after one pass and ends the pass early if a goal fails or is canceled. Check that each waypoint lies in navigable space before using a different map.

```bash
ros2 launch articubot_one patrol.launch.py
```

## Frontier Exploration

The explorer scans the interior of the live `OccupancyGrid` for free cells touching at least one unknown cell in the eight-neighborhood. It checks candidates in grid scan order, converts candidate indices to map-frame poses, and prechecks up to eight unblocked candidates per map update round. It issues one navigation goal at a time.

```bash
ros2 launch articubot_one bringup_explore.launch.py
```

The combined launch starts the test world, SLAM Toolbox, Nav2 and the explorer at fixed delays. SLAM and Nav2 have separate launch scopes so each loads its own default parameter file. The staged procedure below lets you check readiness at each step.

## Algorithm Improvements

- **Path feasibility.** Asynchronous `ComputePathToPose` requests use `GridBased`. A candidate needs a successful plan with at least two poses and an endpoint within `0.1 m` of the requested target before the explorer sends `NavigateToPose`.
- **Navfn tolerance.** `GridBased.tolerance` changed from `0.5 m` to `0.1 m`. The larger tolerance could produce a path ending away from its frontier and contribute to repeated navigation successes without meaningful progress in the recorded tests.
- **Goal cooldown.** Attempted regions are excluded for 180 seconds of monotonic time. The exclusion radius is `0.45 m` after success, `0.65 m` after an aborted navigation or rejected path check, and `0.60 m` for cancellation, action rejection or other navigation errors.
- **Planar state estimation.** The EKF fuses wheel `vx`, `vy` and `wz` with IMU `wz`. Differential-drive odometry supplies the near-zero lateral velocity measurement. The relay preserves that velocity and assigns it a provisional variance of `1.0e-5`. The EKF publishes `odom → base_link` and uses `two_d_mode`.

## Experimental Results

Project runs recorded working Gazebo simulation and sensor input, EKF estimation, SLAM Toolbox mapping, AMCL localization, Nav2 navigation with static obstacle avoidance, one-pass patrol, frontier exploration with path prechecks, and map saving.

One exploration run sent **4 `NavigateToPose` requests**. Three returned `SUCCEEDED`, and one returned `ABORTED`. Multiple candidates were filtered before navigation. These counts describe that run only.

| Saved exploration map | Value |
| --- | ---: |
| Grid size | 239 × 239 cells |
| Resolution | 0.05 m/cell |
| Free / occupied / unknown | 53,522 / 2,107 / 1,492 |
| Known / unknown fraction | 97.39% / 2.61% |
| Remaining frontier cells / 8-connected components | 30 / 12 |

The denominator for 97.39% is the rectangular extent of the saved map. Reachable-area coverage and exploration completion were not established. The [2026-11-26 experiment record](experiments/2026-11-26-verification.md) describes the evidence and the frozen snapshot.

Release validation used a clean build and a short Fortress server-only run. Nav2 activated with `GridBased.tolerance=0.1`, the explorer filtered unplannable candidates, and one navigation goal completed. A separate localization startup loaded the installed v2 map and activated AMCL. The short checks ended before a full exploration or patrol run.

## Reproduction Instructions

1. Start the test room as shown in **Simulation Setup**. Wait for `/clock`, `/scan`, and the EKF’s `odom → base_link` transform.
2. Start SLAM Toolbox. Wait for `/map` and `map → odom`.
3. Start Nav2. Check `ros2 param get /planner_server GridBased.tolerance` for `0.1` and check both action servers with `ros2 action list -t`.
4. Start the explorer in a separate terminal. Observe `Checking path`, planning rejections or `Path accepted`, `Sending frontier goal`, and the navigation results.
5. Save the live map to a writable directory using a new output basename. The map saver writes the YAML and image together.

```bash
ros2 run articubot_one frontier_explorer.py
# In another sourced terminal, once ready to save the live map:
ros2 run nav2_map_server map_saver_cli -f "$PWD/frontier_reproduction" \
  --ros-args -p use_sim_time:=true
```

For saved-map patrol, stop the exploration stack and restart the same test room with AMCL, Nav2 and patrol as documented above.

## Known Limitations

- The explorer has no formal completion criterion, frontier clustering, or path-cost ranking. Its per-round limit and temporary spatial exclusions can leave some candidates unvisited. The grid scan skips its outermost row and column on each side.
- A passing precheck does not guarantee reachability when `NavigateToPose` replans. Nav2’s navigation goal checker has its own positional tolerance (`0.25 m`).
- Combined launch timers are fixed delays. Startup timing and navigation outcomes can vary by machine and run.
- An interrupt can produce an `rclpy` double-shutdown error in the explorer and covariance relay on Humble.
- EKF covariances are provisional. The lateral constraint assumes negligible sideslip, and wheel/IMU fusion without absolute observations can drift. The project was verified in simulation.

## Acknowledgments and Attribution

This repository retains the history of [Samer Odeh’s ROS 2 Autonomous Robot](https://github.com/samerodeh/ros2_autonomous_robot). Odeh’s [Apache 2.0 license and attribution](LICENSE) cover his contributions, including the autonomy baseline, configuration tuning, simulation bring-up, and documentation. Existing per-file notices, including the Nav2 launch attribution, are retained.

The base robot description and inherited launch files trace back to [Josh Newans’s Articulated Robotics `articubot_one`](https://github.com/joshnewans/articubot_one), which does not declare a repository-wide license. Its predecessor, the [`my_bot` template](https://github.com/joshnewans/my_bot), is Apache 2.0. Consult the original notices for the scope of each license. Documentation assets in `docs/` describe Odeh’s upstream setup, including its TF arrangement.
