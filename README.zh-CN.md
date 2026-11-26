# ROS 2 自主清洁机器人

[English](README.md)

## 项目概述

本项目在 Ubuntu 22.04、ROS 2 Humble 和 Gazebo Fortress 中仿真一台差速驱动清洁机器人。机器人通过二维激光雷达构建占据栅格地图，使用 Nav2 规划路径并避开静态障碍物。它可以在建图时自主探索未知区域，也可以加载已有地图，按固定航点巡航。

Samer Odeh 的上游项目提供了机器人模型、测试房间、仿真桥接、SLAM 和 Nav2 启动方式、DWB 参数，以及前沿探索和航点巡航的基础实现。本项目在此基础上接入仿真 IMU、用 EKF 发布里程计坐标变换、将巡航改为单轮执行，并为前沿探索加入路径预检查、终点误差检查和目标冷却。

## 系统架构

```text
Gazebo Fortress → ros_gz_bridge
  ├─ /scan → SLAM Toolbox 或 AMCL → map → odom
  ├─ /odom、/imu/data → 协方差中继 → EKF → odom → base_link
  └─ /joint_states → robot_state_publisher → 机器人各连杆的坐标变换

实时 /map → 前沿探索 → ComputePathToPose → NavigateToPose → Nav2
已保存地图和定位结果 → 航点巡航 → NavigateToPose → Nav2
Nav2 → /cmd_vel → ros_gz_bridge → Gazebo 差速驱动
```

协方差中继发布 `/wheel/odom` 和 `/imu/data_cov`，EKF 发布 `/odometry/filtered` 以及 `odom → base_link`。Nav2 的行为树导航器仍读取 Gazebo 提供的原始 `/odom`。仿真桥接不转发 Gazebo 的 TF。建图模式由 SLAM Toolbox 发布 `map → odom`，已知地图模式由 AMCL 发布该变换。

| 目录 | 内容 |
| --- | --- |
| `config/` | Nav2、DWB 覆盖参数、EKF 和 SLAM Toolbox 配置 |
| `description/`、`worlds/` | Xacro 机器人模型、仿真传感器和 Fortress 场景 |
| `launch/` | 仿真、建图、定位、导航、巡航和组合探索启动文件 |
| `scripts/` | 前沿探索、单轮巡航和协方差中继节点 |
| `maps/` | 成对保存的 PGM 和 YAML 地图，包括实验探索地图 |
| `rviz/`、`docs/` | RViz 配置和上游项目文档资源 |
| `experiments/` | 历史实验记录和快照来源说明 |

## 依赖

使用 Python 3 和 ROS 2 Humble。运行依赖包括 SLAM Toolbox、Nav2、Nav2 Simple Commander、`robot_localization`、RViz 2、Xacro，以及与 Fortress 配套的 `ros_gz_sim` 和 `ros_gz_bridge`。完整依赖见 [package.xml](package.xml)。

默认仿真启动方式会打开 Gazebo 图形界面，需要可用的显示环境和 Ogre2 渲染。可用 `ign gazebo --versions` 检查 Gazebo Sim 版本，Fortress 对应 6.x。

## 安装与构建

先安装 ROS 2 Humble、Gazebo Fortress、`rosdep` 和 `colcon`。首次使用 `rosdep` 时，先运行 `sudo rosdep init`，再运行 `rosdep update`。

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
git clone https://github.com/RongXiang101/ros2-autonomous-vacuum-robot.git src/articubot_one
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build --packages-select articubot_one
source install/setup.bash
```

## 启动仿真

在每个新终端加载 ROS 2 Humble 和工作空间的环境脚本，并让各终端使用同一个 ROS 域。以下命令启动包含静态障碍物和 IMU 系统的测试房间。

```bash
PKG_SHARE="$(ros2 pkg prefix --share articubot_one)"
ros2 launch articubot_one launch_sim.launch.py world:="$PKG_SHARE/worlds/slam_test.sdf"
```

ROS 与 Gazebo 的桥接主题定义在 [仿真启动文件](launch/launch_sim.launch.py) 中。启动建图或导航前，检查 `/clock`、`/scan`、`/odom`、`/imu/data`、`/wheel/odom` 和 `/odometry/filtered`。如需指定 EKF 参数文件，可传入 `ekf_config:=/path/to/ekf.yaml`。

## SLAM 建图

仿真正常运行后，在另一终端启动 SLAM Toolbox。它订阅 `/scan`，发布实时地图 `/map` 和 `map → odom`。

```bash
ros2 launch articubot_one online_async_launch.py
```

## AMCL 定位

使用已保存地图导航时，保持仿真运行、停止 SLAM Toolbox，再启动 AMCL。默认地图为功能包内的 `slam_test_map_v2.yaml`。也可通过 `map` 参数选择其他地图，例如 `frontier_explore_v1.yaml`。

```bash
PKG_SHARE="$(ros2 pkg prefix --share articubot_one)"
ros2 launch articubot_one localization_launch.py use_sim_time:=true \
  map:="$PKG_SHARE/maps/slam_test_map_v2.yaml"
```

在另一终端启动 RViz。用 **2D Pose Estimate** 设置初始位姿，并检查激光扫描是否与地图对齐。

```bash
rviz2 -d "$(ros2 pkg prefix --share nav2_bringup)/rviz/nav2_default_view.rviz" \
  --ros-args -p use_sim_time:=true
```

## Nav2 导航

实时 SLAM 建图模式和 AMCL 定位模式均可启动 Nav2。Navfn（`GridBased`）负责全局路径规划，DWB 使用包内覆盖参数执行局部控制，代价地图接收激光扫描。启动后检查 Nav2 生命周期节点已激活，且 `/compute_path_to_pose` 和 `/navigate_to_pose` 可用。也可用 RViz 的 **Nav2 Goal** 手动发送目标。

```bash
ros2 launch articubot_one navigation_launch.py use_sim_time:=true autostart:=true
```

## 多目标巡航

AMCL 定位和 Nav2 就绪后启动巡航。节点依次发送 `map` 坐标系下的三个固定航点 `(1, 0)`、`(2, 1)`、`(0, 1)`，单位为米。完成一轮后退出，遇到失败或取消时提前结束。更换地图时，先核对这些航点是否位于可通行区域。

```bash
ros2 launch articubot_one patrol.launch.py
```

## 前沿探索

探索节点扫描实时 `OccupancyGrid` 的内部栅格，将八邻域内至少有一个未知格的空闲格视作前沿。节点按栅格扫描顺序选取候选，把栅格索引转换为 `map` 坐标系下的目标位姿。每轮最多预检查 8 个未被排除的候选，同一时间只执行一个导航目标。

```bash
ros2 launch articubot_one bringup_explore.launch.py
```

组合 Launch 按固定延时启动测试房间、SLAM Toolbox、Nav2 和前沿探索节点。SLAM 与 Nav2 各自在独立的 Launch 参数作用域中读取默认配置。需要逐步排查时，可按下文的复现步骤分终端启动。

## 算法与工程改进

- **路径可行性预检查**。通过 `GridBased` 规划器异步调用 `ComputePathToPose`。只有规划成功、路径包含至少两个位姿且终点误差不超过 `0.1 m`，才发送 `NavigateToPose`。
- **Navfn 规划容差**。将 `GridBased.tolerance` 从 `0.5 m` 调至 `0.1 m`。较大的容差允许路径终点偏离原前沿，实验中曾出现导航反复返回成功、探索却没有相应进展的情况。
- **目标冷却**。以单调时钟为准，已尝试目标附近的区域暂时排除 180 秒。导航成功时半径为 `0.45 m`，导航中止或路径预检查被拒时为 `0.65 m`，导航取消、目标请求被拒或其他导航错误时为 `0.60 m`。
- **平面状态估计**。EKF 在 `two_d_mode` 下融合轮速里程计的 `vx`、`vy`、`wz` 与 IMU 的 `wz`。差速里程计提供接近零的横向速度，中继保留原测量并为其赋予暂定方差 `1.0e-5`。EKF 发布 `odom → base_link`。

## 实验结果

历史实验验证了 Gazebo 仿真、传感器接入、EKF 状态估计、SLAM Toolbox 建图、AMCL 定位、Nav2 导航与静态避障、单轮巡航、带路径预检查的前沿探索及地图保存。

其中一次探索实验共发送 **4 次 `NavigateToPose` 请求**，结果为 **3 次 `SUCCEEDED`、1 次 `ABORTED`**。多个不可规划候选在发送导航请求前被过滤。这组计数只描述该次实验。

| 已保存的探索地图 | 结果 |
| --- | ---: |
| 地图尺寸 | 239 × 239 栅格 |
| 分辨率 | 0.05 m/栅格 |
| 空闲 / 占据 / 未知栅格数 | 53,522 / 2,107 / 1,492 |
| 已知 / 未知栅格占比 | 97.39% / 2.61% |
| 剩余前沿栅格 / 八连通区域 | 30 / 12 |

**97.39% 是保存地图矩形范围内的已知栅格比例。** 实验未测定可达区域覆盖率，也未建立探索完成判据。历史快照和统计依据见 [2026-11-26 实验记录](experiments/2026-11-26-verification.md)。

发布前复测使用干净构建和短时的 Fortress 无界面仿真。Nav2 以 `GridBased.tolerance=0.1` 激活，探索节点过滤了不可规划候选，并完成一个导航目标。单独启动定位时，AMCL 成功激活，地图服务器读到了安装目录内的 v2 地图。本轮未复测完整 GUI、整轮巡航或长时间探索。

## 分步复现

1. 按**启动仿真**一节启动测试房间，等待 `/clock`、`/scan` 和 EKF 发布的 `odom → base_link`。
2. 启动 SLAM Toolbox，等待 `/map` 和 `map → odom`。
3. 启动 Nav2。运行 `ros2 param get /planner_server GridBased.tolerance` 确认容差为 `0.1`，再用 `ros2 action list -t` 检查路径规划和导航动作。
4. 在另一终端启动前沿探索节点，观察路径检查、候选拒绝、路径通过、目标发送和导航结果日志。
5. 在可写目录下使用新的文件名保存地图，保存工具会同时写出 YAML 和图像文件。

```bash
ros2 run articubot_one frontier_explorer.py
# 在另一个已加载工作空间环境的终端中保存实时地图
ros2 run nav2_map_server map_saver_cli -f "$PWD/frontier_reproduction" \
  --ros-args -p use_sim_time:=true
```

如需复现已知地图巡航，先停止探索流程，再按前文步骤启动同一个测试房间、AMCL、Nav2 和巡航节点。

## 已知限制

- 探索节点未实现正式的完成判据、前沿聚类或基于路径代价的排序。每轮检查上限和临时空间排除都可能使一部分候选未被访问。栅格扫描跳过地图四周最外侧的行和列。
- 路径通过预检查后，`NavigateToPose` 仍会重新规划，目标的可达性可能变化。Nav2 的导航目标检查器另有 `0.25 m` 的位置容差。
- 组合 Launch 使用固定延时。各组件就绪时间和单次导航结果会随运行环境变化。
- 在 Humble 上中断程序时，前沿探索和协方差中继节点可能出现 `rclpy` 重复关闭错误。
- EKF 使用暂定的测量协方差。横向速度约束假设侧滑可忽略。缺少绝对观测时，仅靠轮速里程计和 IMU 融合仍会漂移。项目验证范围为仿真。

## 致谢与许可证

本仓库保留了 Samer Odeh 的 ROS 2 Autonomous Robot 项目历史及文件中的版权和归属声明，包括 Nav2 Launch 的归属信息。Odeh 的 Apache 2.0 许可涵盖他在自主导航基础实现、配置调参、仿真启动和文档方面的贡献。`docs/` 中的资料描述的是 Odeh 上游版本及其当时的 TF 配置。[上游项目](https://github.com/samerodeh/ros2_autonomous_robot) · [许可证](LICENSE)

基础机器人描述和继承的 Launch 文件源于 Josh Newans 的 Articulated Robotics `articubot_one`。该仓库没有声明仓库级许可证。更早的 `my_bot` 模板采用 Apache 2.0。各文件的许可范围以其来源及原始声明为准。[Articulated Robotics 上游仓库](https://github.com/joshnewans/articubot_one) · [my_bot 模板](https://github.com/joshnewans/my_bot)
