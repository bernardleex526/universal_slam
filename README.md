# 通用 SLAM 系统（universal_slam）

一套可部署到不同机器人的通用**建图 + 导航**系统。前提条件：机器人配备 3D 激光雷达 + IMU 即可，**不依赖轮式里程计**。机器人通过各自 SDK 提供数据接入与运动控制。

- 建图/定位：**FAST-LIO2**（激光-惯性里程计，持续运行，建图即定位，输出 `/odom`）
- 导航：**Nav2 (Humble)** —— SmacPlanner2D 全局规划 + MPPI Controller 局部规划（`motion_model: "Omni"` 全向）
- 适配层：每机型一个轻量 SDK 适配器，SDK 差异全部收敛在适配器内，核心算法栈与机型无关

## 数据流

```
机器人 SDK ──点云/IMU──> 适配节点 ──PointCloud2/Imu──> FAST-LIO2 ──odom/TF──> Nav2
Nav2 cmd_vel ──> 适配节点 ──SDK 控制接口──> 机器人底盘
```

约定话题（各层间统一接口）：

| 话题 | 类型 | 帧 | 说明 |
|------|------|----|----|
| `/points_raw` | `sensor_msgs/PointCloud2` | lidar | 原始点云（适配器输出，FAST-LIO 输入） |
| `/imu_raw` | `sensor_msgs/Imu` | imu | 原始 IMU（适配器输出，FAST-LIO 输入） |
| `/odom` | `nav_msgs/Odometry` | odom | FAST-LIO `/Odometry` remap 而来，供 Nav2 |
| `/cmd_vel` | `geometry_msgs/Twist` | — | Nav2 输出，适配器消费转 SDK 指令 |

约定 TF（REP-105 对齐，P1 落地的完整链）：

```
map ──静态──> odom ──FAST-LIO 动态──> body ──静态──> base_link ──静态──> lidar / imu
```

- `map→odom`：静态（恒等/偏移，D9 对齐用）
- `odom→body`：FAST-LIO2 动态 TF（实际为 `camera_init→body`）
- `body→base_link`：静态补齐（FAST-LIO 与 REP-105 的衔接）
- `base_link→lidar`、`base_link→imu`：静态，取自 `robot_configs/<robot>.yaml` 外参初值

## 环境

| 项 | 值 |
|----|----|
| 操作系统 | Ubuntu 22.04 LTS |
| ROS 2 | Humble |
| 上位机与机器人连接 | 有线直连（96 线点云 ≥ 千兆网；WiFi 需降采样） |
| 关键依赖 | vcstool, rosdep, colcon, Nav2, PCL(1.12), Eigen(3.4) |

> M20 机载为 ROS2 Foxy，仅作采集（话题桥接或 zenoh-bridge-ros2dds）；算法主机为 Humble。SW01 无 ROS，走 Daedalus SDK 适配器。

## 构建步骤

```bash
# 首次
source /opt/ros/humble/setup.bash
sudo apt install python3-vcstool python3-rosdep  # 如未安装

# 一键初始化（导入第三方仓库 + rosdep + colcon build）
./scripts/setup_workspace.sh

# 或分步执行：
# vcs import src < universal_slam.repos
# rosdep install --from-paths src --ignore-src -r -y
# colcon build --symlink-install

# 使用前
source install/setup.bash
```

## 目录结构（D8：单仓库多包分层）

```
universal_slam/
├── slam_nav_core/              # FAST-LIO2 + Nav2 配置、launch 模板（与机型无关）
├── robot_adapters/             # 适配器接口定义
│   ├── adapter_m20/            #   M20: 话题桥 + cmd_vel→轴指令桥（UDP JSON）
│   └── adapter_sw01/           #   SW01: Daedalus SDK 适配器（待 SDK 文档）
├── robot_configs/              # 每机型一份 yaml: 外参初值/最大速度/雷达类型/TF
├── universal_slam_bringup/     # 统一入口: ros2 launch ... robot:=m20
├── scripts/                    # setup_workspace.sh / save_map.sh
└── src/                        # vcs import 导入的第三方仓库（FAST_LIO_ROS2、pcd2pgm）
```

接入新机型 = **一个 adapter 包 + 一份 config yaml**。

## 当前阶段：P0 环境与仓库

- ✅ 单仓库多包骨架（本目录）
- ✅ 依赖锁定：FAST_LIO_ROS2（ros2 分支）、pcd2pgm
- ⬜ P1 核心栈 `slam_nav_core`（录包/模拟验证，与机型无关）
- ⬜ P2 适配层（`adapter_m20` ∥ `adapter_sw01`）
- ⬜ P3 Bringup 集成 → P4 实机验证与打磨

详见 [PLAN.md](PLAN.md)。
