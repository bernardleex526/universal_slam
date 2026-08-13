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

## 当前阶段：P0 安全修复 + P1/P2 契约项完成，核心栈与适配层待实机验证

已实现（本仓库代码，构建/单测在 WSL Ubuntu-22.04 + Humble 验证）：

- ✅ 单仓库多包骨架、依赖锁定（FAST_LIO_ROS2@2fffc57、pcd2pgm@d3654a2）
- ✅ 适配器安全基座（P0/P1）：
  - `adapter_m20` / `adapter_sw01` 的 cmd_vel 在归一化/除法前拒绝 NaN/Inf（非法指令触发看门狗零速）
  - 启动参数 fail-fast：`vx_max/vy_max/wz_max`、频率、端口等非法（非有限/非正/超范围）直接退出
  - SW01 UDP 点云解析为长度驱动纯函数：截断/残留/尾错/陈旧缓冲全部拒绝，`dynamic`/`fixed` 两种协议布局显式选择
  - SW01 控制默认 **30Hz**（SDK 契约 ≥20Hz 校验保留，为 50ms 看门狗留调度余量）
  - M20 ErrorHalt：每 tick 持续零速；有界自动恢复（默认关，证据不足不猜协议）+ 人工复位服务
    `ros2 service call /m20_cmd_vel_bridge/reset_error std_srvs/srv/Trigger {}`
  - 单测：`colcon test` 可执行（gtest 纯函数测试，不依赖 SDK/硬件）
- ✅ FAST-LIO launch 数字参数原生 int/float（不再字符串注入，契约测试锁定）
- ✅ 外参接线（P1）：RPY（度）→ 四元数静态 TF；`extrinsics.imu/lidar` 换算为
  FAST-LIO `mapping.extrinsic_T/R`（lidar→IMU）注入——**数值为配置初值/手册值，
  实机标定（P4）前不代表真实外参**
- ✅ `use_adapter` 真正接线：`use_adapter:=true` 启动所选机型 adapter launch；缺 SDK/可执行时 fail-fast
- ✅ launch 契约测试（slam_nav_core / universal_slam_bringup）
- ✅ Nav2 依赖修正：`nav2_recoveries` → `nav2_behaviors`（Humble），config/launch 名称一致

**未实机验证（P4）**：SW01 SDK 调用与点云协议布局（`cloud_packet_layout` 待抓包确认）、
M20 UDP JSON 帧布局与控制序列（含 ErrorHalt 恢复的协议前提）、外参标定、FAST-LIO 实际建图/导航效果。

**地图管线未实现**：当前 launch **不发布 `/map`、不加载静态地图**（Nav2 static layer 保持禁用），
`scripts/save_map.sh` 为占位。当前导航模式 = FAST-LIO2 持续建图 + 实时点云代价地图。
D9（PCD 留档 → pcd2pgm 栅格 → 静态层）仍未实现，**不要将本仓库当前状态视为"一键全链路建图导航"**。

### 点云字段契约（不伪造数据）

- SW01 协议不提供 ring/逐点 time → `/points_raw` 仅 `x,y,z,intensity`。锁定版 FAST-LIO
  （commit 2fffc57）`lidar_type=2` 在缺 time 字段时走 yaw 角时序回退（`given_offset_time=false`），
  缺 ring 时全部点归入 layer 0 —— 与现有字段真实兼容，适配器不生成伪 ring/time。
- M20 话题桥原样转发机载点云；`timestamp_unit` 必须与机载驱动 time 字段单位一致（待实测）。

### 桥接不是零配置

M20 机载（Foxy）→ 算法主机（Humble）的数据接入需要显式配置，**不存在开箱直连**：

- 同网段直连 DDS：两端 `ROS_DOMAIN_ID` 一致 + RMW 一致（Fast-DDS）+ 防火墙放行 + **有线千兆**（96 线点云 ≈ 240–720 Mbps，WiFi 实测会掉到 ~2Hz）
- 跨网段/WiFi：两端部署同版本 `zenoh-bridge-ros2dds`（0.11.0-stable），且需先降采样（本仓库尚未实现 voxel_downsample）
- `use_adapter:=true` 只启动 `m20_topic_bridge`（frame_id/QoS 整形）+ `m20_cmd_vel_bridge`（UDP 控制），不解决网络可达性

详见 [PLAN.md](PLAN.md)。
