# 通用 SLAM 系统（通用slam）

## 目标
一套可部署到不同机器人的通用建图 + 导航系统。前提条件：机器人配备 3D 激光雷达 + IMU 即可，**不依赖轮式里程计**。机器人通过各自 SDK 提供数据接入和运动控制。

## 核心组成
- **建图/定位**：FAST-LIO2（激光-惯性里程计，一阶段跑完建图与定位）
- **导航**：ROS 2 Nav2（插件化）
  - 全局规划器：Smac 系列（smac_planner）
  - 局部规划器：MPPI Controller
- **机器人适配层**：SDK 适配器 —— 从 SDK 取数据（点云/IMU），发布标准 ROS 话题；订阅导航 cmd_vel，转换为 SDK 运动指令下发

## 数据流（草案）
```
机器人 SDK ──点云/IMU──> 适配节点 ──PointCloud2/Imu──> FAST-LIO2 ──odom/TF──> Nav2
Nav2 cmd_vel ──> 适配节点 ──SDK 控制接口──> 机器人底盘
```

## 已自动确定（无需再问）
- **必须是 ROS 2**：MPPI 是 Nav2/ROS 2 专属，ROS 1 无 Nav2，无替代方案。
- **Nav2 里程计来源 = FAST-LIO2 输出的 odom**：前提是机器人只有雷达+IMU，没有轮式里程计可选。
- **单机部署、多机型复用**（非多机协同）："通用"指同一套代码换机型可跑，不是多台机器人同时协作。

## 待定决策（TBD，访谈中逐项解决）
- **D1.（已解决）目标机器人与 SDK 形态**：SDK 适配器模式；首批机型调研结论：
  - **云深处 山猫 M20**（轮足四足）：机载跑 **ROS2 Foxy**，上电发布 `/IMU`、`/LIDAR/POINTS`（RoboSense 96 线点云）；**无标准 cmd_vel** —— 控制为 UDP JSON 轴速度指令（归一化 [-1,1] 比例，20Hz），需写小桥把 `cmd_vel` 映射为轴指令；**不能与厂商自带遥控/导航模式并存**。官方 `deep-robotics-msg` 支持 Foxy/Humble/Jazzy。
  - **璇玑动力 Hypertron-SW01**（中型四足，Astrall Dynamics）：专有 C/C++ SDK（Daedalus，`libASTRALL_SDK.so`，linux x86_64/ARM64），**无 ROS 支持** → 适配器必须写代码：
    - 数据：IMU 订阅（`AstrallImuData`，最高 250Hz，accel/gyro/quaternion）；点云经 **UDP 6100** 推送自定义协议 `AstrallPosCloudData`（SOF 0xAA55，int32 xyz+RGBA，单包≤50 点，需按 total/index 重组为完整帧）；另有雷达直连口（10.18.0.120/121，MSOP 6699/6969，标准雷达协议）
    - 控制：`AstrallMove(vx, vy, vyaw)`（标称 m/s、rad/s，实际范围 [-1,+1] 归一化）；**50ms 无指令自动停止** → 适配器须 ≥20Hz 持续发指令、停止时发 (0,0,0)；控制前须申请控制权（遥控器可抢占）；运动模式须切到 MOVE；心跳 10Hz（500ms 无心跳进失控保护）
    - 官方外参初值：雷达 (373.2,0,80)mm、IMU (-1.15,0,59.5)mm（相对质心，SW01 专属手册）；系列手册另给前后双雷达 (444,0,-32)/(-420,0,19) → **实机确认雷达数量与配置**
    - 待确认：点云坐标单位（int32，疑似 mm，需实测验证）
- **D2.（已解决）ROS 2 发行版**：算法主机 **Humble**（Ubuntu 22.04）；M20 机载 Foxy 只做采集，话题桥接（relay/zenoh）到算法主机；SW01 待厂商 SDK 确认（若为 ROS2，Humble 为最低风险交集）。
- **D3.（已解决）导航定位方案**：FAST-LIO2 持续运行（建图即定位），odom 直接给 Nav2，map→odom 恒等 TF，代价地图用实时点云滚动窗口。
- ~~D4.（已取消）~~ 随 D3 选择 FAST-LIO2 持续运行而消失，无需 2D 栅格转换。
- **D5.（已解决）底盘运动模型**：全向（omni）—— M20 轮足、SW01 足式均可横向平移/原地转向；SmacPlanner2D（无运动学约束）+ MPPI 全向配置；每机型最大速度在适配器 config 中映射（M20 轴指令为归一化比例）。
- **D6.（已解决）雷达类型覆盖**：适配层统一输出 `sensor_msgs/PointCloud2`，FAST-LIO2 用 `lidar_type` 参数切换 Livox/通用雷达，不做架构分叉（M20 实测为 RoboSense 96 线 → 通用点云路径）。
- **D7.（已解决）外参标定流程**：静态初值 + FAST-LIO2 在线精估计（`extrinsic_est_en=true`）；每机型 config 存安装位姿初值（**M20 与 SW01 官方手册均已提供外参数值**，可直接作初值）；在线估计不稳的机型可关在线、退纯静态。
- **D8.（已解决）多机型配置管理/仓库结构**：单仓库多包分层结构——
  ```
  universal_slam/
  ├── slam_nav_core/            # FAST-LIO2 + Nav2 配置、launch 模板（与机型无关）
  ├── robot_adapters/           # 适配器接口定义
  │   ├── adapter_m20/          #   M20: 话题桥 + cmd_vel→轴指令桥（UDP JSON）
  │   └── adapter_sw01/         #   SW01: 待 SDK 文档（占位）
  ├── robot_configs/            # 每机型一份 yaml: 外参初值/最大速度/雷达类型/TF
  └── universal_slam_bringup/   # 统一入口: ros2 launch ... robot:=m20
  ```
  接入新机型 = 加一个 adapter 包 + 一份 config。
- **D9.（已解决）地图保存与复用**：FAST-LIO2 保存 PCD 留档 → 离线 pcd2grid 转 2D 栅格（z 高度过滤/地面剔除）→ 作为 Nav2 全局代价地图 **static layer**（obstacle 层仍用实时点云）；定位仍由 FAST-LIO2 持续运行，栅格不参与定位。**约束**：静态地图与 FAST-LIO2 map frame 对齐——部署需同起点启动，或提供 map→odom 偏移参数。

## Resolved Decisions

1. **D1 目标机器人与 SDK 形态** — SDK 适配器模式（每机型一个轻量适配节点，SDK 差异全部收敛在适配器内，核心算法栈与机型无关）。Why: "只要雷达+IMU 就能用"的前提要求适配层完全解耦；若 SDK 本身发布 ROS 话题，适配器退化为 launch+参数零代码。首批机型：云深处 M20、璇玑动力 SW01。(user confirmed)
2. **D1 具体 SDK 事实** — M20：机载 ROS2 Foxy，`/IMU`+`/LIDAR/POINTS`（RoboSense 96 线）话题直接可采，适配 = 话题桥接 + cmd_vel→轴指令桥（归一化比例，20Hz）；SW01：SDK 未公开，待厂商文档（阻塞风险）。(auto-resolved from 联网调研：deeprobotics.us 开发手册 V0.0.4、github.com/DeepRoboticsLab、astralldynamics.com)
3. **D3 导航定位方案** — FAST-LIO2 持续运行，odom 直供 Nav2，代价地图实时点云滚动窗口，不做 2D 栅格。Why: 无轮式里程计前提下 FAST-LIO2 是唯一自洽 odom 源；有 fast-lio-navigation 先例。接受限制：无回环、长时漂移、全局视野限传感器范围。(user confirmed)
4. **D4 地图转换** — 已取消。D3 选择持续定位后无 2D 栅格需求。(derived)
5. **D5 运动模型** — 全向 omni：SmacPlanner2D + MPPI 全向配置。Why: M20（轮足）/SW01（足式）均可平移+原地转向，无差速/阿克曼约束。(auto-resolved from 调研)
6. **D6 雷达类型** — 适配层统一 `sensor_msgs/PointCloud2`，FAST-LIO2 `lidar_type` 参数切换，不设架构分叉。(auto-resolved)
7. **D2 ROS 2 发行版** — 算法主机 Humble；架构两段式：M20 机载 Foxy 采集 → 桥接 → Humble 主机跑 FAST-LIO2 + Nav2。Why: M20 机载固定 Foxy 无法跑 MPPI；官方 sdk_deploy 主机侧即 Humble；FAST-LIO2/Nav2 集成先例以 Humble 为主；SW01 若为 ROS2 大概率 Humble。(user confirmed, 调研佐证)
8. **D7 外参标定** — 静态初值 + 在线精估计结合，config 可关在线退回纯静态。Why: 通用部署无逐台人工标定预算，在线估计是 FAST-LIO2 官方标准路径，静态值兜底。(user confirmed)
9. **D8 仓库结构** — 单仓库多包：core / adapters / configs / bringup 四层，统一 launch 按 `robot:=` 参数选机型。Why: 核心改动一处生效、机型 diff 可见、接入新机型成本收敛为"一个 adapter 包 + 一份 config"。(user confirmed)
10. **D9 地图保存与复用** — PCD 留档 + 离线 pcd2grid 转 2D 栅格，作为 Nav2 全局代价地图 static layer；定位仍由 FAST-LIO2 持续运行，栅格不参与定位。在推荐（v1 仅保存）之上扩展了栅格管线。约束：静态层与 FAST-LIO2 map frame 对齐需同起点部署或 map→odom 偏移配置。(user override)
11. **SW01 SDK 事实（D1 补全，阻塞风险解除）** — 专有 C 库 Daedalus，无 ROS；IMU 订阅 250Hz + 点云 UDP 6100 自定义协议（单包≤50 点需重组）；`AstrallMove` 归一化 [-1,1] 控制，**50ms 无指令自动停止**、须申请控制权（遥控器可抢占）、心跳 10Hz、模式切 MOVE；官方外参初值可用（雷达 (373.2,0,80)mm、IMU (-1.15,0,59.5)mm 相对质心）。待实机确认：点云坐标单位、单/双雷达（系列手册显示前后双雷达）。(auto-resolved from D:\璇玑\技术文档\ASTRALL_SDK_1.0.7\ASTRALL_SDK_1.0.7：interface.h、SW01 手册、系列手册)
12. **实现技术选型（lib-2/lib-3 调研落地）** — FAST-LIO2 用 Ericsii/FAST_LIO_ROS2（ros2 分支）；机械雷达 `lidar_type:2`；`camera_init→body` 动态 TF + 静态 `body→base_link` 补齐 REP-105；MPPI Humble backport 可用（`motion_model:"Omni"`）+ SmacPlanner2D（`angle_quantization_bins:1`）；M20 桥接首选同网段直连 DDS、WiFi 用 zenoh-bridge-ros2dds（同版本 0.11.0-stable）、96 线点云必须有线千兆；PCD→栅格用 pcd2pgm 系列（"pcd2grid" 无同名标准仓库）。(auto-resolved from 调研)
13. **M20 轴指令协议格式（lib-4 落地）** — 16 字节帧头（`EB 91 EB 90` + ASDU 长度 LE + 自增 msgID LE + 0x01=JSON）+ JSON 体 `{"PatrolDevice":{"Type":2,"Command":21,"Time":"YYYY-MM-DD HH:MM:SS","Items":{"X","Y","Z","Roll","Pitch","Yaw"}}}`（归一化 [-1,1]，clamp 2 位小数，Basic/Stair 仅 X/Y/Yaw 生效）。控制序列：心跳(100,100) 1Hz→确保 Regular 模式(1101,5,Mode=0)→起立(2,22,MotionParam=1)→等 MotionState==17→切步态(2,23,GaitParam=0x1001)→轴指令循环 20Hz。无逐帧 ACK，靠主动上报（1002/6 2Hz 状态、1002/4 10Hz 速度反馈、1002/3 错误码门控）。**公开资料无"控制权申请帧"**——以 Regular 模式 + 心跳归属为控制准入，实机验证（钉钉文档需登录，待用户提供）。IP 以 10.21.31.103 为准（SDK 默认 10.21.33.103 为笔误）。(auto-resolved from 调研)
14. **WSL 构建验证（本机 Ubuntu-22.04 + Humble 环境）** — 全部 7 包构建成功：fast_lio、adapter_m20（m20_cmd_vel_bridge + m20_topic_bridge）、adapter_sw01（链接真实 libASTRALL_SDK.so）、pcd2pgm、robot_configs、slam_nav_core、universal_slam_bringup。3 个 launch 解析通过；fastlio_mapping 以本仓库参数文件启动成功（lidar_type 2 生效）；m20 桥节点正常启动。修复记录：① adapter_m20/adapter_sw01 CMakeLists `target_link_libraries` plain/keyword 混用 → 去 PRIVATE；② 三个 main 的 `make_shared` 缺 `NodeOptions()` 参数；③ **FAST_LIO_ROS2 上游无条件依赖 livox_ros_driver2（apt 无此包）** → 新增 `scripts/patch_fastlio_livox.sh`（幂等，WITH_LIVOX_SUPPORT=OFF 默认），`universal_slam.repos` 锁定 FAST_LIO_ROS2 至 commit 2fffc57，`setup_workspace.sh` 自动应用。环境备注：WSL 内 apt 需 root + 清代理（TUNA 镜像直连）；git 克隆走 Windows 侧 curl/tar（宿主代理仅绑定 Windows 回环）。(verified in WSL)

## 实现规划

### 依赖图
```
P0 环境与仓库
 └─> P1 核心栈 slam_nav_core（录包/模拟验证，与机型无关）
 │    └─> P3 Bringup 集成 ─> P4 实机验证与打磨
P2 适配层（adapter_m20 ∥ adapter_sw01，与 P1 并行）───┘
```

### P0 环境与仓库
- Ubuntu 22.04 + ROS2 Humble workspace（可选 Docker）；colcon 构建脚本
- 仓库初始化：universal_slam 单仓库多包（D8 结构），git 初始化
- 依赖锁定：FAST-LIO2 Humble 分支（细节 [待 lib-2]）、Nav2 Humble、PCL

### P1 核心栈 slam_nav_core（无机型依赖）
任务：
1. FAST-LIO2 集成：**Ericsii/FAST_LIO_ROS2（ros2 分支，Humble 推荐，apt PCL1.12/Eigen3.4 即可）**；输入话题 YAML 配置（`lid_topic`/`imu_topic`，约定 `/points_raw`、`/imu_raw`）；**`lidar_type: 2`（VELO16 分支，RoboSense 等机械雷达走标准 PointCloud2）**、`scan_line`、`timestamp_unit` 按 M20 驱动实测；`extrinsic_est_en: true`（初值来自 robot_configs，D7；实测发散则改 false）；`pcd_save.pcd_save_en` + `map_file_path`；输出 `/Odometry`（nav_msgs）→ remap `/odom`，**无独立 map 帧——`camera_init` 即固定世界系**（与 D9 对齐逻辑一致）
2. Nav2 配置：**SmacPlanner2D**（全向，`angle_quantization_bins: 1`、`tolerance: 0.5`、`smooth_path`）+ **MPPI `motion_model: "Omni"`**（Humble 已 backport；`vx/vy/wz_max` 每机型从 robot_configs 映射；`reset_period: 1.0` 为 Humble 特有）；costmap：obstacle 层 `data_type: "PointCloud2"` + `min/max_obstacle_height`（96 线切地面/天花板）+ static 层（`map_subscribe_transient_local: true`、`trinary_costmap`、global 开 `track_unknown_space`）；odom 接入 = TF `odom→base_link` + 各节点 `odom_topic: /Odometry` remap
3. TF 骨架：静态 `map→odom`（恒等/偏移参数，D9 对齐用）→ 动态 `odom→base_link`（**FAST-LIO 出的是 `camera_init→body`，需静态补 `body→base_link`**，nomoto39 方案）→ `base_link→lidar`/`imu`（来自 robot_configs）
4. 地图管线：`pcd_save`（**PointXYZINormal，单位米、camera_init 系；intensity 勿用于判地面/障碍**）→ **pcd2pgm（LihanChen2004，主选：z 窗口 PassThrough + 发布 /map + `map_saver_cli` 出 PGM+YAML，分辨率 0.05）**；地面噪声/斜坡痛点升级 **kzm784/pcd2pgm**（PMF 地面分割 + KNN 相对高度带 h_min/h_max）；备选 octomap_server 2D 投影。⚠️ 帧对齐：FAST-LIO2 全局系为 **camera_init**（非 map）——栅格图必须以同次建图的 camera_init 系输出，导航会话同起点启动即与 FAST-LIO2 当前帧对齐（D9 约束的具体落点）；z 窗口参考 `thre_z_min≈-0.04 / thre_z_max≈0.35`，多层结构需相对地面高度处理
**验证门**：录包/模拟 → FAST-LIO2 出 odom/TF → 导航目标可达 → PCD 保存 → 转栅格 → 重新加载无报错

### P2 适配层 robot_adapters（与 P1 并行，两机型独立）
通用约定：适配器输出 `/points_raw` + `/imu_raw`（sensor_msgs），消费 `/cmd_vel`；共用状态机模板（SDK 初始化 → 申请控制权 → 运动模式 → 指令循环 → 失控/抢占恢复）
- **adapter_m20**：Foxy→Humble 桥接——**首选同网段直连 DDS（同 `ROS_DOMAIN_ID` + 同 RMW，点云/IMU 话题级互通零组件，sensor_msgs 类型 CDR 兼容）**；跨网段/WiFi 用 **zenoh-bridge-ros2dds**（两端同版本，Humble 时代用 0.11.0-stable）；**带宽硬约束：96 线点云 ≈ 240–720 Mbps，必须有线千兆，WiFi 需先降采样/隔帧**；+ `cmd_vel`→轴指令桥（UDP JSON，20Hz，归一化映射，控制权/模式处理）
- **adapter_sw01**：ROS2 节点封装 Daedalus 库——初始化 + 心跳 10Hz、申请控制权、切 MOVE 模式、IMU 订阅 250Hz→`sensor_msgs/Imu`、UDP 6100 点云帧重组→`PointCloud2`（坐标单位实测确认）、`cmd_vel`→`AstrallMove` ≥20Hz + 50ms 看门狗、停止发 (0,0,0)
- **robot_configs**：`m20.yaml` / `sw01.yaml`（外参初值、最大速度映射、雷达类型、TF 帧名）
**验证门**：台架/录包回放：话题数据正确、指令流时序正确（实机放 P4）

### P3 Bringup 集成
- `universal_slam_bringup`：`ros2 launch ... robot:=m20|sw01` 一键全链路；remap 参数接线对齐
- 统一操作接口：建图 / 保存 PCD / 转栅格 / 发导航目标
**验证门**：端到端跑通、导航目标到达、PCD→栅格→静态层生效

### P4 实机验证与打磨
- **M20 实机验收**：上电→桥接→建图→保存→pcd2grid→导航→到达；静态层对齐（同起点）验证
- **SW01 实机**（待硬件/SDK 就绪）：同流程
- 开放确认项：SW01 点云坐标单位、单/双雷达、MPPI 足式调参（振动对代价地图影响）
- 打磨：控制权被抢占恢复策略、导航参数调优

### 里程碑
- M1（P1 完）：核心栈录包跑通建图+导航
- M2（P2 完）：双机型适配器数据/指令正确
- M3（P3 完）：一键 launch 端到端
- M4（P4 完）：≥1 台实机完成"上电→建图→导航→到达"验收

### 实现风险清单
1. SW01 点云坐标单位未知（int32）→ P2 实机实测
2. SW01 雷达数量（单/双雷达手册不一致）→ 实机确认
3. 静态层与 FAST-LIO2 map frame 对齐（同起点假设；FAST-LIO2 全局系实为 `camera_init`，PCD 与之同系）→ P4 验证，预留 `map→odom` 偏移参数
4. 足式机器人点云振动/运动模糊对 FAST-LIO2 与代价地图的影响 → MPPI/代价地图调参
5. M20/SW01 遥控器抢占控制权 → 状态机恢复策略
6. **M20 点云带宽硬约束**：96 线 ≈ 240–720 Mbps，WiFi 实测掉到 ~2Hz → 默认**有线千兆直连 DDS**；仅 WiFi 时降采样/隔帧再发；备选架构：FAST-LIO 直接跑 M20 机载（Foxy 可跑），只桥 `/Odometry` + 降频点云（P4 实测 M20 网络接口后定）

## 当前落地状态（2026-08，与 README「当前阶段」一致）

已落地并在 WSL (Ubuntu-22.04 + Humble) 构建/单测通过（`colcon test` 全绿，不含第三方实机组件）：

- **P0 安全修复**：双适配器 cmd_vel NaN/Inf 除法前拒绝；启动参数 fail-fast 校验；
  SW01 UDP 长度驱动解析（dynamic/fixed 布局精确长度规则，无陈旧缓冲依赖）；
  FAST-LIO launch 数字参数原生类型（string abort 路径已消除，契约测试锁定）；
  `use_adapter` 真正接线（缺 launch/可执行 fail-fast）。
- **P1/P2 契约项**：`nav2_recoveries`→`nav2_behaviors`（依赖/config 名称一致）；
  pcd2pgm 锁定 commit d3654a2；RPY(度)→四元数静态 TF 与 FAST-LIO
  `mapping.extrinsic_T/R`（lidar→IMU）按机型注入（**值为初值/手册值，待实机标定**）；
  SW01 控制默认 30Hz（≥20Hz 契约校验保留）；M20 ErrorHalt 持续零速 + 有界恢复
  （默认关）+ 人工复位服务 `/m20_cmd_vel_bridge/reset_error`；清理死参数 robot_ip。
- **地图管线仍未实现**：不发布 `/map`、static layer 禁用、save_map.sh 为 fail-fast 占位。

仅剩硬件阻断项（P4，无硬件不可推进）：
1. SW01 实机：SDK 接口行为、点云协议实际布局（dynamic/fixed）、坐标单位、单/双雷达
2. M20 实机：UDP JSON 收帧头布局（Type/Command 字节偏移）、控制序列与步态码、
   ErrorHalt 自动恢复的协议前提（是否可无人工干预清除错误）
3. 外参标定（两机型）：RPY 与 xyz 初值替换为实测值
4. 端到端建图/导航验收（rosbag 或实机）
