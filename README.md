# universal_slam

ROS 2 Humble 的 LiDAR + IMU 建图和在线局部导航算法（FAST-LIO2 + Nav2 SmacPlanner2D/MPPI），以及 M20 Pro 桥接节点。默认仅建图，不启动运动控制。

## 包结构与松耦合

| 包 | 角色 |
| --- | --- |
| `slam_nav_core` | 算法：FAST-LIO2 参数/launch、odometry_adapter、obstacle_filter、Nav2 配置。与机型无关。 |
| `universal_slam_bringup` | 算法统一入口。按 profile 中的 `bridge.package`/`bridge.launch` 动态包含桥接，不依赖任何机型包。 |
| `m20pro_bridge` | 唯一的机型相关包：M20 Pro profile、传感器归一化（m20_topic_bridge）、cmd_vel→UDP 轴指令（m20_cmd_vel_bridge）。 |

迁移到其他机器人：新写一个桥接包（profile + 传感器归一化 + cmd_vel 执行），算法包不改。契约见 docs/BRIDGE_INTERFACE.md；`universal_slam_bringup` 的测试会拒绝算法包中出现机型引用。

## 当前边界

- 已修复：M20 JSON 报文解析、控制默认禁用/人工解锁/状态与里程计超时、速度上限与硬件满量程分离、RoboSense 字段转换、Nav2 参数树、TF 所有权、LIO 速度输出、完整地图保存、LIO 发散锁止、随机器人高度的障碍地面切割。
- `m20pro.yaml` 的 calibration_verified=false。use_nav=true 或 enable_control=true 会拒绝启动，必须先完成实机标定、包络与 base_height 验收。
- M20 Pro 原厂 rslidar 驱动已把前后雷达合并到机身中心帧 lidar_link，因此 profile 的雷达位姿为零，IMU 取官方 MJCF imu_site (0.0632, -0.0268, +0.0435)。
- LIO 发散（速度或位置跳变越界）时 odometry_adapter 锁止并停止发布 odom/TF，需人工调用 `/odometry_adapter/reset`；下游 Nav2 与桥接因里程计超时停车。
- 没有回环/重定位后端；地图保存不等于重新启动后可在任意位置导航。
- Nav2 只面向平整地面；不具备楼梯、沟槽或负障碍可通行性判断。全局代价图为实时滚动窗口，允许规划穿过未观测区域，障碍由实时点云阻挡。

## 构建

在 Ubuntu 22.04 + ROS2 Humble 环境执行：

    source /opt/ros/humble/setup.bash
    bash scripts/setup_workspace.sh
    source install/setup.bash

脚本导入锁定依赖、初始化 ikd-Tree 子模块、应用 Livox 可选依赖补丁与 runtime 补丁，再扫描全部一方/第三方包依赖。runtime 补丁必须保留在仓库中，不能仅手工改 src/FAST_LIO_ROS2。

仓库通过 .gitattributes 强制 LF；在 Windows 检出后拷入 Linux 构建也不会出现 CRLF 导致的 shell/shebang 失败。路径含非 ASCII 字符时 rosidl 会失败，请在 ASCII 路径下构建。

## 运行

    P=$(ros2 pkg prefix m20pro_bridge)/share/m20pro_bridge/config/m20pro.yaml
    ros2 launch universal_slam_bringup universal_slam.launch.py robot_config:=$P use_bridge:=true

- `robot_config:=`（必填）：机器人 profile 绝对路径，由桥接包提供。
- `use_bridge:=true`：启动 profile 指定的桥接（M20 Pro 默认只启动传感器桥）。
- `use_nav:=true`：启动 Nav2 与障碍分支，需已验证 profile 与 `navigation.base_height`。
- `enable_control:=true`：仅允许非仿真、已验证 profile，且要求桥接自身的满量程标定与 expected_gait；启动后仍为 DISARMED。

M20 Pro 优先在经授权可获取传感器的 GOS/伴随计算机部署。原厂文档限制点云外发，先取得厂商支持；不要改动 AOS 底层运动系统或覆盖 /opt/robot。隔离原厂 localization、planner 与自研 TF/命令；未经人工确认，不停/禁用任何原厂服务。

M20 UDP 后端不自动切模式、起立、切步态或抢回控制权。操作员须先确认常规/RL状态、未急停/未充电、正向和已标定步态，并有新鲜里程计。之后可人工调用 /m20_cmd_vel_bridge/arm（std_srvs/srv/SetBool）。任何超时或错误解除武装；恢复数据不自动恢复运动。退出只提供尽力零速，真实断网/进程崩溃停止仍需厂商看门狗和物理急停验收。/NAV_CMD SI 单位后端尚未实现。

## DDS

使用 CycloneDDS 且 ROS_LOCALHOST_ONLY=1 时，默认参与者索引上限不足以容纳完整建图+Nav2 栈，需加载仓库配置：

    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI=file://$(ros2 pkg prefix slam_nav_core)/share/slam_nav_core/config/cyclonedds_localhost.xml

FastDDS（Humble 默认）无此限制。

## 官方 bag

来源：DeepRoboticsLab/lightning-lm-deep-robotics 的 M20 Dataset（https://github.com/DeepRoboticsLab/lightning-lm-deep-robotics）。整包 8301 条 /IMU、452 条 /LIDAR/POINTS，约 45.3 秒；PointCloud2 为 XYZI float32、ring uint16、timestamp float64（绝对秒），point_step=26，ring 0..191，点云 frame 为 lidar_link，IMU frame 为空。存在 53 次点云消息头时间不递增，桥接使用逐点时间确定帧首，并拒绝非单调扫描。离线 profile：`m20pro_bridge/config/m20_official_bag.yaml`。

    python3 scripts/inspect_bag.py <bag>/lidar_data_bag_0.db3
    python3 scripts/replay_smoke.py <bag_dir> build/bag_test --rate 0.5 [--verify-grid]

该脚本强制 ROS_DOMAIN_ID=221、ROS_LOCALHOST_ONLY=1，只播放两个传感器话题，控制始终关闭。

## TF 和话题

    uslam/map -> uslam/odom -> uslam/base_link -> uslam/imu
                                            -> uslam/lidar

map->odom 在当前在线建图模式下为静态恒等；odom->base_link 只由 odometry_adapter 发布。原生 FAST-LIO camera_init->body 只留在 /uslam/lio/tf 调试通道。输入由桥接转换为 /uslam/points_raw、/uslam/imu_raw；LIO 输出 /uslam/lio/odometry，适配后输出 /uslam/odom。Nav2 命令使用 /uslam/cmd_vel。外参方向为 T_IMU_LIDAR = inverse(T_BASE_IMU) * T_BASE_LIDAR。障碍分支从去畸变的 /uslam/lio/cloud_registered 提取，以 `odom 中 base 高度 - base_height` 为地面，发布 /uslam/obstacles。

## 保存地图

    ros2 run slam_nav_core save_map.py                      # 仅 PCD
    ros2 run slam_nav_core save_map.py my_map --convert --out-dir /absolute/output   # PCD + Nav2 栅格(PGM/YAML)

客户端查询运行中 mapper 的 map_file_path，检查服务结果、文件更新和非零点数；体素缓存容量不足会拒绝保存。栅格转换按 1 m 网格估计局部地面（低分位数 + 邻域最小），把高出局部地面 `--min-height`(0.15)～`--max-height`(2.0) m 的点记为障碍，因此长时间 LIO 高度漂移不会把地面画成障碍；不再需要手工给地面高度，也不再依赖 pcd2pgm。

## 测试

    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon test --packages-select m20pro_bridge slam_nav_core universal_slam_bringup --return-code-on-test-failure
    colcon test-result --verbose

证据与历史修复见 docs/。PLAN.md 是历史设计记录，其中 SW01 与旧的接入假设已被本说明取代。
