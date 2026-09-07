# universal_slam

ROS 2 Humble 的 LiDAR + IMU 建图和在线局部导航集成。默认仅建图，不启动运动控制。

## 当前边界

- 已修复：M20 JSON 报文解析、控制默认禁用/人工解锁/状态与里程计超时、速度上限与硬件满量程分离、RoboSense 字段转换、Nav2 参数树、TF 所有权、LIO 速度输出和完整地图保存。
- 官方 M20 bag 已用于本机离线链路测试。bag 身份是 M20，不能据此把当前 M20 Pro 的物理外参标为已验证。
- 默认 profile 的 calibration_verified=false。use_nav=true 或 enable_control=true 会拒绝启动，必须先完成实机标定、包络与地面基准验收。
- 没有新增回环/重定位后端；地图保存不等于重新启动后可在任意位置导航。
- Nav2 当前只面向平整地面；不具备楼梯、沟槽或负障碍可通行性判断。
- SW01 没有可用 SDK 的实机验证，保持传感器模式，禁止自动申请控制权。

## 构建

在 Ubuntu 22.04 + ROS2 Humble 环境执行：

    source /opt/ros/humble/setup.bash
    bash scripts/setup_workspace.sh
    source install/setup.bash

脚本导入锁定依赖、初始化 ikd-Tree 子模块、应用 Livox 可选依赖补丁与 runtime 补丁，再扫描全部一方/第三方包依赖。runtime 补丁必须保留在仓库中，不能仅手工改 src/FAST_LIO_ROS2。

已有本机 WSL checkout：/home/bernardx526/codex-reviews/universal_slam。

## 官方 bag

来源：DeepRoboticsLab/lightning-lm-deep-robotics 的 M20 Dataset。

    https://github.com/DeepRoboticsLab/lightning-lm-deep-robotics
    https://drive.google.com/drive/folders/19T__ai6u5WCTwyWi3L4KC9gVZI-jiQM-

本机文件：/home/bernardx526/codex-reviews/datasets/m20-official。

整包检测：8301 条 /IMU，452 条 /LIDAR/POINTS，约45.3秒。PointCloud2 为 XYZI float32、ring uint16、timestamp float64，point_step=26；timestamp 是绝对秒，ring 范围0..191。点云 frame 为 lidar_link，IMU frame 为空。存在53次点云消息头时间不递增，适配器使用逐点时间确定帧首，并拒绝非单调扫描。

只读检查，不发布任何数据：

    python3 scripts/inspect_bag.py /home/bernardx526/codex-reviews/datasets/m20-official/lidar_data_bag_0.db3

一键隔离回放测试，只播放两个传感器话题，控制始终关闭：

    python3 scripts/replay_smoke.py /home/bernardx526/codex-reviews/datasets/m20-official build/bag_test --rate 0.5

同时测试 PCD -> 栅格转换（-1.2m 是该参考测试的地面假设，不是实机通用参数）：

    python3 scripts/replay_smoke.py /home/bernardx526/codex-reviews/datasets/m20-official build/bag_grid_test --rate 0.5 --verify-grid

该脚本强制 ROS_DOMAIN_ID=221、ROS_LOCALHOST_ONLY=1，默认使用已安装的 CycloneDDS；不修改系统网络设置、不启用厂商服务、不启动控制节点。可在进程环境选择 RMW_IMPLEMENTATION。

手动回放须在两个终端使用同一隔离 domain/RMW，且不得回放控制话题或原厂 TF：

    export ROS_DOMAIN_ID=221 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    ros2 launch universal_slam_bringup universal_slam.launch.py robot:=m20_official_bag use_adapter:=true use_sim_time:=true

另一个终端：

    ros2 bag play /home/bernardx526/codex-reviews/datasets/m20-official --clock --rate 0.5 --topics /IMU /LIDAR/POINTS

## TF 和话题

    uslam/map -> uslam/odom -> uslam/base_link -> uslam/imu
                                            -> uslam/lidar

map->odom 在当前在线建图模式下为静态恒等；odom->base_link 只由 odometry_adapter 发布。原生 FAST-LIO camera_init->body 只留在 /uslam/lio/tf 调试通道，不进入全局 /tf。

输入由适配器转换为 /uslam/points_raw、/uslam/imu_raw；LIO 输出 /uslam/lio/odometry，适配后输出 /uslam/odom。Nav2 命令使用 /uslam/cmd_vel。外参方向为 T_IMU_LIDAR = inverse(T_BASE_IMU) * T_BASE_LIDAR。

LIO 的原始输入保留逐点时间；障碍分支从去畸变的 /uslam/lio/cloud_registered 提取平地障碍，发布 /uslam/obstacles，不能把它送回 LIO 当作原始点云。

## Profile 和部署

- robot:=m20 / m20pro：现场标定模板，不是已验证配置。
- robot:=m20_official_bag：官方包的离线参考，使用官方示例的单位外参假设，不能直接启用实机导航。
- robot_config:=/absolute/profile.yaml：加载指定 profile。
- use_adapter:=true：启动传感器；默认不会启动 M20 控制桥。
- use_nav:=true：启动 Nav2 与障碍处理，需已验证 profile 和明确的 ground_z。
- enable_control:=true：仅允许非仿真、已验证 M20 profile，并要求 control 中满量程标定与 expected_gait；启动后仍为 DISARMED。

M20 Pro 优先在经授权可获取传感器的 GOS/伴随计算机部署。原厂文档限制点云外发，先取得厂商支持；不要改动 AOS 底层运动系统或覆盖 /opt/robot。隔离原厂 localization、planner 与自研 TF/命令；未经人工确认，不停/禁用任何原厂服务。

M20 UDP 后端不自动切模式、起立、切步态或抢回控制权。操作员须先确认常规/RL状态、未急停/未充电、正向和已标定步态，并有新鲜里程计。之后可人工调用 /m20_cmd_vel_bridge/arm（std_srvs/srv/SetBool）。任何超时或错误解除武装；恢复数据不自动恢复运动。退出只提供尽力零速，真实断网/进程崩溃停止仍需厂商看门狗和物理急停验收。

/NAV_CMD SI 单位后端尚未实现；不要把 UDP 归一化指令与该话题混用。现有 UDP 满量程默认0，故无法在未知标定时启用控制。

## 保存地图

    bash scripts/save_map.sh
    bash scripts/save_map.sh my_map --convert --ground-z=-1.2 --out-dir /absolute/output

客户端查询运行中 mapper 的 map_file_path，检查服务结果、文件更新和非零点数。默认仅保存 PCD；转换栅格必须显式给出地图坐标系下的地面高度。体素缓存有容量限制，容量不足会拒绝保存，而不是宣称部分地图已完整保存。

## 测试

    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon test --packages-select adapter_m20 adapter_sw01 slam_nav_core universal_slam_bringup --return-code-on-test-failure
    colcon test-result --verbose

    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp python3 scripts/probe_nav_parameters.py "$PWD" build/nav_parameters.json

证据和未完成项见 docs/REPAIRS_2026-09-07.md。PLAN.md 是历史设计记录，其旧的自动控制/零配置直连/地图能力声明已被本说明取代。
