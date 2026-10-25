# universal_slam

ROS 2 Humble 的 LiDAR + IMU 建图和在线局部导航算法（FAST-LIO2 + Nav2 SmacPlanner2D/MPPI），以及 M20 Pro 桥接节点。默认仅建图，不启动运动控制。

## 包结构与松耦合

| 包 | 角色 |
| --- | --- |
| `slam_nav_core` | 算法：FAST-LIO2 参数/launch、odometry_adapter、obstacle_filter、Nav2 配置。与机型无关。 |
| `uslam_backend` | 算法后端：关键帧位姿图回环（GTSAM iSAM2，近邻 + Scan Context 候选，GICP 验证）与已存地图中的重定位；唯一的 map->odom 发布者。与机型无关。 |
| `universal_slam_bringup` | 算法统一入口。按 profile 中的 `bridge.package`/`bridge.launch` 动态包含桥接，不依赖任何机型包。 |
| `m20pro_bridge` | 唯一的机型相关包：M20 Pro profile、传感器归一化（m20_topic_bridge）、cmd_vel→UDP 轴指令（m20_cmd_vel_bridge）。 |

迁移到其他机器人：新写一个桥接包（profile + 传感器归一化 + cmd_vel 执行），算法包不改。契约见 docs/BRIDGE_INTERFACE.md；`universal_slam_bringup` 的测试会拒绝算法包中出现机型引用。

## 当前边界

- 已修复：M20 JSON 报文解析、控制默认禁用/人工解锁/状态与里程计超时、速度上限与硬件满量程分离、RoboSense 字段转换、Nav2 参数树、TF 所有权、LIO 速度输出、完整地图保存、LIO 发散锁止、随机器人高度的障碍地面切割。
- `m20pro.yaml` 的 calibration_verified=false。use_nav=true 或 enable_control=true 会拒绝启动，必须先完成实机标定、包络与 base_height 验收。
- M20 Pro 原厂 rslidar 驱动已把前后雷达合并到机身中心帧 lidar_link，因此 profile 的雷达位姿为零，IMU 取官方 MJCF imu_site (0.0632, -0.0268, +0.0435)。
- LIO 发散（速度或位置跳变越界）时 odometry_adapter 锁止并停止发布 odom/TF，需人工调用 `/odometry_adapter/reset`；下游 Nav2 与桥接因里程计超时停车。阈值取 profile 的 `odometry_guard`（机器人在任何驾驶方式下的物理上限，含建图时原厂遥控），不再由 Nav2 限速推导；旧推导会在 1.5 m/s 手动建图时误锁止。
- FAST-LIO 保持上游数值行为（camera_init = 开机 IMU 姿态），只额外发布初始化时估计的重力；odometry_adapter 据此定义 odom：z 与重力相反、x 为机身起始朝向的水平投影、原点为机身起点。开机倾斜或 IMU 倒装都不再让 odom 倾斜/翻转，偏航也是确定的。启动后约 1 s 内机器人必须静止。
- 障碍链路（地形感知）：obstacle_filter 在重力对齐的 odom 系维护滚动 2.5D 高程图（0.1 m 格，取格内最低回波，两次观测确认；机器人再走 5 m 或 60 s 未观测即遗忘，高度只做局部比较，LIO 高度漂移相互抵消）。
  - 竖直障碍 → /uslam/obstacles → obstacle_layer（普通标记 + 射线清除）：当前帧中高出所在格地面 0.15–2.0 m 的点（墙、人、箱体侧面）；尚未确认的格以本帧该格最低回波为地面，初次看到的坡面不会被当成墙。
  - 地形标记 → /uslam/terrain_obstacles → terrain_layer：
    - 台阶边缘：与视角无关的坡度/台阶检验，坡度 ≤ `max_slope_deg`(15°) 的坡道从任意方向可通行；高差 > `min_step_height`(0.05 m) 的边缘只标较高的一格（路沿、坡道侧面、平台/台阶边缘）。2.5 m 以外只比较观测时间相差 ≤ 0.3 s 的相邻格：不同帧的建图姿态残差不同（仿真实测 0.5–1.6°），5 m 外跨帧拼接会出现 5–9 cm 的伪台阶。
    - 落差：沿雷达射线在高程图上行走，低于当前地面 `negative_drop`(0.15 m) 的格（坑、下行楼梯、平台边缘外），以及它与上一块地面之间本应可见却没看到的格；落差之后低处成为新的地面基准（站在平台上看，下面的地面只在边缘处算落差）。射线经过遮挡物（台阶上沿、竖直障碍）后，在重新回到地面之前不判落差：箱子后面的阴影不是坑，远处地面因建图姿态残差（足式步态晃动）略低也不算；被遮挡的坑等直接看到时再标记。落差被记忆，直到清除射线扫过或机器人站上去，所以坑沿进入近距盲区后仍被标记，滚动的局部代价图也不会丢。
  - 清除射线 → /uslam/clear_rays：从雷达出发，只经过连续已观测、无竖直障碍的地面，遇到落差、台阶、障碍或未观测间隙就停止。
    - terrain_layer 只标记，只由清除射线清除：地形标记只有在原处重新看到地面时才会消失，到墙后或坑对岸的射线擦不掉它们。
    - obstacle_layer 也接入清除射线：开阔处后面没有障碍可供射线清除的残留标记（走开的行人）也能被清掉。
  - costmap 的层级与观测源高度门限都已放开，标记距离 8 m（Humble 默认 2.5 m）。
- 回环与重定位由 uslam_backend 提供（见下）。不开后端（use_backend:=false）时 map->odom 为静态恒等，LIO 高度漂移（仿真 200 s 内 0.1–0.2 m）不被修正，回访处的二维栅格会出现多层地面伪障碍。Nav2 仍只用实时滚动代价地图，不加载静态地图层。
- 重定位依赖场景几何。全局重定位要求 GICP 内点率 ≥ `reloc_min_inlier_ratio`（0.85）：仿真中正确结果均为 1.00，而相似房间里的错误结果（LOST 后进入未建图的相邻房间、点对称房间按建图原点猜测）有 0.67–0.69，旧阈值 0.5 曾接受它们。因此在建图边界或未建图区域保持 LOST，直到机器人回到建图区域。每次尝试验证 Scan Context 排名靠前、彼此相距 ≥ 1 m 的最多 15 个候选（只取前 5 名时，对称孪生位置附近的关键帧曾挤掉真实位置）；两个位姿明显不同的候选同样好时报 AMBIGUOUS，需要更多运动或 RViz 2D Pose Estimate（/initialpose）给初值。几何对称的场景无法保证：如果地图在真实位置附近覆盖差、而在对称孪生位置附近覆盖好，仍可能直接定位到孪生位置（仿真对称房间 B 起步：误差 14 m、偏航差 180°，唯一的不对称物体只占视野 2% 的点）。对称或重复的场地应给 /initialpose，或从建图起点启动，并让建图覆盖均匀。每次全局尝试需对最多 16 个候选做 GICP，仿真中（WSL 桌面 CPU）耗时 1–6 s，只在 INITIALIZING/LOST 时运行。
- 地形只做 2D 可通行性判断，不是爬楼梯/崎岖地形规划器：楼梯、坑、平台边缘都是禁行区，坡道按坡度放行。落差检测受雷达视场限制：最低线束够不到的近距盲区不做推断（每个方位按实际观测到的最低线束计算；仿真 32 线、最低 −25°、离地 0.6 m 时约 1.3 m），靠高程图记忆和代价地图保留在盲区外看到的标记。从未在盲区外看到过的落差检测不到，例如开机就停在陡坡顶或台边。窄于约 0.2 m 的沟缝、浅于 0.15 m 的落差不算负障碍；远处的坑只有掠射深度超过 0.15 m 才能检出（仿真中 1.5 m 宽的坑约 5 m 内）。
- 全局代价图为 40 m 实时滚动窗口，允许规划穿过未观测区域，障碍由实时点云阻挡。
- 全局规划使用的半径 = `robot_radius` + MPPI `collision_margin_distance`（0.05 m）+ 0.05 m 采样余量，只规划控制器愿意进入的通道。仅按机身半径规划时，规划器会选中心线紧贴 MPPI 临界区的窄缝，机器人停在缝口不动，也不绕行。按当前 0.55 m 圆形包络，可通行宽度约需 1.3 m，再加上障碍标记的格化，实际间距约 1.5 m 以下的缝会绕行或不可达。实测机身包络后应按实际尺寸更新 profile。

## 构建

在 Ubuntu 22.04 + ROS2 Humble 环境执行：

    source /opt/ros/humble/setup.bash
    bash scripts/setup_workspace.sh
    source install/setup.bash

脚本导入锁定依赖、初始化 ikd-Tree 子模块、应用 Livox 可选依赖补丁与 runtime 补丁，再扫描全部一方/第三方包依赖。runtime 补丁必须保留在仓库中，不能仅手工改 src/FAST_LIO_ROS2。已有旧版补丁的源码树需先恢复上游再重打：

    git -C src/FAST_LIO_ROS2 checkout -- . && bash scripts/patch_fastlio_livox.sh && bash scripts/patch_fastlio_runtime.sh

仓库通过 .gitattributes 强制 LF；在 Windows 检出后拷入 Linux 构建也不会出现 CRLF 导致的 shell/shebang 失败。路径含非 ASCII 字符时 rosidl 会失败，请在 ASCII 路径下构建。

## 运行

    P=$(ros2 pkg prefix m20pro_bridge)/share/m20pro_bridge/config/m20pro.yaml
    ros2 launch universal_slam_bringup universal_slam.launch.py robot_config:=$P use_bridge:=true

- `robot_config:=`（必填）：机器人 profile 绝对路径，由桥接包提供。
- `use_bridge:=true`：启动 profile 指定的桥接（M20 Pro 默认只启动传感器桥）。
- `use_nav:=true`：启动 Nav2 与障碍分支，需已验证 profile 与 `navigation.base_height`。
- `enable_control:=true`：仅允许非仿真、已验证 profile，且要求桥接自身的满量程标定与 expected_gait；启动后仍为 DISARMED。
- `slam_mode:=mapping|localization`（默认 mapping）、`map_dir:=/absolute/dir`（默认 ~/.ros/universal_slam_map）、`use_backend:=true`（默认）。

### 建图（带回环）与重定位

    # 建图：后端做关键帧回环，map->odom 随优化更新；完成后保存（地图目录 + 栅格）
    ros2 launch universal_slam_bringup universal_slam.launch.py robot_config:=$P use_bridge:=true map_dir:=/data/site_a
    ros2 run slam_nav_core save_map.py map2d --convert
    # 之后在同一场地任意位置启动并定位（可选 RViz 2D Pose Estimate 给初值）
    ros2 launch universal_slam_bringup universal_slam.launch.py robot_config:=$P use_bridge:=true slam_mode:=localization map_dir:=/data/site_a

仿真大场景（6×4 个相似房间、273 m、563 s，实时回放）：315 个关键帧，接受 31 个近邻回环，Scan Context 在相似房间之间提出的 105 个错误回环全部被漂移检验拒绝；关键帧位置误差 RMSE 0.16 m（纯里程计 0.44 m），高度 0.04 m（纯里程计 0.20 m）；后端内存 56→108 MB，回环检测每次 ≤ 120 ms，地图发布 ≤ 74 ms；FAST-LIO 内存 135–149 MB。以上为 WSL 桌面 CPU 上的仿真结果，伴随计算机上需实测。

定位状态在 /uslam/backend/status（MAPPING / INITIALIZING / LOCALIZED / LOST / AMBIGUOUS，latched）。LOCALIZED 之前不发布 map->odom，Nav2 的全局规划因此等待；跟踪失败超过 lost_timeout_s 进入 LOST 并自动重新做全局定位。后端参数见 uslam_backend/config/backend.yaml。

M20 Pro 优先在经授权可获取传感器的 GOS/伴随计算机部署。原厂文档限制点云外发，先取得厂商支持；不要改动 AOS 底层运动系统或覆盖 /opt/robot。隔离原厂 localization、planner 与自研 TF/命令；未经人工确认，不停/禁用任何原厂服务。

M20 UDP 后端不自动切模式、起立、切步态或抢回控制权。操作员须先确认常规/RL状态、未急停/未充电、正向和已标定步态，并有新鲜里程计。之后可人工调用 /m20_cmd_vel_bridge/arm（std_srvs/srv/SetBool）。状态/里程计超时、NaN 或错误立即解除武装；恢复数据不自动恢复运动。cmd_vel 指令流过期（到达目标、Wait 行为、在 Nav2 发指令前先解锁）只发零速并保持武装，下一个指令继续执行；连续 idle_disarm_s（默认 30 s）无指令才解除。退出只提供尽力零速，真实断网/进程崩溃停止仍需厂商看门狗和物理急停验收。/NAV_CMD SI 单位后端尚未实现。

## DDS

使用 CycloneDDS 且 ROS_LOCALHOST_ONLY=1 时必须加载仓库配置：默认参与者索引上限不足以容纳完整建图+Nav2 栈；并且配置把 MaxMessageSize 限到 1400 B。Nav2 障碍层以 best-effort 订阅 /uslam/obstacles，在会丢大 UDP 包的环境（WSL 镜像网络回环、MTU 1500 链路）默认配置下多分片样本整条丢失，代价地图一直为空；可靠链路的大点云也会退化为逐分片重传而成批丢帧。实测：默认配置 0/60、本配置 58/60 条点云送达。

    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI=file://$(ros2 pkg prefix slam_nav_core)/share/slam_nav_core/config/cyclonedds_localhost.xml

FastDDS（Humble 默认）无参与者数限制，但上述 best-effort 大消息问题需自行在目标机上用 `ros2 topic hz --qos-reliability best_effort /uslam/obstacles` 确认。

## 官方 bag

来源：DeepRoboticsLab/lightning-lm-deep-robotics 的 M20 Dataset（https://github.com/DeepRoboticsLab/lightning-lm-deep-robotics）。整包 8301 条 /IMU、452 条 /LIDAR/POINTS，约 45.3 秒；PointCloud2 为 XYZI float32、ring uint16、timestamp float64（绝对秒），point_step=26，ring 0..191，点云 frame 为 lidar_link，IMU frame 为空。存在 53 次点云消息头时间不递增，桥接使用逐点时间确定帧首，并拒绝非单调扫描。离线 profile：`m20pro_bridge/config/m20_official_bag.yaml`。

    python3 scripts/inspect_bag.py <bag>/lidar_data_bag_0.db3
    python3 scripts/replay_smoke.py <bag_dir> build/bag_test --rate 0.5 [--verify-grid]

该脚本强制 ROS_DOMAIN_ID=221、ROS_LOCALHOST_ONLY=1，只播放两个传感器话题，控制始终关闭。

## TF 和话题

    uslam/map -> uslam/odom -> uslam/base_link -> uslam/imu
                                            -> uslam/lidar

map->odom 只由 uslam_backend 发布（建图时为回环修正，定位时为扫描-地图配准结果；use_backend:=false 时为静态恒等），时间戳与对应的 odom 相同。odom 与重力对齐、原点在机身起点；odom->base_link 只由 odometry_adapter 发布；odom->uslam/lio_world（= FAST-LIO camera_init）为 adapter 发布的静态变换。原生 FAST-LIO camera_init->body 只留在 /uslam/lio/tf 调试通道。输入由桥接转换为 /uslam/points_raw、/uslam/imu_raw；LIO 输出 /uslam/lio/odometry，适配后输出 /uslam/odom。Nav2 命令使用 /uslam/cmd_vel。外参方向为 T_IMU_LIDAR = inverse(T_BASE_IMU) * T_BASE_LIDAR。障碍分支从去畸变的 /uslam/lio/cloud_registered 提取（经 odom<-lio_world 转到重力对齐的 odom），以 `odom 中 base 高度 - base_height` 作为机身脚下的地面，发布 /uslam/obstacles、/uslam/terrain_obstacles、/uslam/clear_rays。

## 保存地图

    ros2 run slam_nav_core save_map.py                      # 仅地图
    ros2 run slam_nav_core save_map.py my_map --convert --out-dir /absolute/output   # 地图 + Nav2 栅格(PGM/YAML)

后端运行时，客户端调用 /uslam_backend/save_map，把回环优化后的地图目录写到 map_dir（map.pcd 为 map 系全局地图，keyframes.txt 与 keyframes/*.pcd 供重定位使用；旧目录保留为 .previous），栅格默认也写入 map_dir。没有后端时退回 FAST-LIO 的原始 PCD（camera_init 系），转栅格前先经 odom<-lio_world 旋到重力对齐的 odom。以下为 FAST-LIO 路径的细节：客户端查询运行中 mapper 的 map_file_path，检查服务结果、文件更新和非零点数；体素缓存容量不足会拒绝保存。地图由每帧去畸变的完整扫描（point_filter_num 抽稀后）按 pcd_save.voxel_size 累积，不再用 0.5 m 体素质心。栅格转换按 1 m 网格估计局部地面（低分位数 + 邻域最小），把高出局部地面 `--min-height`(0.15)～`--max-height`(2.0) m 的点记为障碍，因此随距离缓慢变化的 LIO 高度漂移不会把地面画成障碍；但同一区域在不同高度被回访时会形成多层地面（见当前边界）；不再需要手工给地面高度，也不再依赖 pcd2pgm。

## 测试

    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon test --packages-select m20pro_bridge slam_nav_core universal_slam_bringup uslam_backend --return-code-on-test-failure
    colcon test-result --verbose

证据与历史修复见 docs/。PLAN.md 是历史设计记录，其中 SW01 与旧的接入假设已被本说明取代。
