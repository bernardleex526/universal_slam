# adapter_sw01 — SW01（璇玑动力 Hypertron-SW01）适配器

## 背景

SW01 无 ROS 支持，通过 **Daedalus 专有 C 库**（`libASTRALL_SDK.so`，linux x86_64/ARM64）对接。本包为 ROS2 节点封装：

```
Daedalus SDK ──IMU(250Hz)──> adapter ──/imu_raw──> FAST-LIO2
            ──点云(UDP 6100)──> adapter ──/points_raw──┘
Nav2 /cmd_vel ──> adapter ──AstrallMove(vx, vy, vyaw)──> SW01 底盘
```

## SDK 接口清单（interface.h）

| 接口 | 用途 | 适配器调用 |
|------|------|-----------|
| `AstrallSdkInit(AstrallConfig&, timeout)` | 初始化，注册心跳/状态回调 | init 时 |
| `AstrallSdkDeinit()` | 反初始化 | 析构时 |
| `AstrallHeartbeat(timeout)` | 心跳 | 10Hz 定时器 |
| `AstrallAuthControl(ASTRALL_AUTH_SDK)` | 申请控制权（遥控器可抢占） | 5s 重试定时器 |
| `AstrallSportModeControl(ASTRALL_SPORT_MODE_CMD_MOVE)` | 切行走模式（RL 步态全地形） | 拿到控制权后 |
| `AstrallSubscriptionData(IMU, 250HZ, cb)` | 订阅 IMU 数据 | init 时 |
| `AstrallMove(vx, vy, vyaw, timeout)` | 移动控制 | ≥20Hz 控制定时器 |
| `AstrallGetSystemStatus / GetSportStatus / GetPowerStatus` | 查询（可选） | 未用（靠 sdkStatusCb 回调） |

返回码：`ASTRALL_RES_SUCCESSED(0x8008)` 成功；`ASTRALL_RES_BEEN_OBTAINED(0x8021)` 已有设备持有控制权；`ASTRALL_RES_WITHOUT_AUTH(0x8022)` 无控制权；`ASTRALL_RES_RC_NORELEASE(0x8020)` 遥控器未释放。

## 控制要点

- **控制权**：连接成功后默认无控制权，须先 `AstrallAuthControl` 申请；`sdkStatusCb` 中监控 `ctrlAuthority`，丢失（遥控器抢占）→ 停止移动，每 5s 重试申请。
- **运动模式**：拿到控制权后须 `AstrallSportModeControl(MOVE)` 才能移动。
- **50ms 看门狗**：机器人 50ms 无移动指令自动停止 → 适配器控制定时器 **≥20Hz**，且 50ms 内未收到新 `/cmd_vel` 时下发 `AstrallMove(0,0,0)`。
- **心跳 10Hz**：500ms 无心跳进失控保护。
- **速度映射**：`cmd_vel(m/s, rad/s) / vx_max|vy_max|wz_max → 归一化 [-1,1]`（SDK 参数范围 `vx/vy/vyaw ∈ [-1,1]`）。

## 点云协议（UDP 6100）

```
AstrallPosCloudData:
  uint16 head        = 0xAA55
  uint64 timestampNs
  uint32 total       // 当前帧点云总数
  uint32 index       // 当前包起始点索引
  uint16 posNum      // 当前包点数 (≤50)
  AstrallPosCloudPosData pos[50]
  uint16 tail        = 0xFF00
AstrallPosCloudPosData:
  int32 x, y, z      // 坐标（单位待实测，疑似 mm → point_scale=0.001）
  uint32 r, g, b, a
```

**重组逻辑**：单包 ≤50 点、按 `total/index` 收集；缓冲填满 `total` 个点后组帧发布 `PointCloud2`（字段 `x,y,z,intensity` float32，`intensity=(r+g+b)/3`，frame_id=lidar）。

**解析安全（P0 重构 + review 修复）**：`parseCloudPacket`（`include/adapter_sw01/cloud_packet.hpp`）为长度驱动纯函数，只读取 `[0, n)` 字节——短包不会读到 UDP 缓冲中的陈旧残留字节。**点大小按 `sizeof(CloudPointRaw)` 推导并经 `static_assert` 锁定为 28 字节**（手册 5.10 节：int32 x,y,z + uint32 r,g,b,a，7×uint32）。两种协议布局（启动参数 `cloud_packet_layout`）均要求**包长精确匹配**，截断/残留/头尾错误一律丢弃：

| 布局 | 接受规则 |
|------|---------|
| `dynamic`（默认） | `n == 20 + posNum×28 + 2`（头 20B + 点 + 尾 2B） |
| `fixed` | `n == 1422`（完整固定结构体：头 20 + 50×28 点 + 尾 2，`posNum` 仅计有效点） |

实机抓包后确认实际布局；若实机为其他填充方式，再按实测调整规则（单测覆盖两种布局）。

**ring/time 契约（不伪造数据）**：SW01 协议不提供 ring 与逐点 time，`/points_raw` 仅含 `x,y,z,intensity`。锁定版 FAST-LIO（commit 2fffc57）`lidar_type=2` 在缺 time 字段时走 yaw 角时序回退（`given_offset_time=false`，`src/preprocess.cpp` `velodyne_handler`），缺 ring 时全部点归入 layer 0——与现有字段真实兼容。适配器启动时打印一次该契约（WARN_ONCE），不生成任何无物理意义的 ring/time 值。

## 测试（P0，无需 SDK/硬件）

```bash
colcon build --packages-select adapter_sw01
colcon test --packages-select adapter_sw01
colcon test-result --verbose
```

- `test_cloud_packet.cpp`：两种布局合法/截断/残留/SOF/TAIL 错误/posNum 越界/full→short 陈旧缓冲/帧一致性
- `test_sw01_math.cpp`：cmd_vel NaN/Inf 除法前拒绝、启动参数 fail-fast

## 编译说明（Ubuntu 部署）

```bash
# 1) 把 SDK 目录拷贝到算法主机（含 include/interface.h 与 lib/linux/<arch>/libASTRALL_SDK.so）
# 2) 设置 ASTRALL_SDK_DIR 后构建
source /opt/ros/humble/setup.bash
colcon build --packages-select adapter_sw01 \
  --cmake-args -DASTRALL_SDK_DIR=/path/to/ASTRALL_SDK
```

未设置 `ASTRALL_SDK_DIR` 时该目标被跳过（便于先验证 ROS2 框架部分）。

## 参数

| 参数 | 默认 | 说明 |
|------|------|------|
| `lidar_port` | `6100` | 点云 UDP 监听端口（监听 0.0.0.0；校验 [1,65535]，非法 fail-fast） |
| `imu_frame / lidar_frame` | `imu / lidar` | TF 帧名 |
| `point_scale` | `0.001` | int32 坐标 → 米（有限正数校验；单位待实测） |
| `vx_max / vy_max / wz_max` | `0.5 / 0.5 / 0.6` | 速度映射分母（有限正数校验），按机型覆盖 |
| `cmd_frequency_hz` | `30` | AstrallMove 频率（P1 默认 30Hz；校验 ≥20 的 SDK 契约，为 50ms 看门狗留余量） |
| `cloud_packet_layout` | `dynamic` | 协议布局 `dynamic`/`fixed`（非法值 fail-fast），见上表 |
| `cmd_vel_topic` | `/cmd_vel` | 输入 |

注：**无 `robot_ip` 参数**（P1 清理死参数）—— 控制经 SDK（SDK 内部维护连接），
点云为入站 UDP，无需机器人 IP。

**cmd_vel 安全（P0）**：含 NaN/Inf 的 Twist 在任何除法/归一化前被拒绝，且不更新指令时间戳——50ms 看门狗随即下发 `AstrallMove(0,0,0)`。所有数字启动参数经 `include/adapter_sw01/sw01_math.hpp` 校验，非法直接抛异常退出（fail-fast）。

## 未决项（P4 实机验证）

0. **字节级 packing 未规定**：手册给出字段顺序但未规定是否 packed —— 当前按
   packed 小端解析（头 20B，点 28B）；自然对齐布局（头含 padding，点起点偏移 28）
   为另一种可能，需实机抓包确认后固定（cloud_packet.hpp 中常量为集中定义，改一处即可）。
1. **IMU 四元数顺序**：`quaternion[4]` 未注明 wxyz/xyzw，现按 `[w,x,y,z]` 假定。
2. **IMU/点云时间戳单位**：疑似纳秒（`timestampNs` 明确为 ns；`AstrallImuData::timestamp` 未注明），暂按 ns 处理。
3. **点云坐标单位**：int32，疑似 mm（`point_scale=0.001`），需实测。
4. **单/双雷达**：系列手册另示双雷达，实机确认后更新 config。
5. **SDK 回调与 rclcpp 线程**：SDK 回调线程发布，必要时（P4 实测）加互斥/队列。
6. **点云 UDP 推送触发**：手册 5.10 节说明订阅 `ASTRALL_SUB_TOPIC_ID_LIDAR` 成功后
   机器人向本机 6100 端口推送点云 —— 适配器 init 时已调用该订阅（触发推送），
   若实机不订阅也推送则此调用无副作用。

## 接口约定

见 `robot_adapters/ADAPTER_INTERFACE.md`（话题、QoS、状态机模板）。
