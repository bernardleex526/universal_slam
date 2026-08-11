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
| `robot_ip` | `10.21.31.103` | 参考（控制走 SDK，IP 为点云源参考） |
| `lidar_port` | `6100` | 点云 UDP 监听端口 |
| `imu_frame / lidar_frame` | `imu / lidar` | TF 帧名 |
| `point_scale` | `0.001` | int32 坐标 → 米（单位待实测） |
| `vx_max / vy_max / wz_max` | `0.5 / 0.5 / 0.6` | 速度映射分母，按机型覆盖 |
| `cmd_frequency_hz` | `20` | AstrallMove 频率（≥20Hz） |
| `cmd_vel_topic` | `/cmd_vel` | 输入 |

## 未决项（P4 实机验证）

1. **IMU 四元数顺序**：`quaternion[4]` 未注明 wxyz/xyzw，现按 `[w,x,y,z]` 假定。
2. **IMU/点云时间戳单位**：疑似纳秒（`timestampNs` 明确为 ns；`AstrallImuData::timestamp` 未注明），暂按 ns 处理。
3. **点云坐标单位**：int32，疑似 mm（`point_scale=0.001`），需实测。
4. **单/双雷达**：系列手册另示双雷达，实机确认后更新 config。
5. **SDK 回调与 rclcpp 线程**：SDK 回调线程发布，必要时（P4 实测）加互斥/队列。

## 接口约定

见 `robot_adapters/ADAPTER_INTERFACE.md`（话题、QoS、状态机模板）。
