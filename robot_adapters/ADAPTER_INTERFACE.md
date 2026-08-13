# 适配器接口约定 (ADAPTER_INTERFACE.md)

本文定义所有机型适配器的统一外部接口。核心算法栈（FAST-LIO2 / Nav2）只依赖这些约定，与具体 SDK 无关。

## 1. 数据输出（适配器 → 核心栈）

| 话题 | 类型 | 帧 | QoS | 说明 |
|------|------|----|-----|------|
| `/points_raw` | `sensor_msgs/PointCloud2` | `lidar` | best_effort | 原始点云（不做下采样/变换），FAST-LIO 输入 |
| `/imu_raw` | `sensor_msgs/Imu` | `imu` | reliable | 原始 IMU（accel/gyro/angular/linear，无重力补偿要求） |

- 点云消息头 `frame_id` 必须为 `lidar`，IMU 消息头 `frame_id` 必须为 `imu`（与 bringup 静态 TF 一致）。
- 时间戳必须与主机时钟对齐（M20 直采话题自带；SW01 由适配器打本地时间戳）。
- **字段契约（P0）**：`/points_raw` 至少含 `x,y,z,intensity`；若协议提供 ring/逐点 time 则原样发布，
  **协议不提供时禁止伪造**（SW01 仅 `x,y,z,intensity`，FAST-LIO `lidar_type=2` 走 yaw 角时序回退）。
- **cmd_vel 安全（P0）**：含 NaN/Inf 的 Twist 在适配器内于任何除法/归一化前被拒绝，
  且不刷新指令时间戳（看门狗随即下发零速）。适配器数字启动参数非法（非有限/非正/超范围）必须 fail-fast。

## 2. 指令消费（核心栈 → 适配器）

| 话题 | 类型 | 说明 |
|------|------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 输出（reliable），适配器订阅并映射为 SDK 指令 |

- **速度映射**：`cmd_vel`（m/s, rad/s）× 机型最大速度（`robot_configs/<robot>.yaml` `velocity` 段）→ SDK 比例/标称值。
  - M20：归一化轴指令 `[-1, 1]`（vx/vy/wz 各轴比例），20Hz。
  - SW01：`AstrallMove(vx, vy, vyaw)` 标称 m/s / rad/s，≥20Hz。
- **停止语义**：`cmd_vel` 为零或超时（watchdog）时必须下发零速指令（SW01：`AstrallMove(0,0,0)`；M20：零轴指令）。

## 3. 状态机（所有适配器共用模板）

```
INIT ──> AUTH ──> MODE ──> RUN ──> STOP
                 │         │        │
                 └─────────┴──> WATCHDOG（超时/失权/异常处理）
```

| 状态 | 动作 |
|------|------|
| `INIT` | SDK 初始化、参数加载（robot_configs yaml） |
| `AUTH` | 申请控制权（SW01: `AstrallAuthControl`；M20: 桥接抢占处理）。失败重试，遥控器可抢占 |
| `MODE` | 切换运动模式（SW01: `AstrallSportModeControl(MOVE)`） |
| `RUN` | 订阅 `/cmd_vel` → 周期下发指令（≥20Hz）；同时维持心跳（SW01: 10Hz） |
| `WATCHDOG` | 指令超时 / 失权 / SDK 错误：发零速，尝试重新 AUTH+MODE |
| `STOP` | 收到 shutdown / 失控保护：下发零速并释放控制权 |

## 4. QoS 约定

- 点云：`best_effort`（带宽优先，丢帧可容忍）
- IMU：`reliable`（IMU 数据量小，需可靠投递）
- `/cmd_vel`：`reliable`

## 5. 接入新机型步骤

1. `robot_configs/config/<robot>.yaml`：外参初值 / 最大速度 / 雷达类型 / TF 帧名 / 网络参数
2. `robot_adapters/adapter_<robot>/`：实现上表 1–3 约定（话题输出 + cmd_vel 消费 + 状态机）
3. bringup 按 `robot:=` 参数加载 config 并包含 adapter launch（见 `universal_slam_bringup`）
