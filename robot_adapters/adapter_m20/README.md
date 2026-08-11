# adapter_m20 — M20（云深处 山猫）适配器

## 背景

M20 机载运行 **ROS2 Foxy**，上电发布 `/IMU`、`/LIDAR/POINTS`（RoboSense 96 线点云），**无标准 cmd_vel** —— 控制为 **UDP JSON 轴速度指令**（归一化 `[-1,1]` 比例，20Hz）。不能与厂商自带遥控/导航模式并存。

```
M20 机载(Foxy)             算法主机(Humble)
/IMU ──┐
        ├─(桥接)──> /imu_raw ──> FAST-LIO2
/LIDAR/POINTS ──┘           /points_raw ─┘
                                     ^
Nav2 /cmd_vel ──> m20_cmd_vel_bridge ──> UDP JSON 轴指令(10.21.31.103:30000) ──> M20 底盘
```

## 协议要点（PLAN.md Resolved Decision #13）

**帧格式（16 字节头 + JSON 体）**

| 字节 | 含义 |
|------|------|
| `[0-3]` | 同步字 `0xEB 0x91 0xEB 0x90` |
| `[4-5]` | ASDU 长度（小端，= JSON 体字节数） |
| `[6-7]` | 自增 msgID（小端，0→65535 循环，应答回显） |
| `[8]` | `0x01`（JSON） |
| `[9-15]` | 7×0x00 |

```json
{"PatrolDevice":{"Type":2,"Command":21,"Time":"YYYY-MM-DD HH:MM:SS","Items":{"X":0.0,"Y":0.0,"Z":0.0,"Roll":0.0,"Pitch":0.0,"Yaw":0.0}}}
```

- 数值 `clamp [-1,1]` 且四舍五入保留 2 位小数（与官方 SDK 一致）；`Time` 每次填充当前本地时间。
- 收帧 `[6-7]` 语义为 Type、`[8-9]` 为 Command（发帧 `[6-7]` 是 msgID，按官方 SDK 惯例；布局以实机抓包校准）。

**控制序列（m20_cmd_vel_bridge 自动执行）**

```
心跳 (100,100) 1Hz
  → (1101,5,Mode=0) 确保 Regular 模式
  → (2,22,MotionParam=1) 起立
  → 等待上报 MotionState==17（RL 控制，15s 超时告警不阻塞）
  → (2,23,GaitParam=0x1001) 切基础平地步态
  → RUN: (2,21,{X,Y,Yaw}) 轴指令 20Hz
```

- **错误门控**：收到 `(1002,3)` 错误上报 或 `ControlUsageMode != 0`（非 Regular）→ 停止发轴指令 + `RCLCPP_ERROR`（带错误码）。
- **状态上报**：`(1002,6)` 2Hz（MotionState / ControlUsageMode）、`(1002,4)` 10Hz 速度反馈。
- **无逐帧 ACK**：靠主动上报驱动状态机。
- **控制准入**：公开资料无"控制权申请帧"，以 **Regular 模式 + 心跳归属**为控制准入（实机验证：钉钉文档需登录待提供）。
- 诊断话题：`/m20_bridge_status`（std_msgs/String，当前状态/错误码）。

## 话题桥（m20_topic_bridge）

- 直连 DDS（同 `ROS_DOMAIN_ID` + 同 RMW）：话题级互通，可不用本节点。
- 跨网段 / WiFi：**zenoh-bridge-ros2dds 0.11.0-stable**（两端同版本）。
- ⚠️ **带宽硬约束**：96 线点云 ≈ 240–720 Mbps，**必须有线千兆**；WiFi 需降采样/隔帧（本节点 `voxel_downsample` 留 TODO，P4 启用）。
- 参数化源/目标话题，默认 `/IMU→/imu_raw`、`/LIDAR/POINTS→/points_raw`；QoS：点云 best_effort、IMU reliable。

## 参数（m20_cmd_vel_bridge）

| 参数 | 默认 | 说明 |
|------|------|------|
| `robot_ip` | `10.21.31.103` | 机器人 IP（SDK 默认 10.21.33.103 为笔误，以 10.21.31.103 为准） |
| `port` | `30000` | 控制端口 |
| `cmd_frequency_hz` | `20` | 轴指令频率 |
| `heartbeat_hz` | `1` | 心跳频率 |
| `vx_max / vy_max / wz_max` | `0.5 / 0.5 / 0.6` | 速度映射分母（`cmd_vel / max → [-1,1]`），按机型覆盖 |
| `cmd_vel_topic` | `/cmd_vel` | 输入 |
| `status_topic` | `/m20_bridge_status` | 诊断输出 |

## 实机待验证项（P4）

1. **控制权**：钉钉文档中的"控制权申请帧"（需登录，待用户提供）——当前以 Regular 模式 + 心跳归属为准。
2. **步态码**：`0x1001`（基础平地步态）以 2026 固件为准；备选 `0x1003`。
3. **最大速度映射**：`vx_max/vy_max/wz_max` 需实测校准（轴指令归一化比例）。
4. **收帧头布局**（Type/Command 字节偏移）与 ASDU 长度定义（是否含头）以实机抓包确认。

## 接口约定

见 `robot_adapters/ADAPTER_INTERFACE.md`（话题、QoS、状态机模板）。
