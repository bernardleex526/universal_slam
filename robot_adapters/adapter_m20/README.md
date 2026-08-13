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
- **收帧校验（review 修复）**：
  - 来源校验：只接受配置的 `robot_ip:port` 来源的上报帧（防伪造/串流）；实机若机器人以不同源端口回包，按实测修正（见 m20.yaml 注释）。
  - 严格长度：`n == 16 + asdu_len`（截断/尾随一律拒绝），asdu_len>0 且有上限（4096）。
  - 语义校验：状态帧必须含 `MotionState`+`ControlUsageMode` 且数值在界内（int16/uint16），错误帧必须能提取错误码 —— 缺字段/越界数值的帧不采信。
- **ErrorHalt 恢复（P1，有界/不猜协议）**：
  - ErrorHalt 期间**每 tick 持续下发零速**（防单次零速帧被 UDP 丢失后机器人执行上一非零指令）。
  - 协议无 ACK/错误清除帧 → **自动恢复默认关闭**，ErrorHalt 保持到人工复位或重启。
  - 人工复位（显式门禁，常开）：
    `ros2 service call /m20_cmd_vel_bridge/reset_error std_srvs/srv/Trigger {}`
    → 清除错误锁存并重跑 AUTH+MODE 启动序列；若机器人仍报非 Regular/再报错，门控会再次生效。
  - 可选有界自动恢复（`error_auto_recovery:=true`，需实机确认协议前提后才建议开启）：
    恢复评估以 `state==ErrorHalt` 驱动；仅当"收到过状态上报 + Regular 模式 +
    `error_clear_window_s` 内无新错误帧 + 退避（`base × 2^attempts`）已到"才重跑启动序列；
    重试达 `error_max_recovery_attempts` 后保持 halt。
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
| `robot_ip` | `10.21.31.103` | 机器人 IP（SDK 默认 10.21.33.103 为笔误，以 10.21.31.103 为准；空值 fail-fast） |
| `port` | `30000` | 控制端口（校验 [1,65535]，非法 fail-fast） |
| `cmd_frequency_hz` | `20` | 轴指令频率（有限正数校验，上限 200Hz） |
| `heartbeat_hz` | `1` | 心跳频率（有限正数校验，上限 100Hz） |
| `vx_max / vy_max / wz_max` | `0.5 / 0.5 / 0.6` | 速度映射分母（有限正数校验），按机型覆盖 |
| `cmd_vel_topic` | `/cmd_vel` | 输入 |
| `status_topic` | `/m20_bridge_status` | 诊断输出 |
| `error_auto_recovery` | `false` | ErrorHalt 有界自动恢复开关（默认关：协议无错误清除帧，硬件不确定项） |
| `error_max_recovery_attempts` | `3` | 自动恢复重试上限（[0,100]） |
| `error_recovery_backoff_s` | `5.0` | 指数退避基数（等待 `base × 2^attempts` 秒） |

**cmd_vel 安全（P0）**：含 NaN/Inf 的 Twist 在任何除法/归一化前被拒绝（`include/adapter_m20/cmd_math.hpp` 的 `normalizeCmdVel`），且不更新指令时间戳——500ms 看门狗随即下发零轴指令。所有数字启动参数构造时校验，非法直接抛异常退出（fail-fast）。

## 测试（P0/P1，无需机器人）

```bash
colcon build --packages-select adapter_m20
colcon test --packages-select adapter_m20
colcon test-result --verbose
```

`test_cmd_math.cpp`：归一化正常/钳制、6 个输入各自的 NaN/Inf 除法前拒绝、启动参数 fail-fast（有限正数/频率/端口）。
`test_error_recovery.cpp`：ErrorHalt 恢复决策——默认保持 halt、证据不足（无状态上报/错误置位/非 Regular）不重试、退避计时、有界重试上限。

## 实机待验证项（P4）

1. **控制权**：钉钉文档中的"控制权申请帧"（需登录，待用户提供）——当前以 Regular 模式 + 心跳归属为准。
2. **步态码**：`0x1001`（基础平地步态）以 2026 固件为准；备选 `0x1003`。
3. **最大速度映射**：`vx_max/vy_max/wz_max` 需实测校准（轴指令归一化比例）。
4. **收帧头布局**（Type/Command 字节偏移）与 ASDU 长度定义（是否含头）以实机抓包确认。

## 接口约定

见 `robot_adapters/ADAPTER_INTERFACE.md`（话题、QoS、状态机模板）。
