# 接口参考

本页列出配置字段、命令行、返回值和编程接口。使用步骤见 [README](../README.md)。

## 1. 设备配置

`manipulation configure` 生成的 `device.yaml`，也可手写。单个 YAML 文档，不超过 64 KiB，未知或重复字段一律拒绝。

```yaml
schema_version: 3
device:
    driver: agx_nero_revo2        # 已注册驱动，见 manipulation-device drivers
    model: nero                   # 可选，descriptions/robots/<model>/，fk 和标定需要
    arm:
        device_id: arm_main
        resource_id: agx_can0     # 物理资源标识，同一标识在本机只允许一个写入者
        tolerance_rad: 0.002      # 到位误差，(0, 0.1]，默认 0.01
        timeout_s: 10             # 单个点位期限，(0, 300]，默认 5
        maximum_age_s: 0.2        # 反馈最大时效，(0, 5]，默认 0.2
        joints:                   # 1–16 个，顺序即目标数组顺序
            - {name: joint1, minimum_rad: -2.705261, maximum_rad: 2.705261}
    effector:                     # 可选，省略则只有机械臂
        device_id: hand_main
        channels: [thumb_tip, ...]  # 1–32 个，顺序即位置数组顺序
        feedback: none            # position（默认）或 none，见下文
        tolerance: 0.02           # 归一化位置误差，(0, 0.1]
        timeout_s: 5              # (0, 120]
        maximum_age_s: 0.2        # (0, 1]
    options:                      # 驱动专有，字符串键值，最多 32 项
        channel: can0
steps:                            # 0–128 步，run 按顺序执行
    - {capability: joint_point, target_rad: [...], speed_scale: 0.05}
    - {capability: effector_position, positions: [...], duration_s: 0.5}
```

| 步骤 | 字段 | 约束 |
| --- | --- | --- |
| `joint_point` | `target_rad` | 数量与 `joints` 相同，每项在该关节限位内 |
| | `speed_scale` | 控制器速度倍率 `(0, 1]`，不是 rad/s；不低于驱动下限 |
| `effector_position` | `positions` | 数量与 `channels` 相同，每项在 `[0, 1]` |
| | `duration_s` | `(0, 驱动上限]` |

`options` 中的路径由驱动声明，相对配置文件所在目录解析。

### 末端反馈

| `feedback` | 连接要求 | 移动结果 | 取消 / 中断 |
| --- | --- | --- | --- |
| `position` | 新鲜、完整的位置和状态反馈 | 连续 3 个样本在误差内且静止时 `succeeded` | 驱动确认停止为 `cancelled`，否则 `stop_unknown` |
| `none` | 只要求新鲜的在线样本，不读位置 | 下发后等待 `duration_s`，结果 `unverified` | 始终 `stop_unknown` |

`none` 只能降低验证要求，不会让驱动声称具备反馈能力。`duration_s` 不得超过 `timeout_s`。
`agx_nero_revo2` 在 `none` 下以手指电流帧作为在线证据，手侧显示为 `unknown`。
实测经 Nero 末端口接入的 Revo2 只在手指运动期间上报位置（0–255）和状态，静止时不上报，故使用 `none`。

### 驱动选项

| 驱动 | 选项 | 说明 |
| --- | --- | --- |
| `mock_device` | 无 | 任意关节数和通道数 |
| `agx_nero_revo2` | `channel` | CAN 接口名，`resource_id` 必须为 `agx_<channel>` |
| | `python` | worker 解释器，命令名或路径 |
| | `sdk_root` | pyAgxArm 目录，须含匹配的 `UPSTREAM_COMMIT` |
| | `worker` | `agx_worker.py` 路径 |

`agx_nero_revo2` 要求关节为 `joint1`–`joint7`；有末端时通道必须为 Revo2 的六个通道且顺序一致。
速度倍率下限 0.01（控制器按整数百分比执行），末端时长上限 2.55 s。

### 配置模板

`descriptions/robots/<model>/devices/<variant>.yaml` 是 `manipulation configure` 使用的模板，占位符：

| 占位符 | 替换为 |
| --- | --- |
| `${datadir}` `${libexecdir}` `${bindir}` | 安装目录，相对生成的 `device.yaml` |
| `${python}` | `--python` 的值，默认 `python3`；显式路径原样保留 |
| `${name}` / `${name:-default}` | `--set name=value`；未提供时用默认值，无默认值则报错 |

## 2. 命令行

### manipulation-device

| 子命令 | 作用 |
| --- | --- |
| `drivers` | 列出已注册驱动 |
| `validate PLAN` | 离线校验配置，不导入厂商 SDK |
| `describe PLAN` | 输出驱动、型号、关节名、通道名（JSON），不连接设备 |
| `inspect PLAN` | 获取独占、连接、诊断、输出状态 |
| `run PLAN [--enable]` | 执行 `steps`；`--enable` 先显式使能；`unverified` 步骤不中断序列 |
| `arm-check PLAN [--timeout-only]` | 关节 1 往返 0.01 rad、取消、重连；`--timeout-only` 改为检查 1 ms 期限的超时锁定 |
| `serve PLAN [--read-only \| --enable]` | 本机 JSONL 控制进程，供 Python 接口使用；`--read-only` 拒绝一切运动（只读工具使用） |

退出码：`0` 成功，`1` 任务未成功或被中断，`2` 参数、配置或连接错误，`3` 全部步骤完成但含 `unverified` 步骤。

`inspect` 输出：

```json
{"firmware":"1.20","enabled":1,"position_mode":1,"moving":0,"fault":0,"joints_rad":[...],
 "effector_feedback":"position","effector_positions":[...],"effector_fault":0}
```

`effector_feedback` 为 `none` 时 `effector_positions` 为空数组。

`run` 最后一行：`{"status":"succeeded","physical_grip_verified":false}`。

### manipulation / hand-eye

全局选项 `--site DIR` 写在子命令前，默认 `$MANIPULATION_SITE` 或 `~/.local/share/spacemit-manipulation/site`。
设备类子命令默认使用 `SITE/device.yaml`，可用 `--plan FILE` 替换。

| 子命令 | 参数 |
| --- | --- |
| `manipulation check` | `--profile build\|control\|vision` |
| `manipulation configure MODEL` | `--variant NAME`（默认 `arm`）、`--set NAME=VALUE`、`--python CMD` |
| `manipulation hand-move V...` | 每个通道一个 `[0,1]` 绝对位置；`--duration S`（默认 2） |
| `manipulation hand-step CHANNEL DELTA` | `CHANNEL` 为序号或通道名，`DELTA` 为 `0.01` 或 `-0.01`；仅限有位置反馈 |
| `manipulation teach FILE` / `fk FILE` | 文件相对现场目录；`teach` 不覆盖已有文件 |
| `hand-eye collect NAME` | `--sampler MODULE`、`--option NAME=VALUE`、`--count N`（12–100） |
| `hand-eye solve NAME` | `--max-error-mm`（默认 5，≤15）、`--max-error-deg`（默认 2，≤5） |
| `hand-eye promote/export/restore NAME` | 依次生成 `candidate.json`、`active.json`、`extrinsic.json`、`restored.json` |

## 3. 结果状态

| 状态 | 含义 | 之后 |
| --- | --- | --- |
| `succeeded` | 连续 3 个新鲜、静止、在误差内的反馈样本 | 可继续 |
| `cancelled` | 取消后驱动确认已停止 | 可继续 |
| `timed_out` | 超过 `timeout_s`，驱动确认已停止 | 锁定 |
| `fault` | 反馈异常、控制器故障或写入失败，已确认停止 | 锁定 |
| `stop_unknown` | 无法确认设备已停止 | 锁定 |
| `unverified` | 无反馈末端：命令已下发并等完执行时间，位置未测量 | 可继续 |

锁定后拒绝新的运动和使能，直到显式断开并重新连接；重连只读取实际状态，不重放目标、不归位、不使能。

写入设备之前被拒绝的请求（如前置条件不满足）不改变状态、不锁定，C++ 中抛出 `CommandRejected`。

## 4. 设备独占

连接时在 `/run/lock/spacemit-manipulation/<resource_id>.lock` 上加排他锁，进程退出时自动释放。
目录在构建时由 `MANIPULATION_LOCK_DIRECTORY` 指定，运行时可用同名环境变量覆盖（测试用）；
同一主机上所有程序必须使用同一目录才能互斥。

Nero 驱动另外持有 `/tmp/nero_<channel>_control.lock`，与使用该锁的 pyAgxArm 程序互斥。
两种锁都不约束厂商 Web 和其他主机。

## 5. C++ API

头文件位于 `include/manipulation/`，命名空间 `spacemit::manipulation`。

| 头文件 | 内容 |
| --- | --- |
| `types.h` | `DeviceConfig`、`Result`、`CommandResult`、`CommandRejected` |
| `arm.h` | 驱动接口 `Arm`、`ArmState`、`JointPoint` |
| `device.h` | 驱动接口 `EndEffector`、注册表 `DeviceRegistry`、会话 `DeviceSession` |
| `device_plan.h` | `LoadDevicePlan`、`RunDeviceSteps` |
| `calibration.h` | `SolveEyeToHand`（需 `-DMANIPULATION_ENABLE_CALIBRATION=ON`） |

`DeviceSession` 方法由同一所有者串行调用：

| 方法 | 行为 |
| --- | --- |
| `Connect()` | 获取独占、连接、读取一次状态；不使能、不运动 |
| `ReadArm()` / `ReadEffector()` | 返回新鲜完整的状态，过期或不完整时抛出异常 |
| `Diagnose()` | 主动查询固件等参数，不运动 |
| `EnableArm()` | 显式使能，失败则锁定 |
| `MoveArm(point, cancelled)` | 阻塞执行，返回 `CommandResult`；`cancelled` 每 10 ms 调用一次 |
| `MoveArmPath(path, cancelled)` | 连续经过各路径点，只在最后一点停下，到位判定同上；驱动需支持 `joint_path` |
| `MoveEffector(point, cancelled)` | 同上 |
| `Disconnect()` | 运行中先尝试保持，再断开并释放独占 |
| `Latched()` | 是否处于锁定 |

## 6. Python API

```python
DeviceClient(executable, plan, *, allow_motion=True, allow_enable=False)
```

启动 `manipulation-device serve`，进程存活期间持有设备独占；`allow_motion=False` 以只读方式打开，拒绝一切运动。方法：`state()`、`diagnose()`、
`move_arm(target_rad, speed_scale, tolerance_rad=0, wait_for_arrival=True, resend_interval_s=0)`、`move_arm_path(path_rad, speed_scale, max_velocity_rad_s, max_acceleration_rad_s2, tolerance_rad=0)`、
`move_effector(positions, duration_s=0.5, allow_unverified=False)`、`close()`。

`move_arm_path` 从当前位置出发，以平滑曲线连续经过各路径点（最多 128 个），只在最后一点停下：
`max_velocity_rad_s`（≤0.35）和 `max_acceleration_rad_s2`（≤1.0）限制变化最大的关节，`speed_scale` 为控制器倍率；
每个路径点都须在关节限位内，截止时间为曲线时长加 `timeout_s`。`tolerance_rad` 为终点到位容差（≤0.8°，0 表示用设备的
`tolerance_rad`）：Nero 连续发送时控制器会留下零点几度的稳态偏差，需要按场景给出容差。

到位判定：连续三个新鲜反馈各关节都在容差内且静止。Nero 的“运动中”由关节反馈测得（0.2 s 内变化超过 0.05°），
不使用控制器的“未到达目标”标志；停在目标外超过 1 s 时原样重发同一目标，最多 5 次。
`move_arm` 的 `resend_interval_s`（0 或 0.02–0.2 s）改为在超出容差期间按该间隔原样重发（原应用放杯的控制器端点方式）；
`wait_for_arrival=False` 时连续两个新鲜反馈在容差内即返回 `succeeded`（`detail` 以 `within_tolerance` 开头），不等静止。

`move_arm_trajectory(times_s, positions_rad, command_rate_hz, max_step_rad, speed_scale, tolerance_rad, wait_for_arrival)`
按给定时间发送关节轨迹（≤4000 个点、≤60 s、速率 20–500 Hz、单步 ≤3°），首点须是当前位置（0.5° 内）。
`wait_for_arrival=False` 时发完最后一个点即返回 `succeeded`（`detail` 以 `delivered` 开头），机械臂仍可能在收尾；
此时下一条指令可以在收尾中开始。末端动作期间，收尾中的机械臂可以继续向上一条指令的目标收拢，但离该目标的距离不得比开始时再远 1°；静止的机械臂偏离起点不得超过 1°。状态中的 `controller_limits`
为控制器实时报告的各关节位置、速度和加速度限制（连接时读取）。
`allow_unverified=True` 时接受无反馈末端的 `unverified` 结果，否则同样抛出 `DeviceCommandError`。
每个方法接受 `timeout`（秒，默认 10，最长 180，包含排队时间）和 `is_cancelled`（回调）。

运动未成功时抛出 `DeviceCommandError`：设备进程返回的结果为字典，含 `status`；
设备进程拒绝请求时为错误字符串。
