# 接口参考

配置字段、命令行、结果状态和编程接口。上手步骤见 [README](../README.md)。

## 1. 设备配置

`manipulation configure MODEL` 按型号模板生成 `SITE/MODEL/device.yaml`，也可手写。文件为单个 YAML 文档，不超过 64 KiB，未知或重复字段一律拒绝。

```yaml
schema_version: 3
device:
    driver: agx_nero_revo2        # 已注册驱动，见 manipulation-device drivers
    model: nero                   # 可选，descriptions/robots/<model>/；运动学和标定需要
    arm:
        device_id: arm_main
        resource_id: agx_can0     # 物理资源标识；同一标识在本机只允许一个写入者
        tolerance_rad: 0.002      # 到位容差，(0, 0.1]，默认 0.01
        timeout_s: 10             # 单次动作期限，(0, 300]，默认 5
        maximum_age_s: 0.2        # 反馈最大时效，(0, 5]，默认 0.2
        joints:                   # 1–16 个，顺序即目标数组顺序
            - {name: joint1, minimum_rad: -2.705261, maximum_rad: 2.705261}
    effector:                     # 可选，省略则只有机械臂
        device_id: hand_main
        channels: [thumb_tip, ...]  # 1–32 个，顺序即位置数组顺序
        feedback: none            # position（默认）或 none
        tolerance: 0.02           # 归一化位置容差，(0, 0.1]
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
| `joint_point` | `target_rad` | 数量与 `joints` 相同，每项在关节限位内 |
| | `speed_scale` | 控制器速度倍率 `(0, 1]`，不是 rad/s；不低于驱动下限 |
| `effector_position` | `positions` | 数量与 `channels` 相同，每项在 `[0, 1]`：`0` 闭合，`1` 张开 |
| | `duration_s` | `(0, 驱动上限]`，且不超过末端 `timeout_s` |

`options` 中的路径由驱动声明，相对配置文件所在目录解析。

### 末端反馈

| `feedback` | 连接要求 | 移动结果 | 取消或中断 |
| --- | --- | --- | --- |
| `position` | 新鲜、完整的位置和状态反馈 | 连续 3 个样本在容差内且静止时 `succeeded` | 驱动确认停止为 `cancelled`，否则 `stop_unknown` |
| `none` | 只要求新鲜的在线样本 | 下发后等待 `duration_s`，结果 `unverified` | 始终 `stop_unknown` |

- 所有末端的位置含义相同：`0` 闭合，`1` 张开，由驱动转换为设备自身的刻度。
- `none` 只降低验证要求，不会让驱动声称具备反馈。
- 经 Nero 末端口接入的 Revo2 静止时不上报位置，须用 `none`；此时以手指电流帧作为在线证据，手侧显示为 `unknown`。

### 驱动选项

| 驱动 | 选项 | 说明 |
| --- | --- | --- |
| `mock_device` | 无 | 任意关节数和通道数 |
| `agx_nero_revo2` | `channel` | CAN 接口名；`resource_id` 必须为 `agx_<channel>` |
| | `python` | worker 解释器，命令名或路径 |
| | `sdk_root` | pyAgxArm 目录，须含匹配的 `UPSTREAM_COMMIT` |
| | `worker` | `agx_worker.py` 路径 |
| `so101` | `port` | 必填，串口路径，相对配置目录解析 |
| | `arm_ids` / `gripper_id` | 关节 ID（默认 `1,2,3,4,5`）/ 夹爪 ID（默认 `6`），不能重复 |
| | `arm_calibration` | 机械臂校准文件，模板默认 `so101_calibration.json`；配置中省略时只能读取状态 |
| | `gripper_calibration` | 夹爪校准文件，模板默认 `so101_gripper_calibration.json`；配置夹爪时必填 |
| | `max_velocity_rad_s` | 速度倍率为 1 时的关节速度，`0.01–3.6`，默认 `1.0` |
| | `grip_load_threshold` | 判定夹住的最小负载，`1–1000`（额定输出的 0.1%），默认 `100` |

**Nero / Revo2**：
- 关节必须为 `joint1`–`joint7`；有末端时通道必须为 Revo2 的六个通道，顺序一致。
- 速度倍率下限 0.01，控制器按整数百分比执行；末端单次动作不超过 2.55 s。

**SO101**：
- 关节顺序为 `shoulder_pan`、`shoulder_lift`、`elbow_flex`、`wrist_flex`、`wrist_roll`，夹爪通道为 `[gripper]` 且 `feedback: position`。
- 关节角为 `ticks × 2π / 4095 − π`；夹爪位置为 `(ticks − closed_ticks) / (open_ticks − closed_ticks)`，支持开闭方向相反的安装。
- 经 SDK `motor` 组件的 Feetech 驱动访问舵机，串口固定为 1 Mbps。连接时只读核对舵机型号、模式、中位偏移和反馈，不写 EEPROM、不使能、不归位。
- 目标和扭矩逐个舵机写入并读回确认，不是同步写，各关节的开始时刻有几毫秒差异。
- **控制范围**：取配置限位与校准行程的交集。
  - 配置模板中前四个关节的限位为 ±π，实际范围由每台机械臂的校准行程决定；
  - `wrist_roll` 可整圈转动，校准行程为 0–4095，由配置限位（−157°～+163°，取自模型）约束；
  - 超出范围时拒绝使能或运动，报错列出每个超限关节的当前位置和允许范围。
- 行程文件格式：
  - 机械臂按关节名保存 `id`、`homing_offset`、`range_min`、`range_max`；
  - 夹爪保存 `motor_id`、`open_ticks`、`closed_ticks` 和 `homing_offset`。
  - 夹爪文件可省略 `homing_offset`，此时驱动不核对夹爪偏移，文件只能用于偏移未改变的安装。
- **抓握判断**（能力 `grasp_detection`）：
  - 闭合目标未到达、夹爪静止，且负载绝对值不低于 `grip_load_threshold` 时，`effector_holding` 为 `true`；
  - 连续三个这样的样本使动作以 `effector_grasp_holding` 成功结束；
  - 夹住时舵机的过载保护标志（状态位 5）不算故障；
  - `hand-hold` 保持原闭合目标。
- 使能夹爪前核对最大扭矩不高于 500、保护电流不高于 250 mA。
- 不支持连续路径、定时轨迹或力控。

### 配置模板

`descriptions/robots/<model>/devices/<variant>.yaml` 是 `manipulation configure` 使用的模板，占位符：

| 占位符 | 替换为 |
| --- | --- |
| `${datadir}` `${libexecdir}` `${bindir}` | 安装目录，相对生成的 `device.yaml` |
| `${python}` | `--python` 的值，默认 `python3` |
| `${name}` / `${name:-default}` | `--set name=value`；未提供时用默认值，无默认值则报错 |

## 2. 命令行

### manipulation-device

| 子命令 | 作用 |
| --- | --- |
| `drivers` | 列出已注册驱动 |
| `validate PLAN` | 离线校验配置，不导入厂商 SDK |
| `describe PLAN` | 输出驱动、型号、关节名与限位、机械臂 `timeout_s`、通道名（JSON），不连接设备 |
| `capabilities PLAN` | 离线查询机械臂与末端的驱动能力 |
| `inspect PLAN` | 获取独占、连接、诊断并输出状态 |
| `run PLAN [--enable]` | 执行 `steps`；`--enable` 先显式使能；`unverified` 步骤不中断序列 |
| `arm-check PLAN [--timeout-only]` | 关节 1 往返 0.01 rad，再测试取消和重连；`--timeout-only` 改为检查 1 ms 期限的超时锁定 |
| `serve PLAN [--read-only \| --enable]` | 本机 JSONL 控制进程，供 Python 接口使用；`--read-only` 拒绝一切运动 |

`serve` 的 `--allow-disable`、`--allow-effector-enable`、`--allow-effector-disable` 分别授权机械臂失能、末端使能、末端失能，默认关闭。

退出码：`0` 成功；`1` 任务未成功或被中断；`2` 参数、配置或连接错误；`3` 全部步骤完成，但含 `unverified` 步骤。

`run` 最后一行是汇总结果，形如 `{"status":"succeeded","physical_grip_verified":false}`。

**状态字段**（`inspect` 输出，Python `state()` 字段相同）：

| 字段 | 含义 |
| --- | --- |
| `enabled`、`position_mode`、`moving`、`fault` | 机械臂使能、位置模式、实际运动、故障 |
| `joints_rad` | 关节位置，按配置顺序 |
| `effector_feedback` / `effector_positions` | 末端反馈类型 / 各通道位置（`0` 闭合，`1` 张开）；`none` 时位置为空数组 |
| `effector_enabled` | `true`、`false`，驱动无法读回时为 `null` |
| `effector_loads` | 各通道负载 `[-1,1]`（额定输出的比例，符号为驱动方向），不提供时为空数组 |
| `effector_holding` | 抓握判断；仅声明 `grasp_detection` 的末端给出 `true` / `false`，其他为 `null` |
| `controller_limits` | 控制器报告的各关节位置、速度、加速度限制（驱动提供时） |

### manipulation / hand-eye

- 全局选项 `--site DIR` 写在子命令前，默认为 `$MANIPULATION_SITE` 或 `~/.local/share/spacemit-manipulation/site`。
- 每种设备一个目录 `SITE/NAME/`，存放 `device.yaml`、姿态和校准文件。`SITE/device.yaml` 是指向当前设备配置的符号链接。
- 设备类子命令默认使用当前设备，可用 `--plan FILE` 替换。
- 所有命令都不自动使能、清错或归位。

| 子命令 | 参数与说明 |
| --- | --- |
| `check` | `--profile build\|control\|vision`；`vision` 另检查 OpenCV、SciPy、`pyrealsense2` |
| `configure MODEL` | 生成 `SITE/MODEL/device.yaml` 并设为当前设备；`--variant NAME`（默认 `arm`）、`--set NAME=VALUE`、`--python CMD`；不覆盖已有文件 |
| `use [NAME]` | 把 `SITE/NAME/device.yaml` 设为当前设备；省略 `NAME` 时只列出已配置的设备，`*` 标记当前设备 |
| `validate` / `inspect` / `capabilities` | 同 `manipulation-device` 对应子命令 |
| `monitor` | 只读，持续打印关节角（度）、状态、末端位置、负载与抓握判断；`--rate HZ`（1–20，默认 5）、`--count N` |
| `enable` / `disable`、`hand-enable` / `hand-disable` | 使能 / 释放扭矩；失能前须支撑机械臂并卸载物品 |
| `hold` / `hand-hold` | 保持当前位置，不释放扭矩 |
| `joint-step JOINT DELTA` / `joint-move Q...` | 单关节相对 / 全部关节绝对点位，单位 rad；`JOINT` 为关节名或从 0 开始的序号；`--speed`（默认 0.05） |
| `arm-check` | 同 `manipulation-device arm-check`，会运动 |
| `hand-move V...` | 每个通道一个绝对位置（`0` 闭合，`1` 张开）；`--duration S`（默认 2） |
| `hand-step CHANNEL DELTA` | 单通道 `±0.01`，仅限有位置反馈的末端 |
| `hand-grasp` | 闭合到夹住或完全闭合，仅限声明 `grasp_detection` 的末端；`--positions V...`（默认全 0）、`--duration S`（默认 1） |
| `teach FILE` / `fk FILE` | 记录静止姿态 / 姿态文件的正运动学；文件相对设备配置所在目录，`teach` 不覆盖 |
| `pose` | 只读当前末端位姿：`position_m`、`rpy_rad`（URDF 横滚、俯仰、偏航） |
| `ik X Y Z` | 逆解，不运动；`--rpy R P Y`、`--orientation-weight W`、`--seed-pose FILE`（离线求解）；不收敛时退出码 2 |
| `pose-move X Y Z` | 逆解后关节点位运动；参数同 `ik`，另有 `--speed`；按控制器速度上限估算的时长超过 `timeout_s` 时不运动 |
| `line-move X Y Z` | 末端直线运动；另有 `--step-mm`（默认 5）；支持连续路径的驱动另用 `--path-velocity`、`--path-acceleration` |
| `hand-eye collect NAME` | `--sampler MODULE`、`--option NAME=VALUE`（见下）、`--count N`（12–100） |
| `hand-eye solve NAME` | 生成 `candidate.json`；`--max-error-mm`（默认 5，≤15）、`--max-error-deg`（默认 2，≤5） |
| `hand-eye promote` / `export` / `restore NAME` | 分别生成 `active.json`、`extrinsic.json`、`restored.json` |

`hand-eye collect` 的 `--option` 可重复：

| 选项 | 含义 |
| --- | --- |
| `board=FILE` | 必填，ChArUco 标定板 JSON：`type`、`dictionary`、`squares_x/y`、`square_length_m`、`marker_length_m`；可选 `min_corners`、`max_reprojection_px` 和 `image_profile`（`color_resolution`、`fps`、`crop_xywh`） |
| `serial=SN` | RealSense 序列号，连接多台相机时必填 |
| `measured_width_mm` / `measured_height_mm` | 成对填写打印后的实测宽、高 |
| `window=FILE\|x0,y0,x1,y1` | 标定板须完整位于的图像区域 |

求解残差门槛同时适用于全部样本与留出样本，实际门槛写入结果。

### so101-calibrate

`so101-calibrate OPERATION --port PORT`：所有操作要求舵机已释放扭矩，且从不使能扭矩。

| 操作 | 作用 | 寄存器 |
| --- | --- | --- |
| `all` | 依次执行 `configure`、`center`、`capture`、`limits`；校准文件在写入限位前保存 | 同各步骤 |
| `status` | 读取舵机状态 | 只读 0–70 |
| `configure` | 写入推荐参数 | 写 7=0、21/22/23=16/0/0、33=0；夹爪另写 16=500、28=250、36=25 |
| `center` | 中位校准 | 写 31（偏移，符号位为位 11）及 9/11=0/4095 |
| `capture` | 采集行程，新建校准文件，不覆盖已有文件：前四个关节转动时实时显示 `MIN`/`POS`/`MAX`，按回车结束，记录最小值和最大值；`wrist_roll` 记为整圈 0–4095；夹爪分别读取张开和闭合位置 | 只读 |
| `limits` | 把校准文件中的行程写入舵机限位 | 写 9/11 |

- **校准文件**：默认为 `SITE/so101/` 中的 `so101_calibration.json`、`so101_gripper_calibration.json`，与 SO101 配置模板的默认值一致；`--arm-calibration`、`--gripper-calibration` 可改用其他文件名（相对 `SITE/so101/`）。
- **现场目录**：`--site DIR` 指定 `SITE`，默认值与 `manipulation` 相同。
- **公共选项**：`--part all|arm|gripper`（默认 `all`）、`--arm-ids`（默认 `1,2,3,4,5`）、`--gripper-id`（默认 `6`）、`--library`（默认取安装目录的 `libmanipulation_feetech_bus.so`）。
- **写入方式**：只写与当前值不同的寄存器。`motor` 驱动在每次写入前后解锁（55=0）和上锁（55=1），之后逐项读回核对。
- **`center` 的计算**：先清零偏移、读取实际位置，再写入 `偏移 = 位置 − 2047`，最后核对当前位置在 2047±3 内。
- **`capture` 的检查**：每个关节的行程至少 32 个刻度、须包含中位 2047（`center` 设定的模型零位），且读数不能在编码器两端之间跳变；不满足时一次列出所有问题关节，不保存文件。
- **`limits` 的前提**：要求舵机偏移与文件一致，且当前位置在行程内。

## 3. 结果状态

| 状态 | 含义 | 之后 |
| --- | --- | --- |
| `succeeded` | 满足所选完成条件（见 [§6 完成条件](#完成条件)） | 可继续 |
| `cancelled` | 取消后驱动确认已停止 | 可继续 |
| `timed_out` | 超过期限，驱动确认已停止 | 锁定 |
| `fault` | 反馈异常、控制器故障或写入失败，已确认停止 | 锁定 |
| `stop_unknown` | 无法确认设备已停止 | 锁定 |
| `unverified` | 无反馈末端：已下发并等完执行时间，位置未测量 | 可继续 |

- **锁定**：锁定后拒绝新的运动和使能，直到断开并重新连接。重连只读取实际状态，不重放目标、不归位、不使能。
- **写入前拒绝**：写入设备前就被拒绝的请求（如前置条件不满足）不改变状态、不锁定，C++ 中抛出 `CommandRejected`。
- **软件保持**：Ctrl+C 和停止信号请求软件保持。Nero 以不高于当前命令的速度回到停止时刻的位置，可能先越过再小幅回退。这不是急停。
- **抓稳证据**：`physical_grip_verified` 只在末端按抓握判断结束（`detail` 为 `effector_grasp_holding`）时为 `true`。

## 4. 设备独占

- **资源锁**：连接时在 `/run/lock/spacemit-manipulation/<resource_id>.lock` 上加排他锁，进程退出时释放。
  - 锁目录由组件自动创建并设置 sticky 位，所有使用本组件的程序共用同一目录，因而互斥。
- **Nero**：驱动另外持有 `/tmp/nero_<channel>_control.lock`，与使用该锁的 pyAgxArm 程序互斥。
- **SO101**：
  - 按规范化串口路径同时使用串口排他打开和文件锁，但不能撤销其他程序预先打开的串口。
  - 读取失败时 `motor` 驱动内部重试 3 次，收到不完整响应时最长约 3 s。失败后会话锁定，不重放目标。
  - `motor` 驱动在进程退出前不关闭串口，串口断开后须重新启动进程。
- **约束范围**：所有锁都不约束厂商 Web 和其他主机。

## 5. C++ API

头文件位于 `include/manipulation/`，命名空间 `spacemit::manipulation`。

| 头文件 | 内容 |
| --- | --- |
| `types.h` | `DeviceConfig`、`Result`、`CommandResult`、`CommandRejected` |
| `arm.h` | 驱动接口 `Arm`、`ArmState`、`JointPoint`、`JointPath`、`JointTrajectory` |
| `device.h` | 驱动接口 `EndEffector`、注册表 `DeviceRegistry`、会话 `DeviceSession` |
| `device_plan.h` | `LoadDevicePlan`、`RunDeviceSteps` |
| `calibration.h` | `SolveEyeToHand`（需 `MANIPULATION_ENABLE_CALIBRATION`） |

`DeviceSession` 的方法由同一所有者串行调用：

| 方法 | 行为 |
| --- | --- |
| `Connect()` / `Disconnect()` | 获取独占、连接并读取一次状态，不使能、不运动 / 运行中先尝试保持，再断开并释放独占 |
| `ReadArm()` / `ReadEffector()` | 返回新鲜完整的状态，过期或不完整时抛出异常 |
| `Diagnose()` | 主动查询固件等参数，不运动 |
| `GetArmCapabilities()` / `GetEffectorCapabilities()` | 查询驱动支持的操作 |
| `EnableArm()` / `DisableArm()`、`EnableEffector()` / `DisableEffector()` | 显式使能 / 释放扭矩 |
| `HoldArm()` / `HoldEffector()` | 保持停止，不卸力、不清除锁定 |
| `JogArm(joint_index, delta_rad, speed_scale, cancelled)` | 基于当前位置的单关节相对点位 |
| `MoveArm` / `MoveArmPath` / `MoveArmTrajectory` / `MoveEffector` | 阻塞执行并返回 `CommandResult`；`cancelled` 每 10 ms 调用一次 |
| `FollowArm(point)` / `FollowEffector(point)` | 目标跟随：发出目标即返回，见 [§6 完成条件](#完成条件) |
| `Latched()` | 是否处于锁定 |

## 6. Python API

```python
DeviceClient(executable, plan, *, allow_motion=True, allow_enable=False, allow_disable=False,
             allow_effector_enable=False, allow_effector_disable=False)
```

- 启动 `manipulation-device serve`，进程存活期间持有设备独占。
- `allow_motion=False` 以只读方式打开；使能、失能需对应授权。
- 驱动不支持的操作在写入前拒绝，例如 Nero / Revo2 不支持本接口的失能和末端独立使能，应先查询 `capabilities()`。

| 方法 | 作用 |
| --- | --- |
| `state()`、`diagnose()`、`capabilities()` | 状态（字段见 [§2 manipulation-device](#manipulation-device)）、主动诊断、能力声明 |
| `enable_arm()` / `disable_arm()`、`enable_effector()` / `disable_effector()` | 改变使能状态 |
| `hold_arm()` / `hold_effector()` | 保持停止，不清除已有锁定 |
| `jog_arm(joint_index, delta_rad, speed_scale=0.05)` | 单关节相对点位 |
| `move_arm(target_rad, speed_scale, tolerance_rad=0, wait_for_arrival=True, resend_interval_s=0)` | 关节点位 |
| `move_arm_path(path_rad, speed_scale, max_velocity_rad_s, max_acceleration_rad_s2, tolerance_rad=0)` | 连续路径 |
| `move_arm_trajectory(times_s, positions_rad, command_rate_hz=100, max_step_rad=0.0174533, speed_scale=1.0, tolerance_rad=0, wait_for_arrival=True, effector_schedule=(), allow_unverified=False)` | 定时轨迹 |
| `follow_arm(target_rad, speed_scale)` | 关节目标跟随，发出目标即返回 |
| `move_effector(positions, duration_s=0.5, allow_unverified=False)` | 末端各通道移到绝对位置（`0` 闭合，`1` 张开） |
| `follow_effector(positions, duration_s=0.1, allow_unverified=False)` | 末端目标跟随，发出目标即返回 |
| `close()` | 关闭会话；也可使用 `with` |

- 每个方法接受 `timeout`（秒，默认 10，最长 180，含排队时间）和 `is_cancelled`（回调）。
- 运动未成功时抛出 `DeviceCommandError`：结果为含 `status` 的字典；设备进程拒绝请求时为错误字符串。

### 完成条件

| 用法 | 返回时机 | `detail` 开头 |
| --- | --- | --- |
| 默认 | 连续三个新鲜反馈各关节在容差内且静止 | `target_reached` |
| `move_arm(..., wait_for_arrival=False)` | 连续两个新鲜反馈在容差内，不等静止 | `within_tolerance` |
| `move_arm_trajectory(..., wait_for_arrival=False)` | 最后一个点已发送，机械臂可能仍在收尾 | `delivered` |
| 无反馈末端，或带 `effector_schedule` | 已下发并等完执行时间；结果 `unverified`，须 `allow_unverified=True` | — |
| 抓握判断末端停在物体上 | 连续三个样本判定夹住 | `effector_grasp_holding` |
| `follow_arm` / `follow_effector` | 目标已发出，不等到位；无反馈末端结果 `unverified` | `sent` |

- **目标跟随**：用于遥操作、视觉伺服等高频更新目标的场景。
  - 首个目标要求机械臂静止，之后的目标可以替换机械臂还在运动中的目标；每个目标仍检查关节限位、使能和故障。
  - 结束时调用 `hold_arm()` 停在当前位置，或 `move_arm()` 到达最终目标并确认到位。
  - 更新频率受设备往返时间限制；程序异常退出时，设备停在最后一个目标。
- **容差**：`tolerance_rad` 不超过 0.8°，`0` 表示使用配置中的 `tolerance_rad`。
- **重发**：`resend_interval_s`（`0`，或 0.02–0.2 s）在超出容差期间按该间隔原样重发目标。
- **Nero 运动判定**：Nero 的"运动中"由关节反馈测得（0.2 s 内变化超过 0.05°）；停在目标外超过 1 s 时原样重发，最多 5 次。连续发送时控制器会留下零点几度的稳态偏差，需按场景给出容差。
- **连续路径**：
  - 从当前位置出发，最多 128 个点，只在最后一点停下；
  - `max_velocity_rad_s`（≤0.35）和 `max_acceleration_rad_s2`（≤1.0）限制变化最大的关节；
  - 截止时间为曲线时长加 `timeout_s`。
- **定时轨迹**：
  - 最多 4000 个点、60 s，发送速率 20–500 Hz，单步不超过 3°，首点须是当前位置（0.5° 内）；
  - `effector_schedule` 为 `(at_s, positions, duration_s)` 列表，在机械臂运动期间按时下发末端动作；
  - 末端动作期间，收尾中的机械臂只能向上一条命令的目标收拢，其余偏离超过 1° 判为故障。

## 7. 运动学

`spacemit_manipulation.urdf_fk.UrdfKinematics(urdf, base, tip, joints)` 读取 URDF 关节链（参数取自机型的 `model.json`），不访问设备：

- `forward(joints_rad)` 返回 4×4 末端位姿（行主序 16 个数）；
- `limits` 为 URDF 关节限位。

`spacemit_manipulation.urdf_ik`（需要 `numpy`）：

| 函数 | 作用 |
| --- | --- |
| `pose(position_m, rpy_rad=None, orientation=None)` | 由位置和 URDF 横滚、俯仰、偏航角（或 3×3 旋转矩阵）构造 4×4 位姿 |
| `rpy(rotation)` | 旋转矩阵转横滚、俯仰、偏航角 |
| `solve(model, target, seed_rad, *, orientation_weight=0, limits=None)` | 阻尼最小二乘逆解，返回 `joints_rad`、`converged`、`position_error_m`、`orientation_error_rad`、`iterations` |
| `line(model, start_rad, target, *, step_m=0.005, orientation_weight=0, limits=None)` | 直线路点（关节角列表）；任一路点不可达或关节跳变超过 0.35 rad 时抛出 `ValueError` |
| `jacobian(model, joints_rad)` | 末端位姿与 6×N 几何雅可比（线速度行在前，基座坐标系） |

- **姿态权重**：`orientation_weight` 为 0 时只求位置，为 1 时位置与姿态同等加权。
- **收敛门槛**：位置 0.5 mm、姿态 0.01 rad，最多 200 次迭代；阻尼 1e-4，步长从 0.5 减半到 0.01，单步关节变化不超过 0.5 rad。
- **限位**：`limits` 为配置中的 `[下限, 上限]`，与 URDF 限位取交集。
- **未收敛结果**：`converged` 为 `false` 时只是最优近似，不能作为运动目标。
