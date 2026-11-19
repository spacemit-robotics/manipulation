# 接口参考

配置字段、命令行、结果状态和编程接口。上手步骤见 [README](../README.md)。

## 1. 设备配置

`manipulation configure MODEL` 按型号模板生成 `SITE/MODEL/device.yaml`，也可以手动编写。
文件为单个 YAML 文档，不超过 64 KiB；未知字段或重复字段一律拒绝。

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

`options` 中的路径由驱动声明，相对配置文件所在目录解析。

### 独立角色配置

与整机 `device.driver/options` 二选一，不能混用。其他公共字段和 `steps` 不变：

```yaml
schema_version: 3
device:
    arm:
        driver: mock
        device_id: arm
        resource_id: mock_arm
        joints: [{name: joint1, minimum_rad: -1, maximum_rad: 1}]
        options: {}
    effector:
        driver: mock
        device_id: tool
        resource_id: mock_tool
        channels: [finger]
        options: {}
steps: []
```

| 字段 | 说明 |
| --- | --- |
| `device.arm.driver` | 必填，已注册机械臂驱动 |
| `device.arm.options` | 该机械臂的字符串选项，最多 32 项；默认空 |
| `device.effector.driver` | 有末端时必填，已注册末端驱动 |
| `device.effector.resource_id` | 独立角色模式有末端时必填；整机模式省略时继承臂资源 |
| `device.effector.options` | 该末端的字符串选项，最多 32 项；默认空 |

资源标识只允许字母、数字、`_`、`-`。不同标识调用独立工厂；相同标识要求该驱动对已注册共享工厂，且连接选项一致。

`model` 标识机械臂关节链，不由末端型号推断。组合不改变设备的能力声明。

| 角色驱动 | 选项 |
| --- | --- |
| 机械臂 `mock`、末端 `mock` | 无 |
| 机械臂 `nero`、末端 `revo2` | `channel`、`python`、`sdk_root`、`worker`；各自资源标识须为 `agx_<channel>` |
| 机械臂 `so101` | `port`、`arm_ids`、`arm_calibration`、`max_velocity_rad_s` |
| 末端 `so101_gripper` | `port`、`gripper_id`、`gripper_calibration`、`max_velocity_rad_s`、`grip_load_threshold` |
| 机械臂 `mujoco_ur5e`、末端 `mujoco_robotiq_2f85` | UR5e 与 Robotiq 2F-85 模型；`mapping_file` 指定仿真映射文件，相对设备配置所在目录解析 |
| 机械臂 `mujoco_mapped_arm`、末端 `mujoco_mapped_effector` | 自定义模型；选项与型号仿真驱动相同，关节和执行器由映射定义 |

各硬件选项含义与下表的整机驱动一致。`nero + revo2`、`so101 + so101_gripper` 及同一映射下的仿真机械臂与末端可共享连接；SO101 共享配置中重复提供的选项须一致。

### 仿真映射

| 构建选项 | 构建内容 |
| --- | --- |
| `MANIPULATION_ENABLE_MUJOCO=ON` | 本地引擎与远程客户端 |
| `MANIPULATION_ENABLE_MUJOCO_REMOTE=ON` | 可单独构建远程客户端 |

| 驱动选项 | 默认值 | 含义 |
| --- | --- | --- |
| `mapping_file` | 必填 | 相对设备配置所在目录解析的映射文件 |
| `transport` | 本地 | `tcp` 选择远程连接；本地配置只填写 `mapping_file` |
| `host` | `127.0.0.1` | 远程连接的回环地址；跨机使用 SSH 转发 |
| `port` | `9100` | 远程服务端口 |

远程连接先比较角色、关节、限位和映射契约；不向服务端传递模型路径，也不加载客户端场景。

`maximum_age_s` 是接收端反馈时限，两端可分别配置：

- 服务端检查本地采样年龄。
- 客户端检查源采样年龄加网络往返耗时。

远程示例使用 `1.0` 秒，超过时限仍停止当前任务，不自动重试。

映射采用单个 YAML 文档，最多 256 KiB，拒绝未知或重复字段。

型号驱动还要求映射与对应设备一致：

**机械臂 `mujoco_ur5e`：**

- 六个关节依次为 `shoulder_pan_joint`、`shoulder_lift_joint`、`elbow_joint`、`wrist_1_joint`、`wrist_2_joint`、`wrist_3_joint`。
- 执行器名称分别去掉 `_joint`。
- 所有关节和执行器可使用同一个名称前缀，例如 `robot_`。

**末端 `mujoco_robotiq_2f85`：**

- 单个 `gripper` 通道，使用位置反馈。
- 关节为 `gripper_right_driver_joint`，执行器为 `gripper_fingers_actuator`；两者可使用同一个名称前缀。
- `joint_closed` / `joint_open` 为 `0.8` / `0.0`，`control_closed` / `control_open` 为 `255.0` / `0.0`，数值容差为 `1e-6`。

其他模型使用 `mujoco_mapped_arm` 和 `mujoco_mapped_effector`，按下表定义映射。

| 字段 | 类型 / 默认值 | 含义 |
| --- | --- | --- |
| `schema_version` | 整数，必填 `1` | 映射格式 |
| `scene` | 字符串，必填 | MJCF 文件，相对映射文件解析 |
| `initial_keyframe` | 字符串，可省略 | 初始关键帧；省略时使用模型默认状态 |
| `arm` | 列表，最多 16 项 | 与设备配置的关节名称和顺序一致 |
| `arm[].joint` / `actuator` | 字符串，必填 | 模型中的关节名和执行器名 |
| `arm[].max_velocity_rad_s` | 数值，必填，`(0,10]` | 速度倍率为 1 时的目标变化速率，rad/s |
| `effector` | 列表，最多 32 项 | 与设备配置的末端通道名称和顺序一致 |
| `effector[].channel` / `joint` / `actuator` | 字符串，必填 | 通道名、反馈关节名和执行器名 |
| `effector[].joint_closed` / `joint_open` | 数值，必填且不同 | 闭合 / 张开的关节坐标，使用模型关节单位 |
| `effector[].control_closed` / `control_open` | 数值，必填且不同 | 闭合 / 张开的执行器控制值 |
| `arm_moving_threshold_rad_s` | 数值，默认 `0.002` | 机械臂静止阈值，rad/s，`(0,0.1]` |
| `effector_moving_threshold` | 数值，默认 `0.002` | 末端静止阈值，归一化位置/s，`(0,0.1]` |
| `hold_timeout_s` | 数值，默认 `2` | 保持确认期限，s，`(0,10]` |
| `synchronized_joint_point` | 布尔，默认 `false` | 各关节参考值按共同时间插值；`false` 为各轴独立限速 |

模型与控制约束：

- **执行器**：机械臂使用转动关节和单位传动比的位置执行器；末端支持直接关节或固定关节腱传动的仿射位置执行器。
- **反馈对应**：执行器必须实际驱动对应的反馈关节，各角色的受控关节不能重叠；不支持空间腱传动。
- **初始状态**：须静止且控制值与位置匹配；连接时拒绝被模型禁用的执行器，不自动改写初始目标。
- **速度**：`speed_scale` 缩放目标变化速率，不保证带载时的实际关节速度。
- **末端位置**：需要 `feedback: position`，归一化位置始终为 `0` 闭合、`1` 张开，不报告抓稳判断；动作时长不得小于模型时间步。

物理软限位允许端点附近的小幅超出：不超过末端到位容差与 `0.01` 的较小值时，位置显示为对应端点；
超过该范围则报错。运动状态仍使用实际速度，不因端点显示而认定停止。

相同 `resource_id` 的仿真机械臂与末端必须引用同一个映射文件，并共用物理世界和步进线程。

| 模式 | 世界与控制连接 |
| --- | --- |
| 本地 | 不同资源标识创建不同世界，即使使用相同场景；世界随设备会话创建和销毁 |
| 远程 | 服务端持有世界，连接或关闭客户端不重置场景；服务端租约拒绝第二个控制者，客户端更换 `resource_id` 不能绕过它 |

引擎采样时间、位置和速度参与会话的到位、过期、取消与超时判定。

此后端仅提供关节点位、跟随、末端位置和保持；不提供使能、失能、连续路径、定时轨迹或抓稳检测。

### 仿真服务与相机

`manipulation-simulation --plan FILE [--port 9100] [--viewer] [--camera overview]`

仅接受本地 MuJoCo 设备配置，不执行其中的顺序任务。默认仅监听回环地址。
`--viewer` 和相机采集需要构建 `MANIPULATION_ENABLE_SIMULATION_RENDERER=ON`，并具备 X11 桌面。

`manipulation-simulation-camera --output DIRECTORY [--camera overview] [--host 127.0.0.1] [--port 9100] [--width 640] [--height 480]`

使用只读连接，不取得控制权。输出目录必须不存在。

| 输出 | 格式 |
| --- | --- |
| `rgb.ppm` | RGB8，左上原点 |
| `depth.f32` | 行优先、左上原点、little-endian float32；光学轴 Z 深度，单位 m，无效值为 `0` |
| `metadata.json` | 服务端 `epoch`；`data` 包含宽高、`sample_id`、`sim_time`、相机内参 `K` 和 `T_world_camera` |

- `K` 为行优先 3×3 矩阵，像素中心使用整数坐标。
- `T_world_camera` 为行优先 4×4 矩阵，把光学坐标（右、下、前）变换到世界坐标。

彩色图、深度图、内外参与样本编号来自同一快照；采集不推进物理时间。
相机的 `sample_id` 为引擎样本编号，设备状态的 `sample_id` 为控制会话反馈编号，不能直接比较。

服务器重启产生不同 `epoch`；客户端不会自动重连或重放运动。

| 现象 | 处理 |
| --- | --- |
| `X11 display unavailable` | 登录桌面并在该会话启动服务；纯远程客户端不需要显示器 |
| `remote arm stale/regressed feedback` | 检查网络延迟与客户端 `maximum_age_s`，保留服务端本地采样限制 |
| `real-time scheduling overrun` | 物理步进未能按时运行；减少并行负载，停止后重新创建仿真会话 |
| `simulation contract mismatch` | 对齐两端的角色、关节顺序、限位、映射及动作参数 |

通信失败不等于物理停止已确认；检查服务端状态后再重新连接，不自动重发失败的运动。

### 已注册顺序能力

每一步必须有 `capability`。参数解析、支持条件和执行函数均来自注册表；新增名称不改变解析器。
解析时检查全部步骤及实际驱动能力；执行时再次检查，实时起点和反馈由设备会话判断。

| 能力 | 必填字段 | 可选字段与默认值 |
| --- | --- | --- |
| `joint_point` | `target_rad`、`speed_scale` | `tolerance_rad: 0`、`wait_for_arrival: true`、`resend_interval_s: 0` |
| `effector_position` | `positions`、`duration_s` | 无 |
| `joint_jog` | `joint_index`（从 0 开始）、`delta_rad` | `speed_scale: 0.05` |
| `joint_path` | `waypoints_rad`（1–128 个关节点） | `max_velocity_rad_s: 0.2`、`max_acceleration_rad_s2: 0.2`、`speed_scale: 0.05`、`tolerance_rad: 0` |
| `joint_trajectory` | `times_s`、`positions_rad` | `command_rate_hz: 100`、`max_step_rad: 0.0174533`、`speed_scale: 1`、`tolerance_rad: 0`、`wait_for_arrival: true` |
| `arm_hold` / `effector_hold` | 无 | 无；要求驱动能确认停止 |

- 关节目标为 rad，维数和限位与设备配置一致；`speed_scale` 为 `(0,1]` 且不低于驱动下限。
- `positions` 按末端通道排列，每项 `[0,1]`；`duration_s` 大于 0，且不超过驱动与末端期限。
- 路径和轨迹的范围、起点及到位条件与同名设备 API 一致。顺序轨迹不提供 `effector_schedule`，该组合仍通过直接 API 使用。
- `joint_jog` 的最终目标由实时起点计算，下发前检查限位。
- SO101 的 `joint_jog` / `joint-step` 只更新选中关节，保留其他舵机已有目标；选中关节到位且全臂静止后完成。`joint_point` / `joint-move` 仍按整组目标判定到位。
- `unverified` 不等于到位；顺序任务保留该结果，失败或取消后不执行后续步骤。

### 末端反馈

| `feedback` | 连接要求 | 移动结果 | 取消或中断 |
| --- | --- | --- | --- |
| `position` | 新鲜、完整的位置和状态反馈 | 目标下发完毕，连续 3 个样本在容差内且静止时 `succeeded` | 驱动确认停止为 `cancelled`，否则 `stop_unknown` |
| `none` | 只要求新鲜的在线样本 | 下发后等待 `duration_s`，结果 `unverified` | 始终 `stop_unknown` |

- 所有末端执行器的位置含义相同：`0` 为闭合，`1` 为张开，由驱动转换为设备自身的刻度。
- `none` 只降低验证要求，不会让驱动声称具备反馈。
- 经 Nero 末端口接入的 Revo2 静止时不上报位置，须使用 `none`；此时以手指电流帧作为在线证据，手部状态显示为 `unknown`。

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
- 目标和扭矩逐个舵机写入并读回确认，不是同步写入，各关节的开始时刻相差几毫秒。
- **控制范围**：取配置限位与校准行程的交集。
  - 配置模板中前四个关节的限位为 ±π，实际范围由每台机械臂的校准行程决定；
  - `wrist_roll` 可整圈转动，校准行程为 0–4095，由配置限位（−157°～+163°，取自模型）约束；
  - 超出范围时拒绝使能或运动，报错列出每个超限关节的当前位置和允许范围。
- **校准文件格式**：
  - 机械臂按关节名保存 `id`、`homing_offset`、`range_min`、`range_max`；
  - 夹爪保存 `motor_id`、`open_ticks`、`closed_ticks` 和 `homing_offset`；
  - 夹爪文件可省略 `homing_offset`，此时驱动不核对夹爪偏移，该文件只能用于偏移未改变的安装。
- **抓握判断**（能力 `grasp_detection`）：
  - 下发目标对应实际闭合方向、闭合目标未到达、夹爪静止，且负载绝对值不低于 `grip_load_threshold` 时，`effector_holding` 为 `true`；
  - 连续三个这样的样本使动作以 `effector_grasp_holding` 成功结束；
  - 张开超调、目标不变时的漂移或单独的负载读数不构成夹持证据；
  - 夹住时舵机的过载保护标志（状态位 5）不算故障；
  - `hand-hold` 保持原闭合目标。
- 使能夹爪前，核对最大扭矩不高于 500、保护电流不高于 250 mA。
- 动作中反馈明确失去扭矩时中断并尝试保持；无法确认停止时返回 `stop_unknown` 并锁定会话。
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
| `driver-roles` | 分别列出已注册机械臂与末端驱动，不连接设备 |
| `validate PLAN` | 离线校验配置，不导入厂商 SDK |
| `describe PLAN` | 输出驱动、型号、关节名与限位、机械臂容差、机械臂与末端 `timeout_s`、通道名（JSON），不连接设备 |
| `capabilities PLAN` | 离线查询驱动能力，以及 `registered` 列表中的能力 `id`、`supported`、`reason`、`description` |
| `inspect PLAN` | 获取独占、连接、诊断并输出状态 |
| `run PLAN [--enable]` | 执行 `steps`；`--enable` 先显式使能；`unverified` 步骤不中断序列 |
| `arm-check PLAN [--timeout-only]` | 关节 1 往返 0.01 rad，再测试取消和重连；`--timeout-only` 改为检查 1 ms 期限的超时锁定 |
| `serve PLAN [--read-only \| --enable]` | 本机 JSONL 控制进程，供 Python 接口使用；`--read-only` 拒绝一切运动 |

`serve` 的 `--allow-disable`、`--allow-effector-enable`、`--allow-effector-disable` 分别授权机械臂失能、末端执行器使能和失能，默认均关闭。

| 退出码 | 含义 |
| --- | --- |
| `0` | 成功 |
| `1` | 任务未成功或被中断 |
| `2` | 参数、配置或连接错误 |
| `3` | 全部步骤完成，但含 `unverified` 步骤 |

`run` 最后一行是汇总结果，形如 `{"status":"succeeded","physical_grip_verified":false}`。

独立组合的 `describe` 另含 `arm_driver`、`effector_driver`、`arm_resource_id`、`effector_resource_id`；`driver` 为 `composed`。

**状态字段**（`inspect` 输出与 Python `state()` 返回同一份 JSON）：

| 字段 | 含义 |
| --- | --- |
| `enabled`、`position_mode`、`moving`、`fault` | 机械臂使能、位置模式、实际运动、故障（`true` / `false`） |
| `latched` | 会话是否已锁定，锁定后须重新连接 |
| `joints_rad` | 关节位置，按配置顺序 |
| `joint_reference_verified` | `true` 表示驱动已核对当前关节参考与安装校准一致；`false` 为未校准；未提供验证时为 `null`。不表示手眼外参有效 |
| `effector_feedback` / `effector_positions` | 末端反馈类型 / 各通道位置（`0` 闭合，`1` 张开）；反馈类型为 `none` 时位置为空数组 |
| `effector_moving` / `effector_command_active` | 末端实际运动状态 / 驱动是否仍在下发当前目标 |
| `effector_enabled` | `true`、`false`，驱动无法读回时为 `null` |
| `effector_loads` | 各通道负载 `[-1,1]`（额定输出的比例，符号为驱动方向），不提供时为空数组 |
| `effector_holding` | 抓握判断；仅声明 `grasp_detection` 的末端给出 `true` / `false`，其他为 `null` |
| `controller_limits` | 控制器报告的各关节位置、速度、加速度限制（驱动提供时） |
| `firmware`、`arm_sample_id`、`source_time_s` | 固件信息、反馈样本序号、设备时间戳 |

### manipulation / hand-eye

- 全局选项 `--site DIR` 写在子命令前，默认为 `$MANIPULATION_SITE` 或 `~/.local/share/spacemit-manipulation/site`。
- 每台设备一个目录 `SITE/NAME/`，存放 `device.yaml`、姿态和校准文件；`SITE/device.yaml` 是指向当前设备配置的符号链接。
- 诊断与单次运动子命令默认使用当前设备，可用 `--plan FILE` 替换；`run` 必须显式提供 `--plan FILE`。
- `configure` 复用已有配置时先离线校验并核对型号，保留文件内容后选中；失败时保持原选择。`--variant`、`--set` 和 `--python` 只用于生成新文件。
- 所有命令都不自动使能、清错或归位。

| 子命令 | 参数与说明 |
| --- | --- |
| `check` | `--profile build\|control\|vision`；`vision` 另检查 OpenCV、SciPy、`pyrealsense2` |
| `drivers [--roles]` | 只列已启用构建的驱动；默认整机注册列表，`--roles` 改为独立角色列表；仿真驱动按机械臂和末端角色列出 |
| `configure MODEL` | 创建或复用 `SITE/MODEL/device.yaml` 并设为当前设备；创建选项：`--variant NAME`（默认 `arm`）、`--set NAME=VALUE`、`--python CMD` |
| `use [NAME]` | 把 `SITE/NAME/device.yaml` 设为当前设备；省略 `NAME` 时只列出已配置的设备，`*` 标记当前设备 |
| `validate` / `inspect` / `capabilities` | 同 `manipulation-device` 对应子命令 |
| `run --plan FILE [--enable]` | 执行指定计划中的顺序任务；默认不使能 |
| `monitor` | 只读，持续打印关节角（度）、状态、末端位置、负载与抓握判断；`--rate HZ`（1–20，默认 5）、`--count N` |
| `enable` / `disable`、`hand-enable` / `hand-disable` | 使能 / 释放扭矩；失能前须支撑机械臂并取下所持物品 |
| `hold` / `hand-hold` | 保持当前位置，不释放扭矩 |
| `joint-step JOINT DELTA` / `joint-move Q...` | 单关节相对 / 全部关节绝对点位，单位 rad；`JOINT` 为关节名或从 0 开始的序号；`--speed`（默认 0.05） |
| `arm-check` | 同 `manipulation-device arm-check`，会运动 |
| `hand-move V...` | 每个通道一个绝对位置（`0` 闭合，`1` 张开）；`--duration S`（默认 2） |
| `hand-step CHANNEL DELTA` | 选中通道 `±0.01`，执行时长 1 s；仅限有位置反馈的末端 |
| `hand-grasp` | 闭合到夹住或完全闭合，仅限声明 `grasp_detection` 的末端；`--positions V...`（默认全 0）、`--duration S`（默认 1） |
| `teach FILE` / `fk FILE` | 记录静止姿态 / 姿态文件的正运动学；文件相对设备配置所在目录，`teach` 不覆盖 |
| `pose` | 只读当前末端位姿：`position_m`、`rpy_rad`（URDF 横滚、俯仰、偏航） |
| `ik X Y Z` | 逆解，不运动；`--rpy R P Y`、`--orientation-weight W`、`--seed-pose FILE`（离线求解）；不收敛时退出码 2 |
| `pose-move X Y Z` | 逆解后关节点位运动；参数同 `ik`，另有 `--speed`；按控制器速度上限估算的时长超过 `timeout_s` 时不运动 |
| `line-move X Y Z` | 末端直线运动；另有 `--step-mm`（默认 5）；支持连续路径的驱动另用 `--path-velocity`、`--path-acceleration` |
| `hand-eye configure FILE` | 新建采集配置；必填 `--installation-id ID`、`--board FILE`，可选 `--plan FILE`、`--serial SN` |
| `hand-eye collect NAME` | `--config FILE`，或 `--plan FILE`、`--sampler MODULE`、`--option NAME=VALUE`（见下）；`--count N`（12–100，默认 12） |
| `hand-eye solve NAME` | 生成 `candidate.json`；`--max-error-mm`（默认 5，≤15）、`--max-error-deg`（默认 2，≤5） |
| `hand-eye promote` / `export` / `restore NAME` | 分别生成 `active.json`、`extrinsic.json`、`restored.json` |
| `hand-eye export-perceptive NAME` | 从 `active.json` 新建抓取外参片段；必填 `--installation-id ID`、`--output FILE`；`--camera-type realsense\|spacemit_las2`（默认 `realsense`） |

`joint-step` 从新鲜位置反馈计算相对目标。SO101 只更新选中关节并保留其他舵机目标；未声明
`single_joint_point` 的驱动从全臂反馈生成整组目标，按整组点位执行。`hand-step` 下发完整末端
目标向量，选中通道增减 `0.01`，其他通道取本次反馈位置。

| 命令 | 请求期限（s） |
| --- | --- |
| `joint-step`、`joint-move`、`pose-move`、`line-move` 逐点执行 | `max(10, arm.timeout_s + 5)` |
| `hand-move`、`hand-step`、`hand-grasp` | `max(10, effector.timeout_s + 5)` |

- **请求预算**：在动作期限之外至少预留 5 秒用于反馈读取与停止确认，满足完成条件即返回。设备仍按配置的动作期限超时。单次请求最长 180 s，表中的机械臂点位命令要求 `arm.timeout_s ≤ 175 s`。
- **时长预检**：驱动提供关节速度上限时，`joint-move` 和 `pose-move` 按实测位置、速度上限和配置到位容差估算最短时长；超过 `arm.timeout_s` 的目标在下发前拒绝。
- **整组到位**：`joint-move` 须全部关节在目标容差内且全臂静止，并由连续三个新鲜反馈确认；仅静止不代表完成。
- **末端完成**：有位置反馈时等待到位或有效夹持证据，无反馈时等待指定动作时长后返回 `unverified`。等待期间失去已知的使能状态会中断动作并进入停止确认。

`hand-eye collect` 的 `--option` 可重复：

| 选项 | 含义 |
| --- | --- |
| `board=FILE` | 必填，ChArUco 标定板 JSON：`type`、`dictionary`、`squares_x/y`、`square_length_m`、`marker_length_m`；可选 `min_corners`、`max_reprojection_px` 和 `image_profile`（`color_resolution`、`fps`、`crop_xywh`） |
| `serial=SN` | RealSense 序列号，连接多台相机时必填 |
| `measured_width_mm` / `measured_height_mm` | 成对填写打印后的实测宽、高 |
| `window=FILE\|x0,y0,x1,y1` | 标定板须完整位于的图像区域 |

求解残差阈值同时适用于全部样本与留出样本，实际使用的阈值写入结果。

### 手眼采集配置

JSON 格式；路径相对配置文件解析。`--config` 不能与 `--plan`、`--sampler` 覆盖或 `--option` 混用。

| 字段 | 默认 / 含义 |
| --- | --- |
| `schema_version` | 必填，`1` |
| `installation_id` | 必填，固定机械臂与相机安装的非空标识 |
| `device_plan` / `model` | 必填，设备 YAML 与关节链 `model.json` |
| `sampler.module` | `spacemit_manipulation.charuco_sampler` |
| `sampler.options` | 字符串键值，内置采样器要求 `board` |
| `sampler.path_options` | 文件类选项名，默认 `["board", "window"]`；矩形数值形式的 `window` 不作路径处理 |
| `frames.base` / `frames.flange` | 默认模型 `base` / `tip`，指定值须与模型相同 |
| `frames.camera` / `frames.target` | `color_optical` / `calibration_board` |

共享会话包含 `manifest.json`、样本和绑定的设备、模型、相机、标定板及坐标系身份。
采集未完成、身份混用或依赖文件变化会被拒绝。旧 `dataset.json` 及旧 Dice 会话继续按原格式读取。

跨应用导出条件：

- **安装标识**：`--installation-id` 与结果绑定的 `installation_id` 相同；自动生成的会话编号不能替代显式确认的安装标识。安装变化后重新标定。
- **相机与坐标系**：相机类型匹配，坐标系为 `color_optical`。
- **结果质量**：通过质量门限，并已执行 `promote`；命令读取会话内的 `active.json`。
- **输出文件**：必须不存在，命令不修改应用配置。

`export-perceptive` 写入 `calibration.<camera_type>.T_base_camera`，方向为相机到机械臂基座：

| 字段 | 单位与约定 |
| --- | --- |
| `translation` | m |
| `rotation` | XYZ RPY rad |

### so101-calibrate

`so101-calibrate OPERATION --port PORT`：除 `status` 外，所有操作要求舵机已释放扭矩；工具从不使能扭矩。

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
| `idle` | 没有执行任何动作，例如 `run` 的 `steps` 为空；`run` 以退出码 `1` 结束 | 可继续 |
| `succeeded` | 满足所选完成条件（见 [§6 完成条件](#完成条件)） | 可继续 |
| `cancelled` | 取消后驱动确认已停止 | 可继续 |
| `timed_out` | 超过期限，驱动确认已停止 | 锁定 |
| `fault` | 反馈异常、控制器故障或写入失败，已确认停止 | 锁定 |
| `stop_unknown` | 无法确认设备已停止，包括反馈中断或驱动拒绝时的保持 | 锁定 |
| `unverified` | 无反馈末端：已下发并等完执行时间，位置未测量 | 可继续 |

- **锁定**：锁定后拒绝新的运动和使能，直到断开并重新连接。重连只读取实际状态，不重放目标、不归位、不使能。
- **写入前拒绝**：`CommandRejected` 表示请求在写入设备前被拒绝，不改变状态、不锁定。其子类 `BusyRejected` 表示末端仍在运动或执行命令；调用方可等待新的静止反馈后有限重试。其他异常或已提交命令的失败不能按此规则重试。
- **软件保持**：Ctrl+C 和停止信号请求软件保持。Nero 以不高于当前命令的速度回到停止时刻的位置，可能先越过再小幅回退。这不是急停。
- **抓稳证据**：`physical_grip_verified` 只在末端按抓握判断结束（`detail` 为 `effector_grasp_holding`）时为 `true`。

## 4. 设备独占

- **资源锁**：连接时在 `/run/lock/spacemit-manipulation/<resource_id>.lock` 上加排他锁，进程退出时释放。
  - 独立组合按标识排序、去重并获取全部资源锁后才连接设备；任一失败释放已取得的资源。
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
| `types.h` | `DeviceConfig`、`Result`、`CommandResult`、`CommandRejected`、`BusyRejected` |
| `arm.h` | 驱动接口 `Arm`、`ArmState`、`JointPoint`、`JointPath`、`JointTrajectory` |
| `device.h` | 驱动接口 `EndEffector`、注册表 `DeviceRegistry`、会话 `DeviceSession` |
| `device_plan.h` | `LoadDevicePlan`、`RunDeviceSteps` |
| `device_composition.h` | `DeviceComposition`、`ParseDeviceComposition` |
| `capability.h` | `CapabilityRegistry`、`CapabilityDescriptor`、`MakeBuiltinCapabilityRegistry` |
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
| `JogArm(joint_index, delta_rad, speed_scale, cancelled)` | 基于当前位置的单关节相对点位；支持 `single_joint_point` 的驱动只更新选中关节，其余驱动使用整组点位执行 |
| `MoveArm` / `MoveArmPath` / `MoveArmTrajectory` / `MoveEffector` | 阻塞执行并返回 `CommandResult`；`cancelled` 每 10 ms 调用一次 |
| `FollowArm(point)` / `FollowEffector(point)` | 目标跟随：发出目标即返回，见 [§6 完成条件](#完成条件) |
| `Latched()` | 是否处于锁定 |

设备装配与注册：

- `CreatePlanDevices(plan, registry)` 同时支持整机与独立角色配置。
- `LoadDevicePlan` 和 `RunDeviceSteps` 支持传入能力注册表的重载；未传入时使用构建中登记的能力。
- `DeviceRegistry::RegisterArm` / `RegisterEffector` / `RegisterShared` 分别注册单角色与共享工厂；`Register` 保持整机接口。

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
| `execute_capability(name, parameters=None, *, allow_unverified=False)` | 调用已注册能力；`parameters` 为该能力的字段字典，不能覆盖 `capability`；沿用取消、期限与运动授权 |
| `follow_effector(positions, duration_s=0.1, allow_unverified=False)` | 末端目标跟随，发出目标即返回 |
| `close()` | 关闭会话；也可使用 `with` |

- 每个方法接受 `timeout`（秒，默认 10，最长 180，含排队时间）和 `is_cancelled`（回调）。
- Python 请求期限由调用者设置，与设备动作期限独立；需要覆盖动作期限、反馈读取和停止确认时间，命令行的自动预算不会应用到直接 API 调用。
- 运动未成功时抛出 `DeviceCommandError`：`result` 为含 `status` 的字典；设备进程拒绝请求时为错误字符串，此时 `rejected` 表示请求在写入设备前被拒绝，`latched` 表示会话已锁定、须重新连接。

### 完成条件

| 用法 | 返回时机 | `detail` 开头 |
| --- | --- | --- |
| 整组点位（默认） | 连续三个新鲜反馈全部关节在容差内且全臂静止 | `target_reached` |
| SO101 `jog_arm` | 只更新选中关节；连续三个新鲜反馈选中关节在容差内且全臂静止 | `target_reached` |
| 有位置反馈的末端 | 连续三个新鲜反馈全部通道在容差内且静止 | `effector_position_reached` |
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
- **Nero 运动判定**：Nero 的“运动中”由关节反馈测得（0.2 s 内变化超过 0.05°）；停在目标外超过 1 s 时原样重发，最多 5 次。连续发送时控制器会留下零点几度的稳态偏差，须按场景设置容差。
- **连续路径**：
  - 从当前位置出发，最多 128 个点，只在最后一点停下；
  - `max_velocity_rad_s`（≤0.35）和 `max_acceleration_rad_s2`（≤1.0）限制变化最大的关节；
  - 截止时间为曲线时长加 `timeout_s`。
- **定时轨迹**：
  - 最多 4000 个点、60 s，发送速率 20–500 Hz，单步不超过 3°，首点须是当前位置（0.5° 内）；
  - `effector_schedule` 为 `(at_s, positions, duration_s)` 列表，在机械臂运动期间按时下发末端动作；
  - 末端动作期间，收尾中的机械臂只能向上一条命令的目标收拢，其余偏离超过 1° 判为故障。

## 7. 运动学

### C++ 接口

头文件 `manipulation/kinematics.h`，链接目标 `spacemit::manipulation_kinematics`。

内置注册表提供两个后端，均可独立开关：

| 后端 | 开关 | 默认 | 依赖 |
| --- | --- | --- | --- |
| `eigen` | `MANIPULATION_ENABLE_KINEMATICS_EIGEN` | `ON` | 仅 Eigen 头文件 |
| `pinocchio` | `MANIPULATION_ENABLE_KINEMATICS_PINOCCHIO` | `OFF` | 需自行编译安装 Pinocchio |

`eigen` 后端自带 URDF 读取、变换链正解和阻尼最小二乘逆解，不依赖 urdfdom、Boost 或任何外部运动学库。两个后端对同一 URDF 与关节序列给出一致的正解结果，可在配置中用 `kinematics_solver` 切换，无需改代码。同时启用时两个后端都注册，配置按名字选择。

| 接口 | 含义 |
| --- | --- |
| `KinematicsConfig` | URDF 路径、基座、末端及从基座到末端的关节顺序；支持有界旋转关节链，顺序须与设备一致 |
| `Forward(joints_rad)` | 返回基座下末端位姿，单位米，四元数顺序 `qw,qx,qy,qz`；允许读取限位外的实际姿态 |
| `Inverse(target, seed, options)` | 有限时逆解；解在 URDF 限位内，无解返回 `nullopt`，非法输入抛出异常 |
| `JointNames()` / `LowerLimits()` / `UpperLimits()` | 配置顺序的关节名和限位 |
| `KinematicsRegistry::Register(id, factory)` | 注册求解器，不修改设备接口或具体设备驱动 |
| `MakeBuiltinKinematicsRegistry().Create(id, config)` | 创建已启用的求解器，不连接设备 |

`InverseOptions` 默认时限 `0.1 s`、迭代上限 `1000`、加权误差阈值 `1e-4`。

`position_weight` 在 `[0,1]` 内：`1` 只求位置，`0` 只求方向，其他值对位置和方向按互补权重求解。

求解结果不是碰撞规划；应用仍需检查工作空间、设备限位与完整路径。

### Python 接口

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
- **收敛阈值**：位置 0.5 mm、姿态 0.01 rad，最多 200 次迭代；阻尼 1e-4，步长从 0.5 减半到 0.01，单步关节变化不超过 0.5 rad。
- **限位**：`limits` 为配置中的 `[下限, 上限]`，与 URDF 限位取交集。
- **未收敛结果**：`converged` 为 `false` 时只是最优近似，不能作为运动目标。
