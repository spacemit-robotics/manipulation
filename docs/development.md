# 开发指南

本页说明组件结构、驱动约定和扩展方式。使用步骤见 [README](../README.md)，字段与 API 见 [接口参考](reference.md)。

## 1. 分层

```text
应用（application/native/*）            业务状态机、视觉、现场配置
    │ C++ DeviceSession 或 Python DeviceClient
    ▼
能力与工具（capabilities/、runtime/）     点位任务、诊断、标定、配置解析、命令行
    │ 只依赖公共接口
    ▼
公共接口与会话（include/、core/）         Arm / EndEffector 接口、注册表、独占、结果判定
    ▲ 实现接口并注册
    │
驱动（drivers/）                          型号协议、单位转换、厂商 SDK
    │
设备 / 厂商 SDK / 仿真
```

- 依赖只能向下。
- core 和 runtime 不出现型号名；工具通过 `device.yaml` 和 `describe` 获取关节与通道信息。
- 型号相关的配置模板和 URDF 放在 `descriptions/robots/<型号>/`。

| 目录 | 职责 |
| --- | --- |
| `include/manipulation/` | 公共头文件，安装后对外可见 |
| `core/` | `DeviceSession`、内部 `ArmSession`、设备独占、到位与故障判定 |
| `runtime/` | 解析 `device.yaml`，经注册表创建设备 |
| `drivers/<类别>/<型号>/` | 驱动实现与注册；`drivers/common/` 放共享连接 |
| `capabilities/<能力>/` | 可复用实现及其命令行入口 |
| `descriptions/robots/<型号>/` | 配置模板、URDF、`model.json`、限位来源与许可 |
| `bindings/python/` | Python 客户端与命令行公共逻辑 |

## 2. 术语

| 术语 | 含义 |
| --- | --- |
| 机械臂 Arm | 产生关节运动的设备，不隐含存在末端 |
| 末端执行器 EndEffector | 夹爪或灵巧手；按通道控制，通道不一定对应 URDF 关节 |
| 设备组合 | 一个机械臂加可选末端，共享一个 `resource_id` 和一个写入者 |
| 能力 | 基于公共接口的可复用实现，如点位任务、标定求解 |
| 工具 | 调用能力的命令行或界面入口，不包含能力逻辑本身 |
| 位置到位 | 新鲜反馈在误差内到达目标；不等于实物抓稳 |

## 3. 驱动约定

驱动实现 `Arm`，有末端时实现 `EndEffector`，由会话串行调用。

| 方法 | 要求 |
| --- | --- |
| `Connect` / `Disconnect` | 只建立和关闭通信；不使能、不清错、不归位、不恢复目标 |
| `Read` | 返回完整的一次采样：全部关节、状态标志、接收时间、样本号；缺帧或过期时抛出异常。`moving` 表示关节实际在动，不是“未到达目标” |
| `Diagnose` | 主动查询固件等参数，不运动 |
| `Enable` | 显式使能，仅在 `GetCapabilities().enable` 为真时调用 |
| `Disable` | 显式释放扭矩，仅在 `disable` 能力为真时调用；不能作为停止的替代 |
| `Submit` | 下发目标即返回，不等待完成；目标单位 rad，`speed_scale` 为控制器倍率。可能在运动中调用（接着未等到位的命令或目标跟随），是否允许由会话判断，驱动不因 `moving` 拒绝 |
| `SubmitPath`（可选） | 连续路径，下发即返回；`GetCapabilities().joint_path` 为真时调用。`tolerance_rad` 已由会话确定。执行期间 `Read` 须报告 `moving`，`Hold` 须先停止发送 |
| `Hold` | 新鲜反馈确认静止后返回 true；不以断电或失能代替；无法确认返回 false |

末端执行器 `EndEffector` 另需遵守：

- 位置统一为 `0` 闭合、`1` 张开，驱动负责转换为设备自身的刻度；上层命令和应用不感知具体设备。
- `feedback: none` 时 `Read` 只返回在线样本（`positions` 为空），不得用命令值冒充位置；会话据此开环执行，结果为 `unverified`。
- `EffectorState::enabled` 未知时保留 `std::nullopt`；声明 `enable` / `disable` 能力的驱动必须能读回扭矩状态。
- `EffectorState::loads` 可报告各通道负载（`[-1,1]`），不支持时留空。
- 声明 `grasp_detection` 的驱动在 `EffectorState::holding` 报告抓握判断：闭合目标未到达、已停止且有负载证据时为真。
  会话把连续三个这样的样本视为抓取成功（`effector_grasp_holding`）；`Hold` 不得为此改写闭合目标而松开物体。
  未声明该能力的驱动不得设置 `holding`。

使能时先写入并读回当前位置目标，再改变扭矩，避免设备恢复之前保存的目标（参考 SO101 驱动）。

异常规则：

- 写入设备之前就拒绝（前置条件、参数）时抛出 `CommandRejected`，会话不锁定。
- 写入之后的任何失败抛出其他异常，会话尝试保持并锁定；无法确认停止时结果为 `stop_unknown`。
- 不支持的约束（如无法保证的速度）应拒绝，不要换算成别的单位。

驱动需要的路径通过 `DeviceDriver::resolve_options` 声明，可复用 `ResolveResourceOptions`，
以配置文件所在目录为基准解析。

### 共享连接

臂和手共用一个控制器连接时，由同一个组合驱动创建两者并共享连接对象，参考
`drivers/common/agx_session/` 或 `drivers/common/so101_session/`。
SO101 经 `drivers/common/so101_session/feetech_bus.cpp` 调用 `motor` 组件的公开接口：

- `motor_alloc_uart` 打开串口，`motor_get_paras` / `motor_set_paras` 读写寄存器；
- 不调用 `motor_init`，因为它每次连接都改写 EEPROM 中的工作模式；
- `motor_set_paras` 不报告丢失的写入，所以每次写入后都读回确认。

厂商 SDK 运行在独立 worker 进程时：

- worker 只开放固定操作，不接受任意方法名。
- worker 忽略 SIGINT 和 SIGTERM，并放入独立进程组，只在所有者关闭通道后退出。
  终端 Ctrl+C 只到达所有者，所有者仍能通过 worker 下发保持。
- 应答中标明请求是否在写入设备前被拒绝（`rejected`），以区分可以继续的拒绝和需要锁定的失败。

## 4. 新增驱动

1. 在 `drivers/arms/<型号>/` 实现接口和注册函数：

   ```cpp
   namespace spacemit::manipulation {
   void RegisterMyDevice(DeviceRegistry* registry) {
       registry->Register({"my_driver", 1, Validate, Create});
   }
   }  // namespace spacemit::manipulation
   ```

   注册函数必须在 `spacemit::manipulation` 命名空间内且具有外部链接。
   `Validate` 只检查配置（关节顺序、通道、选项），不连接设备；`Create` 只构造对象。

2. 在模块的 `CMakeLists.txt` 中登记：

   ```cmake
   target_sources(manipulation_drivers PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/my_driver.cpp)
   manipulation_register_device(RegisterMyDevice)
   ```

3. 构建时纳入模块：树外模块用 `-DMANIPULATION_EXTRA_DRIVER_DIRS=<目录>`；
   放入本仓库时在根 `CMakeLists.txt` 增加一个 `MANIPULATION_ENABLE_<型号>` 开关和 `add_subdirectory`。
   构建系统生成显式调用注册函数的代码，静态链接时不会被裁剪。

4. 在 `descriptions/robots/<型号>/` 提供 `devices/<variant>.yaml` 模板、`model.json`、URDF 和许可，
   README 写明关节顺序、单位和限位来源。

5. 测试：用测试替身驱动覆盖重复注册、越界、过期反馈、写入失败、取消、超时、断连和锁定，
   参考 `tests/arm_test.cpp` 与 `tests/test_nero.py`。真机测试前先在 Mock 和替身上通过。

`examples/bench_driver/` 是可直接构建的单轴示例。

## 5. 新增能力与工具

能力是不依赖具体驱动的实现，放在 `capabilities/<能力>/`；命令行入口放在同目录的 `cli/`，只做参数解析和调用。

- 通过 `DeviceSession`（C++）或 `DeviceClient`（Python）访问设备，不直接创建驱动或导入厂商 SDK。
- 从 `device.yaml`、`describe` 和 `descriptions/` 取得关节、通道和模型，不写死型号。
- 运动命令直接执行，不需要额外参数；不自动使能、清错、归位或重试。只读工具用 `DeviceClient(..., allow_motion=False)`。
- 结果写入新文件，不覆盖已有文件。

手眼标定的相机采样是可替换模块（`--sampler`），提供 `create_sampler(options)`，返回的对象实现：

| 方法 | 要求 |
| --- | --- |
| `capture(port, prefix)` | 在机械臂静止时采样并返回记录（含 4×4 `T_base_flange`、`T_camera_board`）；图像以 `prefix` 为前缀写入新文件 |
| `provenance()` | 相机身份与内参（`camera`：`backend`、`frame`、`serial`、`camera_matrix`、`dist_coeffs` 等） |
| `dependencies()` | 需要记录摘要的文件 |
| `close()` | 释放相机 |

`port` 只读取机械臂反馈：`read_joints()` 返回新鲜关节，`read()` 返回 URDF 法兰位姿。
内置实现为 `spacemit_manipulation.charuco_sampler`（RealSense + ChArUco）。
采样判定（静止、丢弃旧帧、三帧一致）由 `capabilities/calibration/python/sampling.py` 提供，新采样模块可直接复用。

## 6. 应用接入

应用放在 `application/native/<应用>/`，通过公共接口使用设备：

- C++ 应用直接使用 `DeviceSession`；Python 应用使用 `DeviceClient`。
- 简单的顺序点位放在 `device.yaml` 的 `steps`；判断、重试、取消和恢复写在应用代码中。
- 应用配置中的路径相对配置文件解析，便于整体迁移。
- 结果区分命令完成、位置到位和实物抓稳；`physical_grip_verified` 只能由实际的抓持证据（如驱动的抓握判断）置为真。
- 末端位姿目标用 `spacemit_manipulation.urdf_ik` 求逆解，`converged` 为真后再交给 `DeviceClient` 执行。

## 7. 验证

```bash
cd components/control/manipulation
bash tools/verify_stack.sh   # 全部软件测试，不访问硬件
```

该脚本：

- 构建并测试组件（含 Nero 替身和标定）、抓杯应用、扩展驱动示例、安装后的 C++ 接入和命令行；
- 检查安装目录移动后的重定位和符号链接；
- 测试使用独立的锁目录，不影响本机正在运行的程序。
