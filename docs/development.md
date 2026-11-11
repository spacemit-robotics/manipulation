# 开发指南

本页说明组件结构、驱动约定和扩展方式。使用步骤见 [README](../README.md)，字段与 API 见 [接口参考](reference.md)。

## 1. 分层

```text
应用（application/native/*、ros2/*）     业务状态机、视觉、现场配置
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
| `drivers/simulation/mujoco/` | MuJoCo 机械臂与末端适配，共用一个物理世界 |
| `simulation/mujoco/` | 仓内 MuJoCo 物理世界、快照、RGB-D 相机与窗口 |
| `capabilities/<能力>/` | 可复用实现及其命令行入口 |
| `descriptions/robots/<型号>/` | 配置模板、URDF、`model.json`、限位来源与许可 |
| `descriptions/simulation/<场景>/` | MJCF 场景、关节与执行器映射、资源清单及模型许可 |
| `bindings/python/` | Python 客户端与命令行公共逻辑 |

## 2. 术语

| 术语 | 含义 |
| --- | --- |
| 机械臂 Arm | 产生关节运动的设备，不隐含存在末端 |
| 末端执行器 EndEffector | 夹爪或灵巧手；按通道控制，通道不一定对应 URDF 关节 |
| 设备组合 | 一个机械臂加可选末端；独立连接分别占有资源，共用连接显式注册共享工厂 |
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
- 驱动仍在分段下发目标时，`EffectorState::command_active` 为真；它与实际运动标志 `moving` 分开报告。到位或持物完成须等待下发结束，`Hold` 须先终止后续下发。
- `EffectorState::loads` 可报告各通道负载（`[-1,1]`），不支持时留空。
- 声明 `grasp_detection` 的驱动在 `EffectorState::holding` 报告抓握判断：闭合目标未到达、已停止且有负载证据时为真。
  会话把连续三个这样的样本视为抓取成功（`effector_grasp_holding`）；`Hold` 不得为此改写闭合目标而松开物体。
  未声明该能力的驱动不得设置 `holding`。

使能时先写入并读回当前位置目标，再改变扭矩，避免设备恢复之前保存的目标（参考 SO101 驱动）。

异常规则：

- 写入设备之前就拒绝（前置条件、参数）时抛出 `CommandRejected`，会话不锁定。
- 写入之后的任何失败抛出其他异常，会话尝试保持并锁定；无法确认停止时结果为 `stop_unknown`。
- 不支持的约束（如无法保证的速度）应拒绝，不要换算成别的单位。

驱动需要的路径通过注册描述的 `resolve_options` 声明，可复用 `ResolveResourceOptions`，
以配置文件所在目录为基准解析。

### 共享连接

| 资源标识 | 连接方式 |
| --- | --- |
| 不同 `resource_id` | 机械臂和末端分别调用各自工厂 |
| 相同 `resource_id` | 必须注册 `SharedDeviceDriver`，由其工厂创建两者并共享连接对象；未登记的同资源组合在连接前拒绝 |

共享连接示例位于 `drivers/common/agx_session/` 和 `drivers/common/so101_session/`。
会话先按名称排序、去重并锁定所有资源，再连接任何设备；任一锁或连接失败时释放全部资源。

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
       registry->RegisterArm({"my_arm", 1, ValidateArm, CreateArm});
       registry->RegisterEffector({"my_tool", 1, ValidateTool, CreateTool});
       registry->RegisterShared({"my_arm", "my_tool", ValidateShared, CreateShared});
   }
   }  // namespace spacemit::manipulation
   ```

   注册函数必须在 `spacemit::manipulation` 命名空间内且具有外部链接。
   只登记模块实现的角色；末端和共享工厂不是必填项。

   | 回调 | 输入 / 返回值 |
   | --- | --- |
   | `ValidateArm` / `ValidateTool` | 分别接收 `DeviceConfig` / `EffectorConfig`，另接收各自 `DriverOptions` |
   | `CreateArm` / `CreateTool` | 返回对应接口的 `unique_ptr` |
   | 共享回调 | 接收 `DeviceDefinition` 和两份选项，须校验连接参数一致；工厂返回 `Devices` |

   校验只检查配置，工厂只构造对象，均不访问硬件。
   `Register(DeviceDriver)` 继续用于兼容整机 `device.driver` 配置。

2. 在模块的 `CMakeLists.txt` 中登记：

   ```cmake
   target_sources(manipulation_drivers PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/my_driver.cpp)
   manipulation_register_device(RegisterMyDevice)
   ```

3. 构建时纳入模块：

   - 树外模块：用 `-DMANIPULATION_EXTRA_DRIVER_DIRS=<目录>` 指定。
   - 仓库内模块：在根 `CMakeLists.txt` 增加一个 `MANIPULATION_ENABLE_<型号>` 开关和 `add_subdirectory`。

   构建系统生成显式调用注册函数的代码，静态链接时不会被裁剪。

4. 在 `descriptions/robots/<型号>/` 提供 `devices/<variant>.yaml` 模板、`model.json`、URDF 和许可，
   README 写明关节顺序、单位和限位来源。

5. 测试：用测试替身驱动覆盖重复注册、越界、过期反馈、写入失败、取消、超时、断连和锁定，
   参考 `tests/arm_test.cpp` 与 `tests/test_nero.py`。真机测试前先在 Mock 和替身上通过。

`examples/bench_driver/` 是可直接构建的单轴示例。
其中 `composed.yaml` 把扩展机械臂与内置 Mock 末端组合，`bench.yaml` 使用整机配置。

### 仿真模型接入

`drivers/simulation/mujoco/` 实现统一 `Arm` 和 `EndEffector` 接口，物理步进由本仓库 `simulation/mujoco/` 的 `World` 提供。公共控制层不依赖 MuJoCo 类型。

物理世界、快照与渲染接口见[仿真模块说明](../simulation/mujoco/README.md)。

接入符合位置控制约定的其他机械臂或末端时：

1. 在 `descriptions/simulation/<场景>/` 放置 MJCF、资源清单与模型许可。
2. 提供 `mapping.yaml`，按名称映射关节和执行器；末端明确闭合、张开的控制值与反馈端点。
3. 自定义模型在设备配置中选择 `mujoco_mapped_arm` 和 `mujoco_mapped_effector`。随附 UR5e 与 Robotiq 2F-85 模型分别使用 `mujoco_ur5e` 和 `mujoco_robotiq_2f85`。共用一个世界时，使用相同的 `resource_id` 和映射文件。
4. 在 `examples/simulation/` 提供点位任务，检查实际反馈、停止、越界拒绝与超时。

符合通用映射约定的模型无需新增驱动或公共 API。需要力矩控制、复杂手部传动或新物理引擎时，
实现对应后端，定义其控制与反馈能力，再注册驱动。

MJCF 不替代运动学与手眼标定所需的 URDF 和 `model.json`。

共享窗口、相机和控制时，各接口的职责如下：

- `MujocoRuntime` 持有世界，通过 `CreateDevices()` 交给 `DeviceSession`。
- `Snapshot()` 供渲染和规划读取；规划只修改复制的数据，不直接写物理世界。
- `AttachWeld()` 在世界锁内重新检查双侧接触。动态接触约束属于场景操作，不能作为通用设备 API 的抓稳结果。

远程后端实现相同私有会话接口，服务端复用本地驱动。图像请求使用独立只读连接，不取得控制租约。

- **异常处理**：控制超时、断连和租约失效会停止参考更新并请求保持；通信失败不能证明物理停止。
- **反馈判定**：服务端时钟不与客户端绝对时间混用，命令序号和采样编号共同防止旧反馈被当作完成证据。

## 5. 新增能力与工具

能力是不依赖具体驱动的实现，放在 `capabilities/<能力>/`；命令行入口放在同目录的 `cli/`，只做参数解析和调用。

- 通过 `DeviceSession`（C++）或 `DeviceClient`（Python）访问设备，不直接创建驱动或导入厂商 SDK。
- 从 `device.yaml`、`describe` 和 `descriptions/` 取得关节、通道和模型，不写死型号。
- 运动命令直接执行，不需要额外参数；不自动使能、清错、归位或重试。只读工具用 `DeviceClient(..., allow_motion=False)`。
- 结果写入新文件，不覆盖已有文件。

### 注册顺序能力

`CapabilityRegistry` 将能力名映射到 `CapabilityDescriptor`：

| 字段 | 职责 |
| --- | --- |
| `id` / `description` | 配置名称与用途 |
| `parse` | 解码步骤，拒绝未知参数；自定义数据放入 `DeviceStep::parameters` |
| `unsupported_reason` | 根据实际驱动能力判断支持条件；支持时返回空字符串 |
| `validate` | 离线检查参数、关节数、通道数、限位与单位 |
| `execute` | 经 `DeviceSession` 执行，传递取消请求并保留原始结果语义 |

`parse`、`unsupported_reason`、`validate` 不访问硬件。执行前检查整个序列；实时状态条件仍由会话在下发前检查。

能力不能自动使能、清错或把 `unverified` 改为成功。
注册表在开始解析任务前装配，不支持并发修改。

模块用以下方式登记，不修改公共解析器和执行器：

```cmake
target_sources(manipulation_motion PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/my_capability.cpp)
manipulation_register_capability(RegisterMyCapability)
```

注册函数位于 `spacemit::manipulation` 命名空间，接收 `CapabilityRegistry*`。
构建时用 `MANIPULATION_EXTRA_CAPABILITY_DIRS` 指定模块目录。

在 SDK 根目录执行以下命令运行随附示例：

```bash
cd components/control/manipulation
cmake -S . -B build-capability -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    -DMANIPULATION_EXTRA_CAPABILITY_DIRS="$PWD/examples/capability_probe"
cmake --build build-capability -j2
build-capability/manipulation-device run examples/capability_probe/device.yaml
```

该示例注册 `assert_joint_position`，只读取 Mock 反馈。

### 复用手眼标定

公共模块提供采样、会话记录、求解、评估和格式导出；应用保留自动运动路线、桌面登记和业务批准流程。

| 应用 | 公共模块的使用方式 |
| --- | --- |
| Dice Arena | CLI/Web 使用相同的采样和求解模块；Nero 自动采样路线由应用管理 |
| `perceptive_grasp` | 真机执行器使用 `DeviceSession` 和运动学接口；标定采集使用只读 `DeviceClient` 与公共采样模块 |

公共 CLI 也可采集并导出 `perceptive_grasp` 的 `T_base_camera` 配置。导出仅新建外参片段，不切换应用控制后端。

| 模块 | 接口 |
| --- | --- |
| `sampling` | `capture_sample`：静止检查、多帧采集与一致性筛选 |
| `calibration_session` | `load_config`、`CalibrationSession`、`read_dataset`、`solve_session`、`export_perceptive` |
| `hand_eye` | `solve`、`evaluate_residuals`：眼在手外求解与残差评估，不访问设备 |

配置指定设备计划、关节模型、相机采样模块和安装标识。记录包含设备与模型摘要、关节顺序、相机序列号、内参、标定板几何及坐标系。

- **会话一致性**：同一会话不能混用身份或未完成的样本。
- **跨应用导出**：要求明确安装标识，并使用通过质量阈值的结果。
- **`export-perceptive`**：导出标识与采集时显式确认的 `installation_id` 相同，相机类型匹配且使用 `color_optical` 坐标系。

固定安装改变后重新标定，不改写标识复用旧结果。可执行流程见 [README 手眼标定](../README.md#6-手眼标定)。

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

### C++ 运动学

应用通过 `manipulation/kinematics.h` 创建独立求解器，链接 `spacemit::manipulation_kinematics`。

- **模型约定**：运动学不放入设备驱动；设备与模型使用相同的关节顺序，求解器不访问硬件。
- **算法扩展**：实现 `Kinematics`，通过 `KinematicsRegistry::Register` 注册工厂；未注册的名称报错，不回退到其他算法。
- **执行前检查**：求解结果只满足运动学与关节限位约束，应用负责碰撞和任务约束检查，再通过设备会话执行。

## 7. 验证

```bash
cd components/control/manipulation
bash tools/verify_stack.sh   # 全部软件测试，不访问硬件
```

该脚本：

- 构建并测试组件（含 Nero 替身和标定）、抓杯应用、扩展驱动示例、安装后的 C++ 接入和命令行；
- 检查安装目录移动后的重定位和符号链接；
- 测试使用独立的锁目录，不影响本机正在运行的程序。
