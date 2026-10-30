# 机械臂控制组件 manipulation

C++17 单臂与末端执行器控制库，附带命令行工具和 Python 接口。
上层通过统一接口控制设备，具体型号由驱动实现，驱动通过注册接入。

## 功能

| 功能 | 说明 |
| --- | --- |
| 设备控制 | 机械臂关节点位、末端执行器多通道位置、状态读取 |
| 运行安全 | 设备独占、取消、超时、故障锁定；无法确认停止时返回 `stop_unknown` |
| 点位任务 | 在配置文件中定义一组点位，按顺序执行 |
| 诊断与取点 | 读取固件与状态、受限往返检查、记录当前姿态、URDF 正运动学 |
| 手眼标定 | 眼在手外：采集、求解、质量检查、导出外参（需 OpenCV） |
| Python 接口 | 通过本机控制进程调用上述功能 |

| 驱动 | 设备 | 说明 |
| --- | --- | --- |
| `mock_device` | 软件模拟的机械臂和末端，无需硬件 | 可用 |
| `agx_nero_revo2` | 松灵 Nero 七轴，可选右手 Revo2 | 关节点位与运动取消；连续路径不保证拐点加速度上限；Revo2 无位置反馈，开环控制 |

## 1. 安装

需要 Linux、CMake 3.16+、C++17 编译器、Python 3.10+。在 SDK 根目录执行：

```bash
source build/envsetup.sh
bash components/thirdparty/pyagxarm/tools/setup_sdk.sh
cd components/control/manipulation
mm --with-deps
```

`mm` 检查 `package.xml` 声明的系统依赖（yaml-cpp、python3-can 等）并提示安装，连同 pyAgxArm 一起构建，
安装到 `output/staging`。`envsetup.sh` 已把其中的 `bin/` 和 `python/` 加入 `PATH`、`PYTHONPATH`，
新终端重新 `source build/envsetup.sh` 即可。

Nero 驱动默认构建；系统装有 OpenCV 开发包时同时构建标定求解器。可用 `mm -DMANIPULATION_ENABLE_NERO=OFF`、
`mm -DMANIPULATION_ENABLE_CALIBRATION=OFF` 关闭。抓杯、标定工作流和骰子游戏统一在
`application/native/dice_arena` 构建。

## 2. 运行 Mock

```bash
manipulation demo
```

不连接硬件，执行两轴机械臂加六通道末端的点位任务。输出：

```text
{"firmware":"offline-mock-v1","enabled":1,"position_mode":1,"moving":0,"fault":0,"joints_rad":[0,0],"effector_positions":[0,0,0,0,0,0],"effector_fault":0}
{"status":"succeeded","physical_grip_verified":false}
```

## 3. 连接 Nero

### 运行前确认

- `can0` 已启用，速率 1 Mbps：`ip -details link show can0`
- 厂商 Web、遥操和其他控制程序已停止
- 机械臂已按厂商规程使能并处于位置控制模式；本工具不自动使能、清错或归位
- 现场有人看护，急停可达，工作区无人无障碍；Nero 断电会坍塌，不以断电或失能作为停止手段

### 机械臂

```bash
manipulation configure nero                # 生成设备配置，不连接设备
manipulation inspect                       # 连接并读取状态，不运动
manipulation arm-check                     # 运动：关节 1 往返 0.01 rad，检查取消与重连
manipulation teach pose_01.json            # 记录当前姿态
manipulation fk pose_01.json               # 计算该姿态的法兰位姿
```

`inspect` 正常输出 `"firmware":"1.20","enabled":1,"position_mode":1,"moving":0,"fault":0`。
`arm-check` 正常输出：

```text
point_roundtrip=succeeded target_reached: three distinct fresh stationary samples
cancel=cancelled cancel requested
return=succeeded
reconnected_actual_rad=[...] moving=0 fault=0
```

使用其他 CAN 接口时：`manipulation configure nero --set channel=can1`。

### 灵巧手 Revo2

经 Nero 末端口接入的 Revo2 静止时只上报手指电流，位置和状态仅在手指运动期间上报，无法用于到位判定，因此按无反馈方式开环控制：

```bash
manipulation configure nero --variant revo2_right
manipulation inspect                                         # 读取机械臂；手只确认在线
manipulation hand-move 0 0 0 0 0 0            # 运动：张开
manipulation hand-move 0 0 0.3 0 0 0          # 运动：食指移到 0.3
```

六个值依次对应 `thumb_tip`、`thumb_base`、`index_finger`、`middle_finger`、`ring_finger`、`pinky_finger`，
位置归一化到 `[0,1]`，0 为张开。默认执行时间 2 s（`--duration` 调整，最长 2.55 s）。

无反馈时手的当前位置未知，命令可能使手指移动任意距离，执行前确认手中无物、手指周围无人无障碍。
命令结果为 `unverified`：已下发并等待执行时间结束，但没有测量是否到位。
手能上报位置时用 `--set feedback=position` 生成配置，此时结果为 `succeeded`，并可使用 `hand-step` 单通道小步。

### 现场文件

设备配置、姿态和标定数据保存在现场目录，默认 `~/.local/share/spacemit-manipulation/site/`，
可用 `--site DIR` 或环境变量 `MANIPULATION_SITE` 指定。`configure` 不覆盖已有的 `device.yaml`，
更换配置时先删除它。

## 4. 手眼标定

眼在手外：相机固定，ChArUco 标定板装在法兰上。结果 `T_base_camera` 供视觉应用把相机坐标换算到机械臂基座。

**摇骰子现场（Nero + Revo2）** 使用抓杯摇杯应用中的标定程序（命令行 `calibrate.sh`、网页 `calibrate_web.sh`，
支持示教、自动采样、固定板恢复），见 [Dice Arena · 手眼标定](https://github.com/spacemit-robotics/dice_arena#手眼标定)。

其他机械臂可用通用命令，由操作者摆好姿态采集，不自动运动：

```bash
sudo apt install libopencv-dev python3-opencv
cd components/control/manipulation
mm                                                       # 重新构建，生成标定求解器
hand-eye collect cal_01 --option board=/path/board.json  # 每个姿态摆好后输入 CAPTURE
hand-eye solve cal_01                                    # 求解并检查质量
hand-eye promote cal_01                                  # 仅质量合格时生成 active.json
hand-eye export cal_01                                   # 导出 extrinsic.json
```

质量门槛默认为留出样本与全部样本残差不超过 5 mm / 2°；`solve --max-error-mm 15 --max-error-deg 5`
可放宽，上限 15 mm / 5°，实际门槛写入结果文件。所有结果写入新文件，不覆盖已有文件。

采样参数（`--option NAME=VALUE`，可重复）：

| 参数 | 说明 |
| --- | --- |
| `board=FILE` | 标定板 JSON：`dictionary`、`squares_x/y`、`square_length_m`、`marker_length_m`、`min_corners`、`max_reprojection_px`，`image_profile` 指定分辨率、帧率和裁剪 |
| `serial=SN` | RealSense 序列号，连接多台相机时必填 |
| `measured_width_mm` / `measured_height_mm` | 打印后实测的标定板尺寸 |
| `window=FILE\|x0,y0,x1,y1` | 标定板必须完整位于的图像区域 |

## 5. 在程序中使用

### C++

```cmake
find_package(manipulation CONFIG REQUIRED)
target_link_libraries(app PRIVATE spacemit::manipulation_motion spacemit::manipulation_drivers)
```

```cpp
#include "manipulation/device_plan.h"
namespace m = spacemit::manipulation;

auto registry = m::MakeBuiltinDeviceRegistry();
auto plan = m::LoadDevicePlan("device.yaml", registry);
m::DeviceSession session(plan.definition, registry.Create(plan.driver, plan.definition, plan.options));
session.Connect();                                    // 获取设备独占，不使能、不运动
auto state = session.ReadArm();
auto result = session.MoveArm({{0.2, -0.2}, 0.05});   // 阻塞至到位、失败或取消
```

用 `mm` 构建的组件自动以 `output/staging` 为 `CMAKE_PREFIX_PATH`，`find_package(manipulation)` 即可找到。

### Python

`source build/envsetup.sh` 后可直接导入：

```python
from spacemit_manipulation import DeviceClient

with DeviceClient("manipulation-device", "device.yaml") as arm:
    print(arm.state())
    arm.move_arm([0.2, -0.2], speed_scale=0.05)
    arm.move_effector([0.5] * 6, duration_s=0.5)
```

运动未成功时抛出 `DeviceCommandError`，其 `result` 字段包含结果状态。
无反馈末端的结果为 `unverified`，需传入 `move_effector(..., allow_unverified=True)` 才接受。

### 点位任务

在设备配置的 `steps` 中写入点位后执行，示例见 [配置式点位任务](examples/point_sequence/README.md)：

```bash
manipulation-device run device.yaml
```

## 6. 新增驱动

1. 实现 `Arm`，有末端时再实现 `EndEffector`，参考 `examples/bench_driver/bench_driver.cpp`。
2. 在模块的 `CMakeLists.txt` 中登记：
   ```cmake
   target_sources(manipulation_drivers PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/my_driver.cpp)
   manipulation_register_device(RegisterMyDevice)
   ```
3. 构建时指定模块目录，确认驱动已注册：
   ```bash
   cmake -S components/control/manipulation -B build -DMANIPULATION_EXTRA_DRIVER_DIRS=/path/to/my_driver
   cmake --build build && build/manipulation-device drivers
   ```
4. 在 `descriptions/robots/<型号>/devices/` 放置配置模板，`manipulation configure <型号>` 即可使用。

不需要修改公共头文件或已有驱动。接口约定与测试要求见 [开发指南](docs/development.md)。

## 7. 命令速查

| 命令 | 作用 | 是否运动 |
| --- | --- | --- |
| `manipulation check` | 检查运行依赖 | 否 |
| `manipulation demo` | 运行 Mock 点位任务 | 否（模拟） |
| `manipulation configure MODEL [--variant V] [--set k=v]` | 由型号模板生成 `device.yaml` | 否 |
| `manipulation validate` | 离线校验配置 | 否 |
| `manipulation inspect` | 读取状态 | 否 |
| `manipulation arm-check` | 关节 1 小范围往返检查 | 是 |
| `manipulation hand-move V...` | 末端各通道移到绝对位置 | 是 |
| `manipulation hand-step CH ±0.01` | 末端单通道小步（需位置反馈） | 是 |
| `manipulation teach FILE` / `fk FILE` | 记录姿态 / 正运动学 | 否 |
| `manipulation-device run PLAN` | 执行配置中的点位任务 | 是 |
| `hand-eye collect/solve/promote/export NAME` | 人工采集、求解、发布、导出 | 否 |

运动命令直接执行，不需要额外参数或输入确认。配置字段、返回值和 API 见 [接口参考](docs/reference.md)。

## 8. 使用限制

- 位置到位不代表实物已抓稳，`physical_grip_verified` 始终为 `false`。
- Ctrl+C 和停止信号请求软件保持：Nero 以不高于当前命令的速度回到停止时刻的位置，可能先越过再小幅回退。
  这不是急停，现场以急停为准。
- 设备独占覆盖本机上本组件的所有程序，以及使用 `/tmp/nero_<channel>_control.lock` 的 pyAgxArm 程序；
  不覆盖厂商 Web 和其他主机。
- 无反馈末端（如当前 Revo2）开环执行：结果 `unverified` 不代表到位，取消或中断后结果为 `stop_unknown`，
  须现场检查后重新连接。手侧（左/右）也无法确认，须核对实物。
- 仅支持单臂；不包含碰撞检测、连续轨迹和逆运动学。

## 目录结构

```text
include/manipulation/   公共接口
core/                   会话、独占、故障锁定
drivers/                mock、Nero、Revo2 驱动
capabilities/           点位、诊断、运动学、标定及其命令行工具
runtime/                配置解析
bindings/python/        Python 接口
descriptions/           机型描述：配置模板、URDF、关节顺序
examples/               Mock 配置、扩展驱动与 C++ 接入示例
tools/                  安装、依赖检查与验证脚本
```

## 许可

本组件代码使用 [Apache-2.0](LICENSE)。厂商 SDK pyAgxArm 使用 LGPL-3.0（见 `components/thirdparty/pyagxarm`），
Nero URDF 使用 MIT（见 `descriptions/robots/nero/LICENSE`）。
