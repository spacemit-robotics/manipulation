# 机械臂控制组件 manipulation

统一控制机械臂与末端执行器：用设备配置选择硬件，用同一套命令或 C++ / Python 接口下发动作，并提供运动学、手眼标定和设备工具。

| 驱动 | 设备 | 说明 |
| --- | --- | --- |
| `mock_device` | 软件模拟 | 无需硬件 |
| `agx_nero_revo2` | 松灵 Nero 七轴，可选 Revo2 右手 | 点位、连续路径、定时轨迹；Revo2 无位置反馈 |
| `so101` | SO101 五轴，可选原配夹爪 | 点位、使能与失能；夹爪按负载判断是否夹住；附舵机校准工具 |

组件不自动使能、清错、归位或重试。设备独占、到位判定、取消、超时和故障锁定由设备会话统一处理。

## 1. 安装

在 SDK 根目录执行：

```bash
source build/envsetup.sh
bash components/thirdparty/pyagxarm/tools/setup_sdk.sh
cd components/control/manipulation
mm --with-deps
```

组件安装到 `output/staging`，缺少系统依赖时按提示安装。新终端需重新执行 `source build/envsetup.sh`。
构建选项 `MANIPULATION_ENABLE_NERO`、`MANIPULATION_ENABLE_SO101` 默认开启；检测到 OpenCV 时同时构建标定求解器（`MANIPULATION_ENABLE_CALIBRATION`）。

## 2. 运行 Mock

```bash
manipulation demo
```

执行两轴机械臂和六通道末端的模拟点位任务，不连接硬件。

## 3. 连接 Nero

Nero 经 SocketCAN USB-CAN 适配器以 1 Mbps 连接主机；Revo2 右手接在机械臂末端口，与机械臂共用连接。退出厂商 Web、遥操等其他控制程序后执行：

```bash
sudo ip link set can0 up type can bitrate 1000000
manipulation configure nero --variant revo2_right   # 仅机械臂时去掉 --variant；其他接口加 --set channel=can1
manipulation inspect
manipulation teach pose_01.json
manipulation hand-move 0 0 0.3 0 0 0
```

- **现场目录**：配置保存到 `SITE/device.yaml`，已有文件不覆盖。`SITE` 默认为 `~/.local/share/spacemit-manipulation/site/`，可用 `MANIPULATION_SITE` 或子命令前的 `--site DIR` 指定。
- **只读命令**：`inspect` 读取状态，`teach` 记录静止姿态，都不运动。
- **灵巧手**：`hand-move` 按 `thumb_tip`、`thumb_base`、`index_finger`、`middle_finger`、`ring_finger`、`pinky_finger` 的顺序给出 `[0,1]` 位置（0 为张开），默认执行 2 s。Revo2 没有位置反馈，结果为 `unverified`（已下发并等完执行时间，未测量是否到位），起点未知时可能大幅动作。

机械臂须按厂商规程使能并处于位置模式。现场须有人看护、急停可达。Nero 失能或断电会坍塌，不以失能或断电代替软件停止。

## 4. 连接 SO101

五个关节舵机为 ID 1–5，夹爪为 ID 6，共用 USB 串口（1 Mbps）。账号须属于 `dialout` 组，并退出 LeRobot 等串口程序。

### 校准舵机

已有原 `manipulator` / `grasp` 的 `so101_calibration.json`、`so101_gripper_calibration.json` 时，放入现场目录即可跳过。新装或重新装配时，先支撑机械臂、清空夹爪并释放扭矩，再执行：

```bash
export MANIPULATION_SITE="$PWD/site/so101" && mkdir -p "$MANIPULATION_SITE"
so101-calibrate --port /dev/ttyACM0 status       # 只读，每个舵机一行 JSON
so101-calibrate --port /dev/ttyACM0 configure    # 推荐参数：PID 16/0/0、返回延迟 0、位置模式、夹爪扭矩保护
so101-calibrate --port /dev/ttyACM0 center       # 中位校准：当前姿态设为编码器中位 2047（模型零位）
so101-calibrate --port /dev/ttyACM0 \
  --arm-output "$MANIPULATION_SITE/so101_calibration.json" \
  --gripper-output "$MANIPULATION_SITE/so101_gripper_calibration.json"   # 采集行程（只读）
so101-calibrate --port /dev/ttyACM0 limits \
  --arm-calibration "$MANIPULATION_SITE/so101_calibration.json" \
  --gripper-calibration "$MANIPULATION_SITE/so101_gripper_calibration.json"   # 行程写入舵机限位
```

- **扭矩**：工具从不使能扭矩，任一舵机有扭矩时拒绝执行。
- **写入方式**：只写与当前值不同的寄存器，写前解锁、写后读回并重新上锁。
- **`center` 的影响**：执行后原有行程文件不再匹配，须重新采集。
- **部分校准**：`--part arm|gripper` 只处理机械臂或夹爪。

### 配置和控制

```bash
manipulation configure so101 --variant gripper --set port=/dev/ttyACM0 \
  --set arm_calibration=so101_calibration.json --set gripper_calibration=so101_gripper_calibration.json
manipulation monitor                                 # 只读，持续打印关节角、夹爪位置与负载
manipulation enable && manipulation hand-enable
manipulation joint-step shoulder_pan 0.05 --speed 0.2
manipulation hand-grasp                              # 闭合，报告是否夹住物体
manipulation pose                                    # 当前末端位姿
manipulation pose-move 0.20 0.00 0.15 --speed 0.2   # 末端移到基座坐标 (m)
manipulation line-move 0.20 0.05 0.15 --speed 0.2   # 末端沿直线移动
```

- **监控与示教**：`monitor` 只读，释放扭矩时可手动拖动机械臂观察读数，再用 `teach` 记录姿态。中位姿态下各关节应接近 0°。
- **抓握判断**：夹爪闭合途中受阻停下，且负载不低于 `grip_load_threshold`（默认额定输出的 10%）时，结果为 `effector_grasp_holding`；完全闭合为空夹。负载判断不是力觉测量，首次抓取新物体时须在旁观察。
- **夹爪使能与保持**：夹爪须先由 `configure` 设好扭矩保护，否则拒绝使能。夹住后 `hand-hold` 不会松开。
- **速度与超时**：`--speed 1` 对应 `max_velocity_rad_s`（默认 1 rad/s）。`pose-move` 估算的时长超过 `timeout_s` 时在运动前拒绝。
- **逆运动学**：五轴机械臂默认只求位置，加 `--rpy R P Y` 同时求姿态。不收敛时不运动；只检查关节限位，不检查碰撞。
- **直线运动**：`line-move` 按 5 mm 间隔逐点求解，任一路点不可达时整条拒绝。SO101 不支持连续路径和定时轨迹。

## 5. 手眼标定

眼在手外：相机固定，ChArUco 标定板装在法兰上，结果 `T_base_camera` 把相机坐标换算到机械臂基座。需要 OpenCV（`sudo apt install libopencv-dev python3-opencv` 后重新 `mm`）和与 Python 版本匹配的 `pyrealsense2`。

```bash
hand-eye collect cal_01 --option board=board.json   # 摆好姿态并静止后输入 CAPTURE 采样
hand-eye solve cal_01                               # 求解；残差门槛默认 5 mm / 2°
hand-eye promote cal_01                             # 确认通过门槛的结果
hand-eye export cal_01                              # 导出 extrinsic.json
```

`board.json` 按实物填写标定板参数，例如 `{"type": "charuco", "dictionary": "4x4_50", "squares_x": 5, "squares_y": 7, "square_length_m": 0.025, "marker_length_m": 0.018}`。采集工具不移动机械臂，结果保存在现场目录且不覆盖已有文件。

## 6. 在程序中使用

**Python**（`source build/envsetup.sh` 后可直接导入）：

```python
from spacemit_manipulation import DeviceClient

with DeviceClient("manipulation-device", "examples/mock_device.yaml") as arm:
    print(arm.state())
    arm.move_arm([0.2, -0.2], speed_scale=0.05)
    arm.move_effector([0.5] * 6, duration_s=0.5)
```

- **失败处理**：运动失败时抛出 `DeviceCommandError`，`result` 字段含结果状态。
- **无反馈末端**：结果为 `unverified`，须传 `allow_unverified=True` 才接受。

**运动学**（不访问设备，需要 `numpy`）：

```python
import json, os
from pathlib import Path
from spacemit_manipulation import urdf_ik
from spacemit_manipulation.urdf_fk import UrdfKinematics

root = Path(os.environ["PREFIX"]) / "share/manipulation/descriptions/robots/so101"
config = json.loads((root / "model.json").read_text())
model = UrdfKinematics(root / config["urdf"], config["base"], config["tip"], config["joints"])
result = urdf_ik.solve(model, urdf_ik.pose([0.2, 0.0, 0.15]), seed_rad=[0.0] * 5)
print(result["converged"], result["joints_rad"])   # 收敛后交给 DeviceClient.move_arm
```

**C++**：

```cmake
find_package(manipulation CONFIG REQUIRED)
target_link_libraries(app PRIVATE spacemit::manipulation_motion spacemit::manipulation_drivers)
```

```cpp
#include "manipulation/device_plan.h"
namespace m = spacemit::manipulation;

auto registry = m::MakeBuiltinDeviceRegistry();
auto plan = m::LoadDevicePlan("examples/mock_device.yaml", registry);
m::DeviceSession session(plan.definition, registry.Create(plan.driver, plan.definition, plan.options));
session.Connect();                                    // 获取设备独占，不使能、不运动
auto result = session.MoveArm({{0.2, -0.2}, 0.05});   // 阻塞至到位、失败或取消
```

**YAML 点位任务**：在设备配置的 `steps` 中写顺序动作，用 `manipulation-device run` 执行，示例见 [examples/point_sequence](examples/point_sequence/README.md)。

## 7. 新增驱动

1. 实现 `Arm`，有末端时实现 `EndEffector`，参考 `examples/bench_driver/bench_driver.cpp`。
2. 在模块的 `CMakeLists.txt` 中登记：
   ```cmake
   target_sources(manipulation_drivers PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/my_driver.cpp)
   manipulation_register_device(RegisterMyDevice)
   ```
3. 构建时用 `MANIPULATION_EXTRA_DRIVER_DIRS` 指定模块目录。
4. 在 `descriptions/robots/<型号>/devices/` 放置配置模板，供 `manipulation configure` 使用。

不需要修改公共头文件或已有驱动。驱动约定见[开发指南](docs/development.md)。

## 8. 使用限制

- **抓稳判断**：位置到位不代表实物已抓稳。只有 SO101 夹爪按负载判断是否夹住，没有力觉或触觉测量。
- **软件停止**：Ctrl+C 请求软件保持，不是急停，现场以急停为准。
- **设备独占**：只约束本机使用同一锁机制的程序，不约束厂商 Web 或其他主机。
- **无反馈末端**：开环执行；取消或中断后结果为 `stop_unknown`，须现场检查后重新连接。
- **碰撞检测**：不提供；`pose-move`、`line-move` 只检查关节限位。
- **机械臂数量**：只支持单臂。

## 目录结构

```text
include/manipulation/   C++ 公共接口
core/                   设备会话：独占、执行、到位判定、故障锁定
runtime/                配置解析与点位任务
drivers/                Mock、Nero、Revo2、SO101 及夹爪驱动
capabilities/           运动、诊断、运动学、标定及其命令行
bindings/python/        Python 接口
descriptions/           机型描述：URDF、model.json、配置模板
examples/               Mock 配置、C++ 接入、扩展驱动
tools/  tests/  docs/   辅助脚本、测试、参考文档
```

## 参考文档

- [接口参考](docs/reference.md)：配置字段、命令参数、返回值和 API。
- [开发指南](docs/development.md)：驱动约定、能力与工具扩展、测试。
- SDK 应用 `application/native/dice_arena`：基于本组件的视觉抓杯与摇骰子游戏。

## 许可

本组件代码使用 [Apache-2.0](LICENSE)。厂商 SDK pyAgxArm 使用 LGPL-3.0（见 `components/thirdparty/pyagxarm`），Nero URDF 使用 MIT（见 `descriptions/robots/nero/LICENSE`），SO101 模型使用 Apache-2.0。
