# MuJoCo 机械臂仿真模块

本模块属于 [manipulation 仓库](https://github.com/spacemit-robotics/manipulation)，提供机械臂仿真使用的物理世界、状态快照、RGB-D 相机与窗口。SDK 路径为 `components/control/manipulation/simulation/mujoco/`。

UR5e 机械臂与 Robotiq 2F-85 夹爪通过 `mujoco_ur5e`、`mujoco_robotiq_2f85` 驱动接入。运行设备示例见[机械臂组件 README](../../README.md#ur5e--mujoco-物理仿真)，本页说明底层接口与构建方式。

## 1. 模块边界

| 目录（相对 manipulation） | 职责 |
| --- | --- |
| `simulation/mujoco/` | 物理世界、状态快照、模拟相机与窗口 |
| `drivers/simulation/mujoco/` | 机械臂与末端驱动、共享连接与远程通信 |
| `capabilities/simulation/` | 常驻仿真服务、窗口与相机命令行工具 |

| 接口 | 用途 | 运行方式 |
| --- | --- | --- |
| `mujoco_sim::World` | 读取关节、写入执行器控制并推进物理世界 | 无窗口，由调用方管理步进 |
| `mujoco_sim::WorldSnapshot` | 保存物理世界的独立状态快照 | 供规划与渲染读取 |
| `mujoco_sim::Renderer` / `Viewer` | 模拟 RGB-D 相机 / 第三视角窗口 | 读取快照，不推进物理 |

`World` 直接加载 MJCF，不假定关节排列或浮动基座。抓取策略、轨迹规划和任务流程由应用负责。

## 2. 构建与链接

支持 Linux x86_64 PC 与 riscv64 K3，使用 CMake 3.16+ 和 C++17 编译器。

机械臂驱动和服务随组件构建，由 `MANIPULATION_ENABLE_MUJOCO` 和 `MANIPULATION_ENABLE_SIMULATION_RENDERER` 选择，步骤见[组件安装与仿真示例](../../README.md#ur5e--mujoco-物理仿真)。

只使用底层物理世界时，在 SDK 根目录执行：

```bash
cmake -S components/control/manipulation/simulation/mujoco -B output/manipulation-mujoco \
    -DCMAKE_INSTALL_PREFIX="$PWD/output/staging"
cmake --build output/manipulation-mujoco -j2
cmake --install output/manipulation-mujoco
```

默认只构建物理世界，不需要 GLFW、X11 或 OpenGL。MuJoCo 库按[依赖查找规则](#5-依赖与许可)获取。

应用使用安装后的 CMake 包：

```cmake
find_package(mujoco_world CONFIG REQUIRED)
target_link_libraries(app PRIVATE mujoco_sim::world)
```

## 3. 物理世界与快照

公开接口位于 `include/mujoco_world.h`：

| 方法 | 作用 |
| --- | --- |
| `World(xml_path)` | 加载独立物理世界，不修改进程工作目录 |
| `Joint(name)` / `Actuator(name)` | 按模型名称读取标量关节状态或执行器参数 |
| `SetControls(controls)` | 校验整批控制值后写入，未列出的执行器保持原控制值 |
| `Step(count)` | 推进指定步数；默认一步 |
| `Reset(keyframe)` | 显式恢复默认状态或指定关键帧 |
| `Time()` / `Timestep()` / `SampleId()` | 读取仿真时间、时间步和样本编号 |
| `Snapshot()` | 复制独立快照，供规划、相机和窗口读取 |
| `TouchingBodies()` | 查询模型中的物体接触 |
| `AttachWeld()` / `ReleaseWeld()` | 显式启用 / 解除场景中已有的焊接约束，不移动物体 |

使用约束：

- **并发访问**：同一世界由单个调用方访问；多线程使用时须在外部串行化。
- **异常状态**：无效物理状态会锁定世界，直到显式重置；不会把引擎自动复位后的状态当作正常反馈。

## 4. 模拟相机与窗口

在 SDK 根目录安装图形依赖并启用可选渲染库：

```bash
sudo apt install -y git ca-certificates libglfw3-dev libgl1-mesa-dev libx11-dev
cmake -S components/control/manipulation/simulation/mujoco -B output/manipulation-mujoco \
    -DMUJOCO_BUILD_RENDERER=ON \
    -DCMAKE_INSTALL_PREFIX="$PWD/output/staging"
cmake --build output/manipulation-mujoco -j2
cmake --install output/manipulation-mujoco
```

应用链接 `mujoco_sim::render`，在主线程创建、调用和销毁 `Renderer`、`Viewer`。公开接口位于 `include/mujoco_render.h`。

- **相机输出**：按 MJCF 相机名称选择视角，输出对齐的 RGB8、米制浮点深度、相机内外参和样本编号。深度为光学轴 Z 距离，无效值为 `0`。
- **显示服务**：相机与窗口都需要 X11，不提供无 X11 的 EGL 或 OSMesa 离屏入口。
- **世界访问**：绘图使用独立快照，不持有物理步进锁；只读观察不重置世界或取得设备控制权。

| 平台 | 默认渲染后端 |
| --- | --- |
| PC | GLFW/OpenGL |
| K3 | X11/gl4es |

通过 `MUJOCO_RENDER_BACKEND=GLFW|X11` 和 `MUJOCO_RENDER_USE_GL4ES=ON|OFF` 选择后端；gl4es 仅支持 X11。

渲染目标导出 `MUJOCO_RENDER_USES_GL4ES`。使用 gl4es 的程序在创建上下文前设置 `LIBGL_GL=21`、`LIBGL_ES=2`，不覆盖用户已有设置。

gl4es 运行库安装在库目录下的 `mujoco_world/gl4es/`，由组件自身的相对运行路径加载。SDK 的 `LD_LIBRARY_PATH` 只需包含公共库目录；gl4es 私有目录由组件自动定位，使其他图形程序继续使用系统 OpenGL。

| 使用场景 | 入口与说明 |
| --- | --- |
| 机械臂窗口与远程相机 | `manipulation-simulation`、`manipulation-simulation-camera`，命令见[机械臂组件 README](../../README.md#ur5e--mujoco-物理仿真) |
| 完整抓取示例 | [perceptive_grasp 仿真运行](../../../../../application/ros2/linksee/perceptive_grasp/README.md#7-仿真运行)；支持 PC 或 K3 本机运行，也支持 PC 运行场景、K3 远程执行抓取 |

通用设备服务使用端口 `9100`，抓取应用服务使用端口 `9090`。两者协议不同，不可互换。

## 5. 依赖与许可

CMake 使用 MuJoCo 3.4.0，并按架构选择预编译包来源：

| 架构 | 来源 |
| --- | --- |
| x86_64 | MuJoCo 官方 GitHub release |
| riscv64（K3） | SpaceMIT bianbu26：`archive.spacemit.com/ros2/prebuilt_libs/bianbu26/opt/ext/mujoco/` |

查找顺序如下，命中即用：

1. CMake 参数 `-DMUJOCO_DIR=...`。
2. 环境变量 `MUJOCO_DIR`。
3. `/usr/local`、`/opt/mujoco`。
4. 缓存路径 `~/.cache/thirdparty/mujoco/mujoco-3.4.0/`。
5. 从对应架构的预编译包来源获取并解压到缓存。

离线构建时设 `SROBOTIS_THIRDPARTY_FETCH_OFF=ON` 禁用下载，并用 `MUJOCO_DIR` 指向已有依赖。

K3 渲染依赖的 gl4es 由本目录 `cmake/FindGL4ES.cmake` 获取固定版本、应用补丁并构建。已有依赖可通过 `-DGL4ES_ROOT=...` 指定。

源码许可见 [LICENSE](LICENSE)，依赖说明见 [NOTICE](NOTICE)。随包分发的 MuJoCo 第三方声明位于 `licenses/mujoco-3.4.0/THIRD_PARTY_NOTICES`；启用 gl4es 时同时安装其许可文件。场景模型与网格的许可随各自资源目录保存。

## 6. 测试

在上述构建目录启用测试并运行：

```bash
cmake -S components/control/manipulation/simulation/mujoco -B output/manipulation-mujoco \
    -DBUILD_TESTING=ON
cmake --build output/manipulation-mujoco -j2
ctest --test-dir output/manipulation-mujoco --output-on-failure
```

| 测试 | 范围 | 运行条件 |
| --- | --- | --- |
| `mujoco_world` | 世界步进、关节、执行器及异常状态 | 无需显示器 |
| `mujoco_snapshot` | 独立快照与场景状态 | 无需显示器 |
| `mujoco_rgbd` | RGB-D 输出与渲染线程约束 | 启用渲染库并安装 `xvfb-run` |
