# 架构图维护

[architecture.py](architecture.py) 是组件架构图的唯一图源，包含节点、连线、文字和排版。生成器仅使用 Python 标准库。

## 1. 生成与同步

在 manipulation 仓库根目录执行，`--docs-ros` 指向 docs-ros 仓库：

```bash
python3 docs/diagrams/architecture.py --docs-ros /path/to/docs-ros
```

同一份输出写入以下位置：

| 仓库 | SVG |
| --- | --- |
| manipulation | `docs/assets/architecture-layered.svg` |
| docs-ros | `zh/k3/05-机器人开发/5.2-运动控制/assets/manipulation-sdk-architecture-layered.svg` |

省略 `--docs-ros` 时只生成 SDK 图。同步只更新上述中文图，不修改英文文档。

检查两份 SVG 是否与图源一致：

```bash
python3 docs/diagrams/architecture.py --docs-ros /path/to/docs-ros --check
```

不一致时命令列出文件并返回非零状态。SVG 的标题、替代说明和可见内容均来自同一图源。

## 2. 架构依据

图展示公共调用关系和模块职责，具体驱动参数见[接口参考](../reference.md)。修改结构时核对以下代码：

| 图中关系 | 代码入口 |
| --- | --- |
| Python 客户端、本机 JSONL 与设备进程 | [client.py](../../bindings/python/spacemit_manipulation/client.py)、[device_cli.cpp](../../capabilities/motion/cli/device_cli.cpp) |
| 命令行的交互控制、任务和检查路径 | [operator_cli.py](../../capabilities/diagnostics/cli/operator_cli.py) |
| 配置解析、角色装配与顺序能力调度 | [device_plan.cpp](../../runtime/device_plan.cpp)、[device.h](../../include/manipulation/device.h) |
| 会话控制与设备接口 | [device.h](../../include/manipulation/device.h) |
| MuJoCo 本地与 TCP 后端 | [device_driver.cpp](../../drivers/simulation/mujoco/device_driver.cpp)、[server.cpp](../../capabilities/simulation/server.cpp) |
| 物理世界、快照、窗口与 RGB-D | [mujoco_world.h](../../simulation/mujoco/include/mujoco_world.h)、[renderer_bridge.cpp](../../capabilities/simulation/renderer_bridge.cpp) |
| 公共计算能力 | [kinematics.h](../../include/manipulation/kinematics.h)、[calibration.h](../../include/manipulation/calibration.h) |

SDK README、开发指南和 docs-ros 中文页面引用同一张图。修改后同步配套说明，并检查原始尺寸和文档显示尺寸下的文字、连线与边界。
