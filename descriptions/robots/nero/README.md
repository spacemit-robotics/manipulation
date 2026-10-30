# Nero V120 设备描述

关节顺序为 `joint1` 至 `joint7`，位置单位 rad；不包含 Revo2 或夹爪反馈。
点位任务按七轴顺序提供目标。

关节限位只有一个来源：厂商 SDK `components/thirdparty/pyagxarm/pyAgxArm/api/constants.py` 的 Nero 预设
（版本见该组件 `UPSTREAM_COMMIT`）。设备模板与 URDF 的 `<limit>` 均使用该预设原值，测试检查三者一致。
该预设在《NERO 用户手册》关节运动范围内：J1–J6 各收窄 2°，J7 为 ±90°（手册 ±97°）。
软件限位并非碰撞检测，不能替代现场工作空间检查。

`nero_description.urdf` 与 `LICENSE` 来自 wangannyi/dice_demo 工作区
`44708ab5ae8e928222c0c536a2e89d19257540e7` 的
`nero_revo2_control/models/hand_geometry/nero/urdf/nero_description.urdf`，
许可为 MIT，版权归 aalicecc；仅 `<limit>` 由原来截断到两位小数的弧度值改为上述 SDK 预设。
模型仅用于离线关节链计算，未复制可视化网格。
`model.json` 明确 `base_link → link7` 和七轴顺序。URDF 中的网格引用在本包中不解析。
FK 不证明自碰撞、持物间隙或实物 TCP。现场 HOME、TCP、外参不作为通用模型发布。
