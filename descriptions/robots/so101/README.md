# SO101 设备模型

提供 SO101 五轴机械臂的关节链和设备模板，支持原配单通道夹爪。

| 文件 | 用途 |
| --- | --- |
| `model.json` | 关节顺序、基座和末端坐标系 |
| `so101.urdf` | 无外部网格依赖的简化模型 |
| `devices/arm.yaml` | 机械臂配置模板 |
| `devices/gripper.yaml` | 机械臂与夹爪配置模板 |

## 关节与单位

- 关节顺序为 `shoulder_pan`、`shoulder_lift`、`elbow_flex`、`wrist_flex`、`wrist_roll`，对应舵机 ID 1–5；夹爪舵机 ID 为 6。
- 运动学链从 `base_link` 到 `gripper_frame_link`，不包含夹爪开合关节。
- 关节角按 `q = ticks × 2π / 4095 − π` 转换，单位为 rad。
- 夹爪位置单独归一化：`0` 为闭合，`1` 为张开。

## 中位与控制范围

- 模型中位须与实际安装一致。`so101-calibrate center` 把中位姿态设为编码器中位 2047（0 rad）。
- 校准文件中的 `homing_offset` 必须与舵机一致。
- 驱动取配置限位、校准行程和舵机限位的交集作为控制范围：
  - 配置模板中前四个关节的限位为 ±π，实际范围由校准行程决定；
  - `wrist_roll` 可整圈转动，校准行程为整圈，由模型限位约束。

## 来源与许可

模型源自 SO-ARM100 项目的 `so101_new_calib.urdf`，网格替换为简单几何体并保留运动学参数，按 Apache-2.0 发布。
来源与版权见 `so101.urdf`、`NOTICE` 和 `LICENSE`。
