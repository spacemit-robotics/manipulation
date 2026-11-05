# 配置式点位任务

用一份配置定义设备和点位，由 `manipulation-device` 按顺序执行。
示例使用 Mock 驱动（五轴机械臂加单通道夹爪），不连接硬件。

## 运行

在 SDK 根目录构建组件并运行：

```bash
source build/envsetup.sh
bash components/thirdparty/pyagxarm/tools/setup_sdk.sh
cd components/control/manipulation
mm --with-deps
manipulation-device run examples/point_sequence/point_sequence.yaml
```

## 修改点位

编辑 `point_sequence.yaml` 的 `steps`：

- `joint_point`：`target_rad` 按 `joints` 顺序给出各关节目标（rad），`speed_scale` 为控制器倍率 `(0,1]`。
- `effector_position`：`positions` 按 `channels` 顺序给出归一化位置 `[0,1]`（`0` 为闭合，`1` 为张开），`duration_s` 为执行时长。

目标须满足关节限位，数组长度须与设备的关节数或通道数一致。

接入真机时，在 `manipulation configure` 生成的设备配置中按实际关节和末端执行器添加 `steps`，再用 `manipulation-device run` 执行。
示例中的关节限位仅用于 Mock，不能用于真机。
