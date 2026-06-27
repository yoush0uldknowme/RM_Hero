# 上 pitch 零点标定流程

这份文档用于机械结构重新装配后，重新标定上 pitch 零点。目标是让任何 AI 或队友只看本文件，就能根据曲线截图辅助完成标定。

## 关键参数

文件：`application/Gimbal/Gimbal.h`

```c
#define UP_PITCH_OFFSET_POSITION
#define ANGLE_TO_POSITION_RATIO_UP
#define UP_PITCH_UP_POSITION_MAX
#define UP_PITCH_UP_POSITION_MIN
```

零点标定时建议先关闭角度修正：

```c
#define UP_PITCH_ANGLE_FIX_ENABLE 0
```

不要靠 PI、重力补偿或扭矩补偿掩盖零点错误。

## 当前基准

当前机械结构调准 0 点后的参考值：

```c
#define ANGLE_TO_POSITION_RATIO_UP 23.40f
#define UP_PITCH_OFFSET_POSITION 5758
```

重新装配后，最常需要重新标定的是 `UP_PITCH_OFFSET_POSITION`。

## 单点标定

1. 进入 `GIMBAL_UP`。
2. 保持 `gimbal.up_pitch.absolute_angle_set = 0`。
3. 等 3 秒以上，记录稳定后的 `gimbal.up_pitch.absolute_angle_get`。
4. 用下面公式计算新 offset：

```text
offset_new = offset_old - get_error * 25
```

其中：

```text
get_error = absolute_angle_get - absolute_angle_set
```

经验方向：

- `get` 偏正：减小 `UP_PITCH_OFFSET_POSITION`
- `get` 偏负：增大 `UP_PITCH_OFFSET_POSITION`

误差小于 1 度时，每次建议只改 `5~20` 个编码值。

## 两点插值

如果误差较大，记录两组：

```text
offset_a, get_a
offset_b, get_b
```

要求两次都是 `set = 0` 的稳定值。计算：

```text
offset_zero = offset_a + (0 - get_a) * (offset_b - offset_a) / (get_b - get_a)
```

历史示例：

```text
offset_a = 5525, get_a = -8.60
offset_b = 5820, get_b = +2.51
offset_zero ≈ 5753
```

后续微调到 `5758` 后，0 点约在 `±0.2°` 内。

## 大角度限位

如果 `set` 能到大角度，但 `get` 上不去，检查软件位置上限：

```text
target_position = UP_PITCH_OFFSET_POSITION + set_angle * ANGLE_TO_POSITION_RATIO_UP
```

例如：

```text
5758 + 45 * 23.40 = 6811
```

`UP_PITCH_UP_POSITION_MAX` 必须大于这个值并留裕量，否则会出现 `set` 有角度但 `get` 被卡住。

## 不要混淆

零点标定只解决 `set = 0` 静止时的固定偏差。

以下问题不要通过改零点解决：

- 快速运动曲线滞后
- 大角度突变抖动
- 上行停住有误差、下行回来又消失
- 模式切换中 pitch 被带偏
- 某些角度高频振动

这些应分别检查输入滤波、动态前馈、机械滞回、切换联动规划、达妙 KP/KD 和软件限位。
