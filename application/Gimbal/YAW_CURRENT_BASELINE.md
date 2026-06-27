# YAW 当前基线记录

## 1. 文件目的

这份文件只做一件事：

**记录当前这台车的大 YAW，眼下最好用、最值得作为继续调试起点的一套状态。**

它不是完整原理文档，完整原理看：

- [YAW_TUNING_GUIDE.md](E:/HJ/2026/HERO/2026_turtle_Hero/2025_Hero/application/Gimbal/YAW_TUNING_GUIDE.md)

这份文件偏实战，给后续接手者一个“先从哪一版开始”的答案。

---

## 2. 当前控制框架

当前 YAW 采用的是：

1. `absolute_angle_set` 作为唯一目标角命令源
2. `Build_Cmd()` 生成平滑的 `vel_ff`
3. `Servo()` 根据角度误差、速度误差、摩擦补偿、刹车等合成 `out_torque`
4. 达妙 MIT 模式下主要用扭矩口驱动 YAW

关键原则：

- **控制器不能改写 `absolute_angle_set`**
- 速度前馈只作为辅助，不能直接拿 `d(set)/dt` 生硬当主目标速度

---

## 3. 当前建议作为起点的参数

以下是当前建议作为“继续调参起点”的宏参数。

如果后续某一轮把系统调坏了，建议先回到这里。

```c
#define GIMBAL_YAW_SERVO_DT                    0.001f
#define GIMBAL_YAW_PLAN_KP                     4.8f
#define GIMBAL_YAW_PLAN_SET_FF                 0.23f
#define GIMBAL_YAW_PLAN_SMALL_ERR              6.0f
#define GIMBAL_YAW_PLAN_SMALL_FF               0.08f
#define GIMBAL_YAW_PLAN_REVERSE_FF             0.14f
#define GIMBAL_YAW_PLAN_REVERSE_ACCEL_SCALE    0.75f
#define GIMBAL_YAW_PLAN_VEL_LPF                0.25f
#define GIMBAL_YAW_PLAN_MAX_VEL                190.0f
#define GIMBAL_YAW_PLAN_MAX_ACCEL              36.0f

#define GIMBAL_YAW_IMP_KP                      0.82f
#define GIMBAL_YAW_IMP_KD                      0.048f
#define GIMBAL_YAW_IMP_KA                      0.000f
#define GIMBAL_YAW_IMP_KF                      0.12f
#define GIMBAL_YAW_IMP_FRICTION_ERR            2.0f
#define GIMBAL_YAW_IMP_KV_ASSIST               0.006f
#define GIMBAL_YAW_IMP_ASSIST_ERR              10.0f
#define GIMBAL_YAW_IMP_MAX_ASSIST_TORQUE       0.65f
#define GIMBAL_YAW_IMP_KB                      0.0155f
#define GIMBAL_YAW_IMP_BRAKE_ERR               3.0f
#define GIMBAL_YAW_IMP_MAX_BRAKE_TORQUE        1.20f
#define GIMBAL_YAW_IMP_FRICTION_MIN_VEL        6.0f
#define GIMBAL_YAW_IMP_MAX_TORQUE              8.0f
#define GIMBAL_YAW_IMP_MAX_VEL_ERR             110.0f
#define GIMBAL_YAW_IMP_MAX_ACC_FF              25.0f
#define GIMBAL_YAW_IMP_DM_KD                   0.02f
#define GIMBAL_YAW_IMP_GYRO_LPF                0.08f
#define GIMBAL_YAW_IMP_DOWN_BIAS               1.00f
#define GIMBAL_YAW_IMP_UP_BIAS                 1.00f

#define GIMBAL_YAW_RC_DEAD_BAND                25
#define GIMBAL_YAW_MOUSE_DEAD_BAND             4.0f

#define GIMBAL_YAW_LOAD_ADAPT_MIN              0.60f
#define GIMBAL_YAW_LOAD_ADAPT_MAX              1.80f
#define GIMBAL_YAW_LOAD_ADAPT_INC              0.0010f
#define GIMBAL_YAW_LOAD_ADAPT_DEC              0.995f
#define GIMBAL_YAW_LOAD_ADAPT_TRACK_ERR        6.0f
#define GIMBAL_YAW_LOAD_ADAPT_SPEED_ERR        18.0f
#define GIMBAL_YAW_LOAD_ADAPT_OSC_GYRO         45.0f
#define GIMBAL_YAW_LOAD_ADAPT_PERIOD           5U
```

---

## 4. 当前这版的总体表现

和最早阶段相比，这版已经明显解决了这些问题：

1. 起停时“猛冲一下再顿一下”的问题被大幅缓解
2. 整体振荡问题已经基本压住
3. 快速反向时的严重爆冲比最差阶段好很多
4. `set` 和 `get` 的整体跟随已经进入可用区间

---

## 5. 当前仍然存在的问题

当前这版还不是最终完美版，主要还有这些尾巴：

1. 小幅度动作时仍可能有一点点超调
2. 到平台后仍可能有一点点小回弹
3. 最终 `set` 和 `get` 会留一丁点差距
4. 快速变化曲线还可以再更顺一点

也就是说：

**当前这版已经适合作为稳定基线，但还没有达到“完全丝滑”的最终状态。**

---

## 6. 明确不建议回去的做法

以下做法已经验证不适合作为主路线：

1. 让控制器偷偷改 `absolute_angle_set`
2. 把 `d(set)/dt` 直接当成主目标速度
3. 一次同时暴力推高很多主增益
4. 只为了压超调而把整体阻尼加得很重

这些做法短时间看像有效，后面都会把问题转移或放大。

---

## 7. 当前最值得信的经验结论

当前这台大 YAW 最有效的一条经验不是某个具体 K 值，而是这条结构：

```text
误差主导
+ 少量 set 前馈
+ 小误差弱前馈
+ 反向柔和处理
+ 末端刹车
```

只要后续继续沿着这条框架微调，通常都不会走太偏。

---

## 8. 如果后续调崩了，怎么快速回退

如果后面某一轮又开始出现：

- 明显振荡
- 起停猛冲
- 咔咔响
- 快速反向严重超调

建议回退顺序：

1. 先把 `GIMBAL_YAW_PLAN_SET_FF / SMALL_FF / REVERSE_FF / REVERSE_ACCEL_SCALE`
   回退到本文件这一版
2. 再把 `IMP_KP / IMP_KD / IMP_KB` 回退到本文件这一版
3. 如果还不行，再核对 `Build_Cmd()` 是否仍是“误差主导 + 少量前馈”的逻辑

---

## 9. 给后续接手者的一句话

如果你刚接手这台车：

**先从这版基线开始，不要从头乱试。**

这版不是最完美的，但它是目前已经验证过“整体方向正确、不会明显炸”的一版。

