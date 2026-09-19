# `motor_sensorless.c` 代码讲解

本文对应当前 `sensorless` 工作树中的实现：

- 实现文件：`user/src/motor_sensorless.c`
- 公开接口：`user/inc/motor_sensorless.h`
- 20 kHz 调用点：`user/src/motor_current_loop.c`
- 启停和电机参数配置：`user/src/motor_system.c`
- 遥测出口：`user/src/pc_protocol.c`

> 重要结论：`sensorless` 配置在 Speed 模式可选两种启动方式：**AS5600 有感启动**，或**开环强拖到交接转速后切换无感**。两种方式都会并行运行观测器；使用估算角时一旦失效，下位机立即关闭 PWM 并停机。

## 1. 整体数据流

```text
                         ┌── θAS5600/θ估算 ── Park ── 电流 PI ── 逆 Park ── SVPWM ── 电机
ADC ── 相电流 ── Clarke ─┤
                         └── iαβ ───────────────────────────────┐
                                                               ↓
上一拍 SVPWM 电压指令 uαβ[k-1] ───────────────────────────→ Observer
                                                               ↓
                                                        eαβ（反电动势）
                                                               ↓
                                                          低通滤波
                                                               ↓
                                                             PLL
                                                               ↓
                                                  θ估算、速度、锁定状态
                                                               ↓
                                                             遥测
```

在 `Motor_CurrentLoop_Run()` 中，执行顺序是：

1. ADC 原始值换算为相电流；
2. 从编码器得到机械角，生成当前有感电角度；
3. Clarke 变换得到 `i_alpha/i_beta`；
4. 调用 `Motor_Sensorless_Update()` 更新并行观测器；
5. 根据当前角度源使用 AS5600 角或有效的估算角做 Park、电流 PI、逆 Park；
6. `SVPWM_SetVoltage()` 生成本拍的新 PWM。

因此传给观测器的 `g_svpwm.v_alpha/v_beta` 是**上一拍保存的 SVPWM 电压指令**，而不是电机端实测电压。它与本拍采到的电流组合起来近似估算反电动势。

## 2. 模块解决什么问题

PMSM 在 αβ 静止坐标系下可以近似写成：

```text
uα = R·iα + L·diα/dt + eα
uβ = R·iβ + L·diβ/dt + eβ
```

移项即可估算反电动势：

```text
eα = uα - R·iα - L·diα/dt
eβ = uβ - R·iβ - L·diβ/dt
```

转子转动时，永磁体产生的反电动势矢量 `eαβ` 会随电角度旋转。代码先估算并滤波 `eαβ`，再用 PLL 追踪它的方向，得到：

- 估算电角度 `electrical_angle_rad`；
- 估算电角速度 `electrical_speed_rad_s`；
- 换算后的机械转速 `mechanical_speed_rpm`；
- PLL 是否已经锁定。

该方法依赖足够大的反电动势，因此 PLL 本身不适合零速和很低速启动。强拖模式会先用固定电角度定向，再线性提升开环电角速度，直到达交接转速且 PLL 有效。

## 3. 公开配置和输出

### 3.1 `MotorSensorlessConfig`

| 字段 | 含义 | 当前默认值 |
|---|---|---:|
| `sample_time_sec` | 电流环采样周期 | 50 µs |
| `resistance_ohm` | 电机相电阻模型参数 | 1.0 Ω |
| `inductance_h` | 电机相电感模型参数 | 1.2 mH |
| `pole_pairs` | 极对数，用于电角速度换机械转速 | 1 |
| `bemf_filter_alpha` | BEMF 一阶低通系数 | 0.05 |
| `minimum_bemf_volts` | 允许 PLL 跟踪的最低 BEMF 幅值 | 0.20 V |
| `pll_kp` | PLL 比例增益 | 250 |
| `pll_ki` | PLL 积分增益 | 12000 |
| `maximum_electrical_speed_rad_s` | 电角速度绝对限幅 | 8000 rad/s |
| `lock_phase_error` | 锁定允许的相位误差门槛 | 0.30 |
| `lock_updates` | 连续合格多少次才锁定 | 100 |
| `loss_updates` | 连续低 BEMF 多少次才失锁 | 250 |
| `pll_divider` | PLL 相对 20 kHz 电流环的分频 | 4 |

`Motor_Sensorless_ConfigureMotor()` 会把辨识所得的 R、L 和极对数写入配置。调用 `Motor_Sensorless_Configure()` 或 `ConfigureMotor()` 后，运行状态会复位，PLL 需要重新捕获和锁定。

### 3.2 `MotorSensorlessOutput`

| 字段 | 含义 | 更新频率 |
|---|---|---|
| `bemf_alpha_volts`、`bemf_beta_volts` | 滤波后的 BEMF 分量 | 20 kHz，首拍除外 |
| `bemf_magnitude_volts` | BEMF 矢量幅值 | PLL 频率，默认 5 kHz |
| `electrical_angle_rad` | 估算电角度，范围 `[0, 2π)` | 默认 5 kHz |
| `electrical_speed_rad_s` | 估算电角速度 | 默认 5 kHz，有效 BEMF 时 |
| `mechanical_speed_rpm` | 估算机械转速 | 默认 5 kHz，有效 BEMF 时 |
| `pll_phase_error` | PLL 当前相位误差 | 默认 5 kHz，有效 BEMF 时 |
| `status` | 禁用、搜索、跟踪或丢失 | 事件驱动 |
| `valid` | 角度是否已经满足锁定判据 | 事件驱动 |

## 4. 内部状态 `MotorSensorlessInternal`

模块只保存一个私有实例 `g_sensorless`。字段可以分成四组：

1. **配置与输出**：`config`、`output`；
2. **观测器历史**：上一拍电流、滤波后的 BEMF；
3. **PLL 历史**：积分速度 `pll_integral_speed`；
4. **时序和状态机**：锁定计数、丢失计数、PLL 分频计数、首拍标志、初相捕获标志、使能和方向。

`Motor_Sensorless_ResetState()` 会清除所有运行历史，但保留：

- 当前配置；
- `enabled`；
- `direction`。

复位后的状态由使能标志决定：使能时为 `SEARCHING`，否则为 `DISABLED`。

## 5. 主函数骨架

`Motor_Sensorless_Update()` 只保留信号流骨架，可以按下面的顺序阅读：

```c
if (未使能) return;

电流差分并估算原始 BEMF；    // 首拍只记录电流，然后 return
对 BEMF 做低通并发布 eαβ；

if (PLL 分频未到) return;

计算 BEMF 幅值和 PLL 实际步长；
if (BEMF 太小) {
    处理低 BEMF、丢失计数和角度外推；
    return;
}

清除丢失计数；
执行 PLL 跟踪；
更新锁定状态；
```

这种结构有两个运行频率：

- BEMF 方程与低通：20 kHz；
- 幅值判断、PLL、锁定/丢失：`20 kHz / pll_divider`，默认 5 kHz。

## 6. 各步骤详细解释

### 6.1 `Motor_Sensorless_EstimateBemf()`：电流差分与 BEMF 估算

首拍没有 `i[k-1]`，无法求导，所以只保存当前电流并返回 0：

```text
last_current = current
current_initialized = 1
```

从第二拍开始使用后向差分：

```text
di/dt ≈ (i[k] - i[k-1]) / Ts
```

随后代入电压方程：

```text
e_raw = u_command[k-1] - R·i[k] - L·(i[k]-i[k-1])/Ts
```

这里有三个需要注意的误差来源：

- 电压使用的是指令值，不含死区、MOSFET 压降和母线波动造成的实际电压误差；
- 电压是上一拍，而电流是本拍，存在一个采样周期的对齐误差；
- 电流差分会显著放大 ADC 和 PWM 噪声。

### 6.2 `Motor_Sensorless_FilterBemf()`：BEMF 一阶低通

代码采用指数滑动平均：

```text
e_filtered[k] = e_filtered[k-1]
              + α · (e_raw[k] - e_filtered[k-1])
```

`α` 越小，输出越平滑，但相位滞后越大。当前 `α=0.05`、`Ts=50 µs`，对应的离散一阶低通截止频率约为：

```text
fc ≈ -ln(1-α) / (2πTs) ≈ 163 Hz
```

滤波结果每个电流环周期都写入输出，因此即使本拍没有运行 PLL，上位机仍能读到较新的 `eα/eβ`。

### 6.3 `Motor_Sensorless_PllTickDue()`：PLL 分频

每次有效 BEMF 更新后将 `pll_counter` 加一。计数达到 `pll_divider` 才执行一次 PLL，然后清零。

默认配置为：

```text
电流环：20 kHz
pll_divider：4
PLL：5 kHz
pll_dt：50 µs × 4 = 200 µs
```

首拍只初始化电流差分，不增加 PLL 分频计数。

### 6.4 BEMF 幅值判断

PLL 拍到来时计算：

```text
|e| = sqrt(eα² + eβ²)
```

如果 `|e| < minimum_bemf_volts`，BEMF 方向容易被噪声主导，代码不会用它校正 PLL，而是进入弱 BEMF 分支。

### 6.5 `Motor_Sensorless_HandleWeakBemf()`：低 BEMF 与失锁

低 BEMF 时执行：

1. 清零 `lock_counter`；
2. 增加 `loss_counter`；
3. 达到 `loss_updates` 后清除 `valid`；
4. 已经捕获过相位则进入 `LOST`，否则保持 `SEARCHING`；
5. 不用当前 BEMF 校正，只按上一次速度外推角度。

角度外推为：

```text
θ[k+1] = wrap(θ[k] + ω_est · pll_dt)
```

注意：BEMF 刚低于门槛时不会立即清除 `valid`。默认需要连续 250 个 PLL 更新，也就是约 50 ms，才正式判定丢失。这种延时可以防止短时噪声或换相扰动造成状态反复跳变。

### 6.6 `Motor_Sensorless_TrackPll()`：初相捕获与 PLL

#### 第一步：BEMF 单位化

```text
eα_unit = eα / |e|
eβ_unit = eβ / |e|
```

PLL 只追踪方向，不让 BEMF 幅值直接改变环路增益。

#### 第二步：首次有效 BEMF 的初相捕获

第一次获得足够大的 BEMF 时，不从 0 慢慢追，而是直接利用 `atan2f()` 初始化角度：

```text
θ_init = atan2(-direction · eα_unit,
                direction · eβ_unit)
```

代码采用的理想 BEMF 方向模型是：

```text
eα_pred = -direction · sin(θ_est)
eβ_pred =  direction · cos(θ_est)
```

`direction` 必须与实际旋转方向一致，否则预测矢量方向、速度限幅方向和积分方向都会错误。

#### 第三步：计算相位误差

预测单位矢量与实测单位矢量做二维叉积：

```text
phase_error = eα_pred · eβ_measured
            - eβ_pred · eα_measured
```

该值本质上是两矢量夹角差的正弦，范围约为 `[-1, 1]`；当相位差较小时，`sin(Δθ) ≈ Δθ`，才可以近似看成弧度误差。因此 `lock_phase_error=0.30` 是基于小角度近似的门槛，不是任意范围内严格等于 0.30 rad。

#### 第四步：PLL PI

积分支路生成速度的低频部分：

```text
ω_integral += Ki · phase_error · pll_dt
```

比例支路提供快速校正：

```text
ω_est = ω_integral + Kp · phase_error
```

积分速度和最终速度都会按 `direction` 限幅：

- 正转只允许 `[0, speed_limit]`；
- 反转只允许 `[-speed_limit, 0]`。

这样可以防止 PLL 在已知转向下积分到相反方向，但也意味着方向判断错误时 PLL 不可能正常跟踪。

#### 第五步：速度积分得到角度

```text
θ_est = wrap(θ_est + ω_est · pll_dt)
```

机械转速换算为：

```text
mechanical_rpm = electrical_rad_s × 60/(2π) / pole_pairs
```

`MOTOR_SENSORLESS_RPM_SCALE=9.549296586` 就是 `60/(2π)`。

### 6.7 `Motor_Sensorless_UpdateLockState()`：锁定判据

每个有效 BEMF 的 PLL 拍检查：

```text
abs(phase_error) <= lock_phase_error
```

- 合格：`lock_counter++`；
- 不合格：计数清零，`valid=0`，状态回到 `SEARCHING`；
- 连续达到 `lock_updates`：`valid=1`，状态进入 `TRACKING`。

默认 `lock_updates=100`、PLL 为 5 kHz，所以理论上的最短连续确认时间约为 20 ms。这里不包括首次 BEMF 建立和 PLL 收敛所需时间。

## 7. 状态机

| 状态 | 进入条件 | `valid` 的典型值 | 含义 |
|---|---|---:|---|
| `DISABLED` | 模块未使能或被关闭 | 0 | `Update()` 不做任何计算 |
| `SEARCHING` | 已使能但还没连续满足锁定条件；或相位误差超限 | 0 | 正在捕获/重新捕获 |
| `TRACKING` | 相位误差连续合格达到 `lock_updates` | 1 | 当前估算角可用于比较 |
| `LOST` | 曾捕获相位，之后 BEMF 连续过低达到 `loss_updates` | 0 | 已失去可靠观测条件 |

简化状态转移如下：

```text
Enable
  ↓
SEARCHING ── 连续相位误差合格 ──→ TRACKING
    ↑                                │
    └──── 相位误差超限 ──────────────┘
                                     │
                                     └── BEMF 连续过低 ──→ LOST
                                                               │
                                                               └── BEMF 恢复后重新满足误差 ─→ TRACKING

Disable ───────────────────────────────────────────────────────→ DISABLED
```

从 `LOST` 恢复时，`phase_initialized` 没有清零，所以不会重新执行 `atan2f()` 初相捕获，而是从弱 BEMF 期间外推的角度继续由 PLL 拉回。只有 Reset、重新配置或重新使能才会清除该标志。

## 8. 方向是怎样得到的

`Motor_Sensorless_SetDirection()` 只接受 `+1` 或 `-1`，传入 0 不改变现有方向。

系统 1 ms 任务根据有感机械转速更新方向：

```text
current_rpm >  20 rpm  → direction = +1
current_rpm < -20 rpm  → direction = -1
速度在 ±20 rpm 内      → 保持上一次方向
```

这样做可避免接近零速时方向在噪声作用下来回跳变。但在首次反向启动、低速换向或方向尚未正确建立时，PLL 可能暂时无法锁定。

## 9. 初始化、启停和当前工程模式

`Motor_Sensorless_Init()` 默认把模块设为禁用，因为头文件中的 `MOTOR_SENSORLESS_DEFAULT_ENABLE` 为 0。

在 `sensorless` 构建配置中，系统进入正常有感运行并使能电流环时会：

1. 用电机辨识结果配置 R、L 和极对数；
2. 使能 SVPWM 与有感电流环；
3. 使能无感观测器并行运行，默认角度源仍为 AS5600。

安全停机时观测器也会被关闭。AS5600 启动时，上位机可在 Speed + HOST、电机运行且 `Motor_Sensorless_IsValid()` 为真时手动切换到估算角。强拖启动时，流程为 300 ms 定向、1500 ms 线性加速，到达设定转速后最多等待 2500 ms 锁定；超时直接停机。交接后如果 `valid` 变为 0，20 kHz 电流环会立即关闭 PWM，随后 1 ms 系统任务进入 `STOPPED`。

## 10. 并发和原子读取

`Motor_Sensorless_Update()` 在 20 kHz 电流环中断中更新多个浮点字段。主循环或通信任务如果逐字段读取，可能碰到“读了一半时中断又更新”的混合快照。

因此 `Motor_Sensorless_GetOutput()` 会短暂关中断，把整个输出结构复制到局部变量后再恢复中断。配置、使能和复位接口也采用同样的临界区保护。

代码先读取 `PRIMASK`，只在调用前中断本来开启的情况下重新开启，避免破坏调用者原有的中断状态。

## 11. 上位机现在看到的内容

通信层每帧获取一次 `MotorSensorlessOutput`，把以下字段送到上位机：

- 无感估算电角度；
- AS5600 参考电角度；
- 无感角 `valid`；
- AS5600 参考角是否有效。

所以上位机显示“未锁定”表示 `sensorless.valid == 0`，常见原因有：

1. 电机速度太低，BEMF 幅值低于 0.20 V；
2. R/L 模型与实际电机不匹配；
3. 电压指令与实际端电压误差较大；
4. 方向尚未正确确定；
5. PLL 参数不合适或相位误差持续超过门槛；
6. 电流采样差分噪声过大；
7. 刚开始运行，还没有累计满 100 次锁定确认。

## 12. 参数如何影响波形

### `resistance_ohm` 和 `inductance_h`

它们直接参与 BEMF 扣除。R 偏差主要影响负载电流较大时的估算；L 偏差和电流差分噪声会直接形成高频 BEMF 误差。应优先保证辨识结果合理。

### `bemf_filter_alpha`

- 调小：BEMF 更平滑，但相位滞后加大；
- 调大：响应更快，但噪声更大，PLL 更容易抖动。

调整该参数后通常需要重新检查 PLL 增益，因为滤波器相位延迟属于 PLL 环路的一部分。

### `minimum_bemf_volts`

- 调低：更容易在低速开始跟踪，但更可能把噪声当成 BEMF；
- 调高：锁定更可靠，但最低可观测速度升高。

### `pll_kp`、`pll_ki`

- `Kp` 增大：相位修正更快，速度输出也更容易受噪声影响；
- `Ki` 增大：稳态速度跟随更快，但过大可能振荡或积分顶到限幅；
- 两者过小：估算角会明显滞后，变速时更难锁定。

### `lock_phase_error`、`lock_updates`

它们只决定“何时宣布锁定”，不会直接改变 PLL 动力学。门槛放宽或次数减少会更快显示锁定，但不代表估算质量真的提高。

### `loss_updates`

决定低 BEMF 持续多久才正式丢失。过小容易闪断，过大会在观测条件已经消失后较长时间仍保留 `valid`。

## 13. 阅读和调试建议

建议按以下顺序判断问题，而不是一开始就调 PLL：

1. 确认 AS5600 参考角和方向正确；
2. 确认辨识得到的 R、L、极对数合理；
3. 观察 `eα/eβ` 是否近似正交旋转、幅值是否随速度上升；
4. 检查 BEMF 幅值能否稳定越过门槛；
5. 检查 `direction` 是否与实际转向一致；
6. 再观察 `pll_phase_error` 和估算速度；
7. 最后调整滤波、PLL 增益及锁定门槛。

对比角度时要使用环形角差，而不能直接相减：

```text
angle_error = wrap_to_pi(theta_est - theta_reference)
```

否则一个角在 `2π` 附近、另一个角刚跨到 0 时，会显示接近 `2π` 的假误差。

## 14. 当前实现的边界

当前代码已经具备 BEMF 估算、PLL、锁定状态、手动角度源切换和基础强拖启动。尚缺少或尚未接入的部分包括：

- 开环角与估算角之间的相位平滑过渡（当前锁定后直接切换）；
- 失锁后的自动降级与重新捕获策略（当前是直接停机）；
- 逆变器死区、器件压降和母线电压变化补偿；
- 更严格的电压/电流采样时序对齐；
- 基于实机数据系统整定的 R、L、滤波和 PLL 参数；
- 无感角长时间接管 Park 变换所需的完整实机验证。

因此，当前最准确的定位是：**Speed 模式可选 AS5600 启动或强拖启动；观测器锁定后让估算角接管 Park，无感失效则直接停机。**
