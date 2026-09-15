# MotorSystem 可读性重构执行计划

本文是执行规范，不是再次征求架构方案。按 Stage 顺序执行，每个 Stage 单独编译、验证和记录结果；不得同时实施尚未验收的后续 Stage。

## 0. 范围、路径和停止规则

- 项目根目录：`D:/WorkDocument/g431_foc`。下文文件路径均相对此目录。
- 审计基线日期：2026-09-14。基线 `user/src/motor_system.c` 为 786 行；行数仅用于识别版本，定位修改必须用函数名和语句内容。
- 工程为 C，没有需要重构的类。保留已有结构体、枚举和公开接口，除 Stage 1 指定的两个无实现声明外不删除公开接口。
- 目标：减少混合职责、重复复位和调试副本，使控制执行顺序直观。没有行数指标。
- 当前工作区有用户及此前任务的未提交修改。以执行开始时的工作区内容为基线，不能用 Git HEAD 覆盖它们，不能 reset、clean、checkout 整个文件回退。
- 本计划创建阶段不修改源码。实际重构由后续执行任务开展。

### 0.1 遇到代码不一致时的固定处理

1. 仅空白、行号或注释不同：按函数和语义定位，允许继续。
2. 待删除符号已有有效调用、待移动函数行为改变、指定连续语句不再相邻、出现新模式/状态/中断访问者：停止该 Stage，记录实际代码、计划预期和受影响调用方；不得自行设计替代实现。
3. 前一 Stage 明确产生的差异不算异常，例如调试函数已经迁出、变量已经改成结构体字段。
4. 编译器报错时，只修复本 Stage 引入的遗漏声明、include 或构建登记。需要改控制公式、类型布局、硬件配置才能通过时必须停止。
5. 验证不通过：保留日志，只撤回本 Stage 自己的修改；不能覆盖用户改动。未定位原因前不得继续依赖它的 Stage。
6. 缺少编译工具或硬件：明确记为“未验证”，不能记为通过。先完成允许的静态检查；不得自动安装工具、改工具链、烧录或驱动电机。
7. 不因发现顺手可修的问题扩大范围。新问题单独记录，本文未指定的算法或行为修复不执行。

### 0.2 最终固定结构

| 文件 | 最终职责 |
|---|---|
| `user/src/motor_system.c` | 系统命令检查、原有状态决策、模式互斥、1 ms 调度、公共输入处理、调用外环/前馈、发布电流目标 |
| `user/inc/motor_system.h` | 保留系统类型、现有有效公开 API；包含共享配置头以保持源代码兼容 |
| `user/inc/motor_config.h`（新增） | 原有共享母线电压、力矩配置和测速调度常量；不增加运行时配置服务 |
| `user/src/motor_debug.c`（新增） | 原 OLED 显示及 VOFA 20 通道组帧，仍在前台运行 |
| `user/inc/motor_debug.h`（新增） | 调试显示声明、唯一必要的系统调试数据读取接口 |
| `user/src/motor_speed_loop.c` | 原速度估计和速度环，加上从系统移入的自适应测速调度 |
| 其余模块 | 保持现有职责和实现，只有明确列出的 include、残留声明清理允许修改 |

不创建 `motor_control.c`、通用状态机、模式注册表、事件总线、函数指针分发表、动态对象或配置管理器。位置/力矩计算使用 MotorSystem 同文件的少量 static 函数。

## 1. 全程必须保持的行为和接口

### 1.1 配置与控制行为

- 默认 `MOTOR_CONTROL_TORQUE`，默认电位器输入；位置模式仍可切换使用。
- `SYSTEM_BUS_VOLTAGE=15.0f`；没有母线电压采样，不增加估算或自动补偿。
- 力矩单位 N·m；`MOTOR_TORQUE_CURRENT_LIMIT_A=2.5f`，`MOTOR_TORQUE_CONSTANT_NM_PER_A=(8.27f / 291.0f)`，电位器中心 2047.5、死区 80、两端力矩范围 0.05 N·m。
- 电位器使用 `4095-raw`，EMA 系数 0.10，首次采样直接装入；所有原来更新电位器的状态继续更新。
- 音乐自动播放保持关闭；所有音乐代码、曲目和手动播放接口保留。
- 保持 BM3514H 现有参数、相电感 1.2 mH 的 RAM 覆盖行为、所有 PID/前馈系数、方向约定、限幅及浮点计算次序。
- 保持位置 D 项连续角度与专用 Reset；保持测速直接差分及 EMA 0.4，不恢复旧四样本窗口。
- 保持控制/音乐电流环带宽 250/2000 Hz。

### 1.2 不允许修改的接口和状态

- `MotorState`、`MotorControlMode` 的枚举值，`MotorSystem`、`MotorRunData`、`PID_Controller`、`MotorSpeedEstimator`、Flash 记录的布局保持。
- `g_motor_system`、`speed_est`、`speed_pid` 等已有有效外部符号保留。
- 保留 `Motor_System_Init/Task/GetState/StartControl/StopControl/SetControlMode/GetControlMode/SetTorqueCurrent/SetTorqueNm/ClearFault/IdentifyAndSave/PlaySong/PlaySongLoop/StopMusic/EnableSensorlessObserver` 的完整签名、返回值、校验条件和副作用。
- 保留 `Motor_ShowDebugInfo_OLED(void)` 名字、签名和原声明；迁移实现后不增加旧接口转发到新接口的包装层。
- `g_run_requested`、`g_fault_latched`、`g_control_mode`、`g_torque_current_a`、`music_was_active`、电位器初始化标记均保留，不互相替代。
- 系统状态、参数状态和辨识状态不合并。`SAVE_PENDING` 继续属于参数忙状态。
- 力矩接口拒绝请求时的清零副作用保留；不得改成简单提前返回。
- 同模式重复设置不复位；切换模式不自动启动、不自动停止音乐。

### 1.3 禁止触碰的实现

- 不改 `Core` 的 ADC/TIM/DMA/GPIO/I2C/SPI/USART 初始化、中断优先级、回调归属、主循环顺序和 `hardware_init.c` 启动顺序。
- TIM2 顺序保持：`Motor_Parameters_ControlTask1ms()` → `Motor_System_UpdateOperatingState()` → `Motor_System_Task()` → LED。
- ADC 采样、变换和 20 kHz FOC 路径不重构；关闭电流环时仍保留用于辨识的采样。
- 不改现有临界区范围和 PRIMASK 恢复方式；不以 volatile 代替临界区；不新增覆盖计算或 I/O 的长临界区。
- 不改 SVPWM 标度、辨识开环 PWM 算法、Flash 地址/版本/CRC/写入/验证、辨识等待时序。
- `motor_parameters.c` 的自动辨识与手动辨识入口、启停副作用保持原样。辨识期间停止请求和编码器过期处理的问题留给独立功能修复。
- 不删音乐、无感观测、轨迹虚拟位置、详细 OLED 诊断能力、MT6826S 遥测；不改通信帧、DMA 忙时丢帧策略。
- 不改 `.vscode/*`、生成器目录 `cmake/stm32cubemx`、工具链、链接脚本、HAL/CMSIS、原理图文件。
- 除计划要求的说明外，不整文件格式化、不翻译全部注释。新增函数必须有中文注释，说明存在原因、职责、输入/输出单位、调用上下文和重要副作用。

## Stage 0 — 固定基线与验证入口（无控制代码修改）

**文件：**只读全工程；允许新增 `refactor_validation/` 保存基线及验证日志，不参与固件构建。

**问题：**当前存在未提交修改，Git HEAD 不代表实际审计版本；仅编译不能证明模式切换行为保持。

**操作：**

1. 记录 `git status --short`、`git diff --stat` 和当前提交号；备份本计划涉及的原始文件至 `refactor_validation/baseline/`，保留相对目录。新增目标文件若已存在先按不一致规则处理，不能覆盖。
2. 保存文件 SHA256 清单；记录现有枚举、控制宏、20 个 VOFA 通道及主循环/TIM2 顺序。
3. 从项目根目录执行 Debug 配置和构建。工具路径以当前 `.vscode/tasks.json` 为准；以下是审计时命令，不修改该配置文件：

```powershell
$env:PATH = 'D:\software\arm-gnu-toolchain-14.3.rel1\bin;C:\Users\scott\AppData\Local\stm32cube\bundles\ninja\1.13.2+st.1\bin;' + $env:PATH
& 'C:\Users\scott\AppData\Local\stm32cube\bundles\cmake\4.3.1+st.1\bin\cmake.exe' --preset Debug
& 'C:\Users\scott\AppData\Local\stm32cube\bundles\cmake\4.3.1+st.1\bin\cmake.exe' --build --preset Debug
```

4. 每条命令检查退出码；配置失败不得继续构建并把旧 ELF 当作新结果。记录现有警告，不能把原有 Flash memcpy 或 RWX 警告记为本轮新增，也不能顺手修复。
5. 后续每个 Stage 重复构建并检查 diff。需要单元/回放验证时放入 `refactor_validation/` 独立构建，不把测试入口加入固件。

**目标结构/接口：**源码不变；取得可比较的工作区基线。

**风险/调用方：**无运行影响。基线备份不写入 CMake 源码列表。

**验收：**基线构建成功或明确报告原始阻塞；基线不可构建时停止后续修改。不得自动烧录。

## Stage 1 — 删除确认无运行用途的残留（低风险）

**修改文件和精确操作：**

| 文件 | 操作 |
|---|---|
| `user/src/motor_system.c` | 删除 `g_debug_uvw_dir` 定义及两处赋值；删除注释掉的 `Motor_SimulateSpring_Task()` 调用 |
| `user/inc/motor_system.h` | 仅删除 `Motor_SimulateSpring_Task(void)` 声明 |
| `user/inc/svpwm.h` | 仅删除没有实现的 `Motor_OpenLoop_Vdq_Control(...)` 声明 |
| `user/src/motor_current_loop.c` | 仅删除注释掉的 `Motor_OpenLoop_Vdq_Control(...)` 调用，不改周围语句 |
| `user/inc/as5600.h` | 删除无引用的 `I2C_TIMEOUT` 宏 |
| `user/src/oled.c` | 删除无引用的 `BitAction` typedef 整块，不删其他类型 |

**问题/收益：**这些内容只写不读、无实现或无引用；删除后不再让读者误以为存在弹簧控制/旧 Vdq 接口。

**执行前验证：**用 `rg -n` 搜索上述符号在全仓库的引用，排除构建产物和基线备份；逐条区分注释与有效代码。若出现未列出的有效用途，停止该删除。

**目标结构/保持接口：**有效 API 全保留；不删除旧但有实现的弱磁、VOFA 包装、整定、观测器 getter 等接口。

**影响调用方：**当前无有效调用方；调试器手工监视 `g_debug_uvw_dir` 将失效，在结果中注明。

**验收：**构建成功；diff 中没有新增控制语句或公式变化；残留符号在活动源码中消失。

## Stage 2 — 把共享常量移出系统管理头（低风险）

**文件：**新增 `user/inc/motor_config.h`；修改 `user/inc/motor_system.h`；修改 `user/src/motor_current_loop.c`、`motor_identify.c`、`motor_trajectory.c`、`svpwm.c` 的 include。

**问题：**低层模块只为一个常量而包含系统状态和音乐接口。

**机械操作：**

1. 新头文件使用独立 include guard，不包含 HAL、系统、音乐头，不定义变量或函数。
2. 从 `motor_system.h` 原样移动以下宏及对应中文说明，数值、后缀、表达式不改：`SYSTEM_BUS_VOLTAGE`；`MOTOR_TORQUE_CURRENT_LIMIT_A`、`MOTOR_TORQUE_CONSTANT_NM_PER_A`、`MOTOR_TORQUE_USE_POT`、`MOTOR_TORQUE_POT_CENTER`、`MOTOR_TORQUE_POT_DEADBAND`、`MOTOR_TORQUE_POT_MAX_NM`；`MOTOR_SYSTEM_TASK_DT_SEC`；全部 `SPEED_EST_*` 宏。
3. `MOTOR_DEFAULT_CONTROL_MODE` 留在 `motor_system.h` 的模式枚举之后，因为它引用该枚举。类型、所有有效声明位置不要求调整。
4. `motor_system.h` 包含 `motor_config.h`，让旧调用方仍能通过原头文件取得常量。禁止复制宏或建立同值别名。
5. 上述四个 `.c` 中把 `#include "motor_system.h"` 替换为 `#include "motor_config.h"`。若编译暴露原来依赖的其他声明，直接包含真实所属头；不能恢复无必要的系统依赖来掩盖缺失。
6. 系统私有 `POT_LPF_ALPHA`、新鲜度门限及带宽常量保持在原文件；其他模块已有配置不迁移。

**目标/调用方：**算法读取相同宏；`hardware_init.c` 等使用系统 API 的调用方继续包含原头，不改。

**风险：**include 传递依赖；不能借机调整常量或参数组织。

**验收：**配置宏逐项与基线比较一致；四个模块不使用系统类型/函数；Debug 构建成功、无新增宏重定义警告。

## Stage 3 — 整体迁移前台调试代码（低至中风险）

**文件：**新增 `user/src/motor_debug.c`、`user/inc/motor_debug.h`；修改 `user/src/motor_system.c` 和根 `CMakeLists.txt`。`Core/Src/main.c` 及 `motor_system.h` 中原调试入口声明不改。

**问题：**系统文件同时组织 OLED 和 VOFA；其私有调试量需要明确的只读出口。

**机械操作：**

1. 把 `Motor_ShowDebugInfo_OLED(void)` 完整函数体移到 `motor_debug.c`，原文件删除定义。新文件包含它实际需要的头：`motor_debug.h`、`motor_system.h`、`motor_current_loop.h`、`motor_speed_loop.h`、`motor_identify.h`、`motor_parameters.h`、`motor_sensorless.h`、`motor_trajectory.h`、`as5600.h`、`mt6826s.h`、`user_io.h`、`oled.h`、`vofa_usart.h`；按编译补充真实依赖，禁止包含 `.c`。
2. `motor_debug.h` 包含 `motor_feedforward.h`，声明原显示函数，并声明唯一新增读取接口：

```c
void Motor_System_ReadDebugData(MotorFeedforwardResult *feedforward,
                               float *pot_target_counts);
```

3. 该接口在 `motor_system.c` 实现并包含 `motor_debug.h`。两个指针由唯一调用方保证非空；中文注释写清此前提和“仅用于前台观测，不保证整帧跨中断原子快照”。不新增关中断操作，不做 I/O，不调用算法。
4. 本 Stage 保留原私有调试变量。读取接口把四个 `g_debug_*` 的值写入对应 `MotorFeedforwardResult` 字段；`output_iq` 显式填 `0.0f`，注明该输出参数字段在此接口暂不使用；电位器输出来自 `g_debug_pot_target_pos`。不得用未初始化字段。
5. 在显示函数中、组装 VOFA 前调用一次读取接口，取得局部 `MotorFeedforwardResult` 和局部电位器值；仅替换 CH4/5/6/9/13 对私有变量的访问。CH7 仍直接读取 `g_foc_state.pi_q.target`。
6. 保持所有其他读取调用、OLED 分支、传感器查询和发送方式。允许五个调试字段集中读取形成帧内略不同采样时刻；不能宣称整帧同步，也不能借此改变控制数据。若需要严格同时采样，应另开任务。
7. 保留 `last_refresh_ms` 和 `< 2U` 门限；把错误的 50 ms 说明改成“2 ms 门限，实际周期还受前台工作耗时影响”，删除失效 `HAL_Delay` 注释。把详细诊断的 `if (0)` 改为文件私有宏 `MOTOR_DEBUG_OLED_DETAILS`（默认 0）的条件，完整保留原诊断内容；此宏不作为全局配置。
8. 在根 CMake 的 `target_sources` 添加 `user/src/motor_debug.c`，仅添加一次。从系统源文件移除确实仅用于迁出代码的 `oled.h`、`vofa_usart.h`、`mt6826s.h`；其他 include 不批量清理。

**保持接口/调用方：**main 原调用原函数，不增加转发；UART 驱动不感知业务；不把 OLED 或 SPI 读取搬进中断。

**VOFA 不变清单：**

| CH | 值/尺度 |
|---|---|
| 0 / 1 | `4095-Pot_ReadRaw()` / AS5600 原始角度 |
| 2 / 3 | `speed_pid.target` / `speed_pid.measure` |
| 4 / 5 / 6 | 基础/摩擦/惯性电流乘 1000 |
| 7 / 8 | `g_foc_state.pi_q.target` / `g_foc_state.park.q`，各乘 1000 |
| 9 / 10 | 前馈有符号加速度 / `MT6826S_ReadRawAngle15()` |
| 11 / 12 | `Motor_Trajectory_GetAccel()*0.1` / `Motor_Trajectory_GetFilteredTarget()` |
| 13 / 14 | 电位器滤波计数 / `Motor_Trajectory_GetPosition()` |
| 15–18 | 原观测器输出的电角度、机械 RPM、反电动势幅值、valid |
| 19 | 系统状态枚举转 float |

**风险：**漏源文件、重复定义、误改通道尺度、误将 CH3 换成实时速度或 CH7 换成基础目标。

**验收：**重新配置并构建；链接只有一个显示函数定义；逐通道对照表；确认阻塞 SPI 和 OLED 仍只在前台。硬件可用且已获运行授权时观察帧格式/显示，不自动启动辨识。

## Stage 4 — 收敛调试副本与相同复位序列（中风险）

**文件：**仅 `user/src/motor_system.c`。新接口和其调用方保持。

**问题：**四个独立调试字段重复表达现有结果类型；四条相同外环历史复位散落三处。

**机械操作 A：调试数据**

1. 新增文件私有 `static MotorFeedforwardResult g_last_feedforward = {0};`。
2. 将 `g_debug_speed_loop_iq/g_debug_friction_iq/g_debug_inertia_iq/g_debug_target_accel_rpm_s` 的每个读写位置分别替换为 `g_last_feedforward.speed_loop_iq/friction_iq/inertia_iq/accel_rpm_s`，然后删四个旧定义。
3. 保持原来四个赋值/清零的顺序和执行分支。不要改成整结构体赋值、memset 或在 Task 入口统一清零；不在 Init/Stop/SetMode 增加原来没有的调试清零。
4. `g_last_feedforward.output_iq` 保持 0，不用于控制或 CH7。采用已有类型只是避免重复字段定义，不建立第二份控制输出。
5. 删除 `g_debug_pot_target_pos` 和每周期复制；读取接口改读 `g_pot_target_filtered`。其余电位器语句不改。二者在首次任务执行后相同、上电均为 0；不得把滤波初始化标记删除。

**机械操作 B：复位**

1. 新增 `static void Motor_System_ResetControlHistory(void)`，函数体严格只有原顺序四句：

```c
Motor_SpeedLoop_SetTarget(0.0f);
PID_Reset(&speed_pid);
Motor_PositionLoop_Reset();
Motor_Trajectory_Clear();
```

2. 仅替换三处完全相同的连续四句：`Motor_System_ResetOuterLoops()` 内、音乐首次进入分支内、Task 非运行分支内。
3. `ResetOuterLoops()` 前面的 target_torque 清零、后面的 q/d 清零保留原位置。音乐进入前的力矩存储清零/带宽切换和后面的标记写入保持。非运行分支后面的调试/q/d 清零保持。
4. 不直接把后两处替换成 `ResetOuterLoops()`，因为它有额外写入，会改变副作用位置。
5. 保留 ForceSafeStop 中四个条件式关闭，不改成无条件调用；不减少停机状态下周期性复位次数。

**目标/调用方：**原分支和复位时机不变，共同四句只有一处实现；外部接口不变。没有新文件或框架。

**风险：**复位提前/延后、调试历史被过早抹掉、关断调用次数变化。

**验收：**构建；展开新 helper 后逐语句与基线比较，变量替换之外控制写入顺序相同。用复位调用记录或调试断点验证位置↔力矩、首次音乐/持续音乐/退出、停止/故障路径；禁止仅凭行数减少验收。

## Stage 5 — 将模式计算拆成同文件静态函数（中风险）

**文件：**仅 `user/src/motor_system.c`。

**问题：**Task 内同时展开公共输入与两个模式算法，读者难以看清调度顺序。

**固定函数与移动边界：**

1. 新增 `static float Motor_System_UpdatePotTarget(void)`：完整移入从 `pot_raw=Pot_ReadRaw()` 到滤波结果局部变量赋值的电位器处理块，返回滤波计数。`g_foc_state.target_d=0.0f` 留在 Task 原位置。Task 用 `float target_pos = Motor_System_UpdatePotTarget();` 替代原块。
2. 新增 `static void Motor_System_RunTorqueMode(void)`：只移入原 TORQUE 分支大括号内部语句，除最后 `return`；包括条件编译的电位器换算、四项调试清零、q/d 给定、Kt 校验和目标力矩反算。Task 保留原双条件判断，调用 helper 后原样 return。
3. 新增 `static void Motor_System_RunPositionMode(float target_pos, float current_rpm)`：只移入原 SENSORED_RUN 位置分支内部语句；局部 actual_pos、actual_mech_rpm、方向、轨迹/速度/前馈调用及调试赋值原序保留。Task 保留该 if/else，else 的清理不移位。
4. 三个函数放在 Task 前、按公共输入→力矩→位置顺序排列。加中文函数注释；不为每个公式再封装小函数。
5. 音乐入口/退出、`music_was_active`、自动播放条件编译、测速和方向更新仍留 Task。不得把音乐清零放入共同入口。
6. 本 Stage 不合并两次方向读取、不把局部方向替换成前面已校验的值、不改变 `UpdateOperatingState()` 的分支和读取顺序。微小冗余保留，以避免无意修复异常参数行为。
7. 可将位置 helper 内局部 `target_elec_rpm` 统一改名为 `target_signed_rpm`，所有引用一并替换；注释明确“按电磁转矩正方向统一符号的机械 RPM，没有乘极对数”。只改该局部名字，不改 VOFA 值或公共 API。

**目标 Task 顺序：**测速/过期处理 → 方向和观测器方向 → 电位器 → 原 d 轴清零 → 自动播放（仍默认关闭）→ 音乐处理与 return → 退出音乐处理 → 原目标力矩清零 → 力矩 helper 与 return → 位置 helper 或非运行清理。

**保持接口/调用方：**TIM2 继续调用 `Motor_System_Task`；所有 public API 及状态检查不变；算法模块不增加转发接口。

**风险：**移动边界漏掉副作用、改变 early return 或条件编译，使位置环覆盖力矩输出。

**验收：**构建；helper 展开后的调用/写入顺序与 Stage 4 一致。对照执行以下场景：参数未就绪、故障、停止、正常力矩、正常位置、音乐首周期/中间周期/结束后的下一周期。逐周期比对状态、目标 q/d、目标力矩、速度 PID 目标/历史、位置复位和轨迹清理次数。测试可用桩，但必须调用实际重构代码，不能重写算法来“证明”自身。

默认配置和手动给定配置均应编译验证：在独立临时验证副本中将 `MOTOR_TORQUE_USE_POT` 改为 0 构建并检查对应路径；不得把这个改动带回交付源码。仅默认配置通过时，手动分支记未验证。

## Stage 6 — 将自适应测速调度归入速度模块（本轮最高风险）

**前置：**Stage 5 验收通过；本阶段另行保存前后输入/输出回放，不与模式计算一起修改。

**文件：**`user/src/motor_system.c`、`user/src/motor_speed_loop.c`、`user/inc/motor_speed_loop.h`。

**问题：**估算器在速度模块，采样周期和累计时间在系统模块；原函数还同时写系统速度和返回相同值。

**固定新增接口：**

```c
float Motor_SpeedEstimator_UpdateAdaptive(float published_speed_rpm);
```

**必须按以下算法移动，禁止自行优化：**

1. 把 `Motor_SpeedEstimator_GetPeriodTicks()` 移入 `motor_speed_loop.c`，仍为 static；阈值和 >= 比较原样保留。绝对值使用原 `(value < 0.0f) ? -value : value` 表达式，不建立公共数学模块。
2. 把 `Motor_UpdateSpeedEstimatorAdaptive()` 移入同文件并改为上述公开函数。速度源文件直接包含 `motor_config.h`、`as5600.h`。不改 `MotorSpeedEstimator` 结构体，不创建新估算器。
3. `ticks` 和 `dt_acc` 继续为函数内 static；初始化、递增、累加、比较后复位的顺序与原代码一致。
4. 未 initialized 时：清 ticks/dt_acc，调用原 `Motor_SpeedEstimator_Update(AS5600_ReadRawAngle(), MOTOR_SYSTEM_TASK_DT_SEC)`，直接返回结果。
5. initialized 时：ticks++、累计 1 ms；使用参数 `published_speed_rpm` 选择周期；到期才读取 AS5600 并调用原 Update，随后清 ticks/dt_acc 并返回该结果；未到期返回传入的 `published_speed_rpm`。
6. 系统原 fresh 分支唯一替换为：`g_motor_system.run_data.speed_rpm = Motor_SpeedEstimator_UpdateAdaptive(g_motor_system.run_data.speed_rpm);`。stale 分支仍只把发布速度写 0，不调用自适应函数、不推进计时、不复位估算器。
7. 特别禁止直接用 `speed_est.speed_rpm` 替代参数。过期时发布速度为 0，但估算器可能仍保存旧速度，两者并不恒等；替代会改变恢复时分频和输出。
8. 不改已有 `Motor_SpeedEstimator_Init/Update()`、EMA、跨零算法，也不在 Init 中额外复位这两个 static；首次未 initialized 调用原来就会复位。
9. 系统中的 `Motor_AbsFloat` 留给电位器映射，不因测速搬走而删除。

**接口/调用方：**新增函数只有系统 fresh 分支调用；原 Init/Update 和 speed_est 布局保留；速度模块不包含 motor_system.h，也不访问 g_motor_system。

**风险：**首次采样、数据过期恢复、分频边界的微小时序变化会改变闭环响应。

**验证：**使用 Stage 0 保存的旧自适应函数作参考，旧/新实现分别重置运行并回放完全相同的角度、新鲜度和发布速度序列。记录每次真实 Update 的调用 tick、角度、dt，以及每周期返回速度。要求更新 tick 完全一致，dt/输出按相同浮点执行环境一致，不放宽阈值掩盖变化。

回放必须覆盖：首次 initialized=0；原始角度 `4094,4095,0,1` 和反向；静止量化抖动；绝对速度在 50/200/500 阈值下方、等于和上方；正负速度；连续多周期 stale 后恢复；再次 Init；未到期时 published speed=0 而内部 speed 非零。读角度桩必须记录调用次数，不能提前每周期读来替代到期读取。

Debug 构建通过后，在有授权和硬件时测 TIM2/ADC 周期与位置/力矩输出；无法测量则标注实时性与实机结果待验证，不以编译成功宣称完全等价。

## 2. 本轮明确不执行的审计候选

以下是范围约束，不是供执行模型选择的待办：

- 不删除四个旧 `VOFA_Send_*`、两种旧整定入口、弱磁函数、无感单值 getter、参数 HasStoredData、轨迹 GetVelocity 或 g_traj_velocity；当前未调用不足以证明外部调试/API 无用途。
- 不删除 `PID_Controller.prev_error`，不改变公开结构体布局。
- 不统一 StartControl/ClearFault/IdentifyAndSave 的校验 helper；它们条件不同。
- 不将状态判断改成数据驱动表，也不合并音乐接口检查。
- 不迁移参数模块内的输出关闭流程，不改自动辨识启动路由，不修复辨识中停止与 stale 的既有行为。此工作涉及功能语义，应在独立任务中重新定义和验证。
- 不把轨迹目标改为位置环输入，不删除轨迹遥测，不调整摩擦或惯性补偿。
- 不为追求更短文件继续拆分 LED、命令换算、简单 getter 或状态函数。

## 3. 最终交付和验收记录

每个 Stage 在 `refactor_validation/RESULTS.md` 记录：修改文件/符号、完成操作、构建命令与退出码、新增/原有警告、验证输入与结果、未完成的硬件验证、不一致或阻塞。不得把“未测”写成“通过”。

最终检查：

1. diff 只含计划允许的源码/头文件、根 CMake 新源登记和验证资料；用户原有改动未被撤回。
2. 原有有效公开接口/枚举/数据布局保持，默认力矩和关闭自动音乐配置保持。
3. 新文件只有配置头、调试模块及验证资料；不出现额外控制框架。
4. Debug 构建成功；Stage 4–6 的顺序/状态/数值验证有证据；实机项目单独标记。
5. 最终报告写清实际完成的 Stage、未验证项和残留风险，不以 MotorSystem 行数作为正确性证明。

没有明确的另行授权，不烧录、不自动驱动电机、不提交或推送 Git。
