# REFACTOR_PLAN 执行记录

基线：工作区当前内容（含此前未提交修改），非 Git HEAD。
审计基线 `user/src/motor_system.c` 786 行。
验证资料目录 `refactor_validation/` 不参与固件构建。

## Stage 0 — 固定基线与验证入口（部分）

- `git status --short`、`git log --oneline -3`、涉及文件 SHA256 已保存至 `refactor_validation/baseline/`。
- 工具链存在：`arm-gnu-toolchain-14.3.rel1`、CMake `4.3.1+st.1`、Ninja `1.13.2+st.1`。
- 配置命令（项目根）：`cmake --preset Debug` → 退出码 0。
- 基线构建：`cmake --build --preset Debug` → 退出码 0（`ninja: no work to do.`，工作区与既有产物一致）。
- 既有警告（非本轮新增）：`motor_parameters.c` Flash `memcpy -Wstringop-overread`；链接 `LOAD segment with RWX permissions`。
- 未执行：未烧录、未驱动电机。

## Stage 1 — 删除确认无运行用途的残留（低风险）

执行前 `rg` 等价的全文搜索确认各符号只有定义、注释或无实现声明，无有效调用。

| 文件 | 操作 | 结果 |
|---|---|---|
| `user/src/motor_system.c` | 删除 `g_debug_uvw_dir` 定义（1 处）、两处赋值、以及注释掉的 `Motor_SimulateSpring_Task()` 调用 | 完成 |
| `user/inc/motor_system.h` | 删除 `Motor_SimulateSpring_Task(void)` 声明 | 完成 |
| `user/inc/svpwm.h` | 删除无实现的 `Motor_OpenLoop_Vdq_Control(...)` 声明 | 完成 |
| `user/src/motor_current_loop.c` | 删除注释掉的 `Motor_OpenLoop_Vdq_Control(...)` 调用 | 完成 |
| `user/inc/as5600.h` | 删除无引用的 `I2C_TIMEOUT` 宏及其说明注释 | 完成 |
| `user/src/oled.c` | 删除无引用的 `BitAction` typedef 整块 | 完成 |

验证：

- 全仓库搜索引擎确认 `g_debug_uvw_dir`、`Motor_SimulateSpring_Task`、`Motor_OpenLoop_Vdq_Control`、`I2C_TIMEOUT`、`BitAction` 在活动源码中已无匹配（仅计划文档尚存文字）。
- 与 `baseline/` 逐文件 diff：只有上述删除，无新增控制语句、无公式或次序变化。
- `cmake --build --preset Debug` → 退出码 0，链接成功；无新增警告，仍为两条既有警告。
- RAM 4264 B / 32 KB，FLASH 80028 B / 126 KB，MOTOR_PARAMS 0 B。

接口/行为：有效公开 API 全部保留；仅调试器手工监视 `g_debug_uvw_dir` 的入口失效（已注明）。
实机观察：未验证（无运行授权/未烧录），本 Stage 为纯删除，不改变运行时行为。

计划一致性：与 `REFACTOR_PLAN.md` Stage 1 完全一致，无冲突。

## Stage 2 — 把共享常量移出系统管理头（低风险）

新增 `user/inc/motor_config.h`（独立 include guard，不含 HAL/系统/音乐头，无变量/函数）。
从 `user/inc/motor_system.h` 原样移动以下 15 个宏（数值/后缀/表达式/中文注释完全一致）：

| 常量 | 原位置 | 现位置 |
|---|---|---|
| `SYSTEM_BUS_VOLTAGE` | `motor_system.h` 全局配置区 | `motor_config.h` |
| `MOTOR_TORQUE_CURRENT_LIMIT_A` | `motor_system.h` | `motor_config.h` |
| `MOTOR_TORQUE_CONSTANT_NM_PER_A` | `motor_system.h` | `motor_config.h` |
| `MOTOR_TORQUE_USE_POT` | `motor_system.h` | `motor_config.h` |
| `MOTOR_TORQUE_POT_CENTER` | `motor_system.h` | `motor_config.h` |
| `MOTOR_TORQUE_POT_DEADBAND` | `motor_system.h` | `motor_config.h` |
| `MOTOR_TORQUE_POT_MAX_NM` | `motor_system.h` | `motor_config.h` |
| `MOTOR_SYSTEM_TASK_DT_SEC` | `motor_system.h` | `motor_config.h` |
| `SPEED_EST_LOW/MID/HIGH_RPM_THRESHOLD` | `motor_system.h` | `motor_config.h` |
| `SPEED_EST_LOW/MID/HIGH/MAX_PERIOD_TICKS` | `motor_system.h` | `motor_config.h` |

- `MOTOR_DEFAULT_CONTROL_MODE` 按要求保留在 `motor_system.h` 的 `MotorControlMode` 枚举之后。
- `motor_system.h` 增加 `#include "motor_config.h"`，旧调用方（`hardware_init.c`、`motor_system.c`）仍可透传取得常量。
- 四个低层模块把 `#include "motor_system.h"` 改为 `#include "motor_config.h"`：`motor_current_loop.c`、`motor_identify.c`、`motor_trajectory.c`、`svpwm.c`。
- 系统私有 `POT_LPF_ALPHA`、新鲜度门限 `MOTOR_AS5600_MAX_SAMPLE_AGE_MS`、带宽常量仍留在 `motor_system.c`，未迁移。

验证：

- 迁移宏行与 `baseline/user/inc/motor_system.h` 逐行 `Compare-Object`：15/15 **完全一致**。
- 全仓库 `#define` 搜索：上述宏在新源码中只出现在 `motor_config.h`，无重复定义、无旧定义残留（baseline 副本除外）。
- 四个模块编译期无对系统类型/函数/变量的依赖（grep `MotorState/MotorControlMode/MotorSystem/g_motor_system/MOTOR_STATE_/MOTOR_CONTROL_/Motor_System_/Motor_ShowDebugInfo_OLED` 均无命中）。
- include 关系无环：`motor_config.h` 不包含任何其他头；除 `motor_system.h` 外无其他头包含 `motor_system.h`。
- `cmake --preset Debug` → 退出码 0；`cmake --build --preset Debug` → 退出码 0，链接成功；无新增 warning，仅原有链接 RWX 警告。
- RAM 4264 B / 32 KB，FLASH 80028 B / 126 KB，MOTOR_PARAMS 0 B：与 Stage 1 完全一致，无 codegen 变化。

接口/行为：公开 API、枚举、结构体布局、控制公式、限幅、FOC/中断/ADC-DMA 均未改动。
实机观察：未验证（未烧录），本 Stage 为纯常量搬迁，不改变运行时行为。

计划一致性：与 `REFACTOR_PLAN.md` Stage 2 一致，无冲突。

## Stage 3 — 整体迁移前台调试代码（低至中风险）

新增文件：

- `user/inc/motor_debug.h`：include `motor_feedforward.h`；声明 `Motor_ShowDebugInfo_OLED(void)` 与唯一新增读取接口 `Motor_System_ReadDebugData(MotorFeedforwardResult *, float *)`。
- `user/src/motor_debug.c`：完整迁入原显示函数体；文件私有 `#define MOTOR_DEBUG_OLED_DETAILS 0`。

修改文件：

- `user/src/motor_system.c`
  - 加入 `#include "motor_debug.h"`。
  - 删除仅被迁出代码使用的 `#include "oled.h"`、`#include "vofa_usart.h"`、`#include "mt6826s.h"`；其余 include 未动。
  - 删除 `Motor_ShowDebugInfo_OLED()` 定义，原位置改为实现 `Motor_System_ReadDebugData()`：只读拷贝四个 `g_debug_*` 与 `g_debug_pot_target_pos`，`output_iq` 显式 `0.0f`，无临界区、无 I/O、无算法调用。
- `CMakeLists.txt`：在 `target_sources` 的 `motor_current_loop.c` 之后加入 `user/src/motor_debug.c`，仅一次。
- `Core/Src/main.c` 与 `motor_system.h` 中的调试入口声明**未修改**。

显示函数内的计划指定改动：

- 组装 VOFA 前调用一次 `Motor_System_ReadDebugData(&feedforward, &pot_target_counts)`。
- 仅替换 CH4/5/6/9/13 的私有变量访问；CH7 仍为 `g_foc_state.pi_q.target`。
- 详细诊断 `if(0)` 改为 `if (MOTOR_DEBUG_OLED_DETAILS)`，OLED 语句原样保留（含原注释）。
- 刷新门限 `last_refresh_ms` 与 `< 2U` 逻辑不变；删除失效 `HAL_Delay` 注释，把错误 50 ms 说明改为“2 ms 门限，实际周期还受前台工作耗时影响”。

VOFA 20 通道逐项核对（与 baseline 对比）：CH0–3 原样；CH4/5/6/9/13 由等价局部值替换，来源映射为 `feedforward.speed_loop_iq/friction_iq/inertia_iq/accel_rpm_s`、`pot_target_counts`；CH7/8/10/11/12/14–19 完全不变。顺序、倍率、发送方式未变。

验证：

- `cmake --preset Debug` → 0；`cmake --build --preset Debug` → 0，链接成功；无新增 warning，仅原有链接 RWX 警告。
- `Motor_ShowDebugInfo_OLED()`：全工程只有 `motor_debug.c` 一处定义，唯一调用点为 `Core/Src/main.c:120`（前台）。
- `Core/Src/stm32g4xx_it.c` 中无 `OLED_/VOFA_/Motor_ShowDebugInfo` 调用；TIM2 回调仍只调用参数任务/状态/Task/LED。
- RAM 4264 B（不变）；FLASH 80028 → 80116 B（+88 B）。来源：新增 `Motor_System_ReadDebugData` 函数体及其跨编译单元的调用开销（原四个调试量直接内联访问，现改为一次函数调用+结构体拷贝）；无算法新增。
- 阻塞 SPI（MT6826S）与 OLED 仍仅经前台显示函数调用，不进入中断。

偏差：无实质偏差。唯一曾是：初次迁移时删除了一行随 `if(0)` 存在的旧注释，已按计划 §0.1.7 恢复原样，最终只有 `if(0)` 行按计划改为宏。

未验证项：实机 OLED 显示与 VOFA 帧格式未上电观察（未烧录），按计划记为未验证。

## Stage 4 — 收敛调试副本与相同复位序列（中风险）

仅修改 `user/src/motor_system.c`。

操作 A（调试数据）：

- 新增文件私有 `static MotorFeedforwardResult g_last_feedforward = {0};`。
- 四个旧调试字段 `g_debug_speed_loop_iq` / `g_debug_friction_iq` / `g_debug_inertia_iq` / `g_debug_target_accel_rpm_s` 的每个读写点替换为 `g_last_feedforward.speed_loop_iq` / `friction_iq` / `inertia_iq` / `accel_rpm_s`，随后删除四个旧定义。
- 保持原赋值/清零顺序与分支：音乐分支清零 4 处、力矩分支清零 4 处、位置分支赋值 4 处、非运行分支清零 4 处、`Motor_System_ReadDebugData` 读取 4 处。未使用整结构体赋值、未使用 memset、未在 Task 入口统一清零、未在 Init/Stop/SetMode 新增清零。
- `g_last_feedforward.output_iq` 保持 0，不参与控制，CH7 仍为 `g_foc_state.pi_q.target`，与 `output_iq` 无关。
- 删除 `g_debug_pot_target_pos` 定义及每周期拷贝；`Motor_System_ReadDebugData` 改读 `g_pot_target_filtered`。`g_pot_filter_initialized` 与电位器滤波逻辑未动。

操作 B（复位）：

- 新增 `Motor_System_ResetControlHistory(void)`，函数体严格为原顺序四句：`Motor_SpeedLoop_SetTarget(0.0f)`、`PID_Reset(&speed_pid)`、`Motor_PositionLoop_Reset()`、`Motor_Trajectory_Clear()`。
- 仅替换三处完全相同且连续的四句：`Motor_System_ResetOuterLoops()` 内、音乐首次进入分支内、Task 非运行分支内。
- `ResetOuterLoops()` 的 target_torque 清零（前）与 q/d 清零（后）、音乐进入前的力矩清零/带宽切换与后面的标记写入、非运行分支后的调试/q/d 清零，位置均保持。
- 未把后两处改成调用 `ResetOuterLoops()`；`ForceSafeStop()` 的四个条件式关闭未改；停机周期复位次数未变。

验证（展开 helper 逐语句对照 Stage 3 基线）：

- 基线中四句连续序列出现在 3 处（`baseline` 行 387-390 / 547-550 / 650-653）；当前只剩 helper 1 处（行 385-388），三个调用点分别位于 `ResetOuterLoops`（行 395）、音乐首次进入（行 551）、非运行分支（行 650）。展开后与基线逐句顺序一致。
- 三个复位的触发条件/时机未变：模式切换与 `ForceSafeStop` 经 `ResetOuterLoops`；音乐首周期经 `music_was_active==0`；非 SENSORED_RUN 经 else 分支。
- 调试字段读写点数量与条件一致（音乐/力矩/非运行各 4 清零，位置 4 赋值，读取接口 4 读取）；`g_debug_*`、`g_debug_pot_target_pos` 在活动源码已无残留。
- `Motor_System_ReadDebugData` 的 CH4/5/6/9 来源为 `g_last_feedforward.*`（与旧 `g_debug_*` 同值），CH13 来源为 `g_pot_target_filtered`（与旧 `g_debug_pot_target_pos` 同值）；上电初值均为 0，首次任务后一致。
- `cmake --preset Debug` → 0；`cmake --build --preset Debug` → 0，链接成功；无新增 warning，仅原有链接 RWX 警告。
- RAM 4264 B（不变）：旧 4 个调试变量（16 B）+ `g_debug_pot_target_pos`（4 B）= 20 B，替换为 `g_last_feedforward`（5×float=20 B），总量相同。
- FLASH 80116 → 80036 B（−80 B）：集中复位序列与调试写入减少重复代码；无算法变化。

偏差：无。计划指定的四句仍连续、调用条件一致、未发现新调用者。

未验证项：实机位置↔力矩、首次/持续/退出音乐、停止/故障路径的复位行为未上电验证（未烧录、未驱动电机），按计划记为未验证。

## Stage 5 — 将模式计算拆成同文件静态函数（中风险）

仅修改 `user/src/motor_system.c`。

- 在 `Motor_System_Task` 前、按“公共输入 → 力矩 → 位置”新增三个 static helper：
  - `Motor_System_UpdatePotTarget(void)`：完整移入电位器处理块并返回滤波计数；`g_foc_state.target_d = 0.0f` 保留在 Task 原位置。`float target_pos = g_pot_target_filtered;` 改为 `return g_pot_target_filtered;`（同值）。
  - `Motor_System_RunTorqueMode(void)`：仅移入力矩分支大括号内语句（含 `#if MOTOR_TORQUE_USE_POT` 换算式、四项调试清零、q/d 给定、Kt 反算）；`return` 留在 Task。
  - `Motor_System_RunPositionMode(float target_pos, float current_rpm)`：仅移入 SENSORED_RUN 位置分支内部语句；局部 `target_elec_rpm` 改名为 `target_signed_rpm`，注释注明“按电磁转矩正方向统一符号的机械 RPM，没有乘极对数”。Task 的 if/else 结构保留，else 清理未移动。
- Task 保留全部：测速/过期处理、方向与观测器方向、`target_d` 清零、自动播放条件编译、音乐处理与 `return`、音乐退出带宽切换、目标力矩清零、力矩双条件判断+helper+return、位置 helper 或非运行清理、弱磁注释。
- 未合并两次方向读取；未修改 early return、q/d 写入顺序、轨迹/速度/前馈调用顺序、调试字段写入顺序、条件编译分支、音乐相关分支。

验证（归一化逐语句对照，去注释/空白并统一 `g_debug_*`→`g_last_feedforward.*`、`target_elec_rpm`→`target_signed_rpm`）：

- TORQUE 块：与 `baseline` **完全一致**（`#if` 换算、四项清零、q/d、Kt 反算顺序一致）。
- POSITION 块：与 `baseline` 一致，唯一差异是 Stage 1 已删除的 `g_debug_uvw_dir = uvw_dir;`（预期）。
- POT 块：逐句一致；末尾由 `float target_pos = g_pot_target_filtered;` 变为 `return g_pot_target_filtered;`，且不含 Stage 4 已删的调试拷贝。
- Task 总体顺序与计划目标一致：测速/过期 → 方向与观测器方向 → 电位器 → d 轴清零 → 自动播放 → 音乐处理及 return → 音乐退出 → 目标力矩清零 → 力矩 helper 及 return → 位置 helper 或非运行清理。
- 分支级场景核对（静态）：参数未就绪/故障/停止由 `UpdateOperatingState` 切状态后走 else 清理；正常力矩走力矩 helper+return；正常位置走位置 helper；音乐首周期走 `music_was_active==0`，持续周期跳过首进入，退出后下一周期切回控制带宽并清零标记。均为代码路径核对，未上电。
- 默认构建：`cmake --preset Debug` / `--build --preset Debug` 均退出码 0；无新增 warning（仅原有 RWX）。RAM 4264 B 不变；FLASH 80036 → 80132 B（+96 B）：Debug(`-O0`) 下三个 static helper 未内联，新增函数序言与调用开销；Release 下预计可内联。
- 手动配置：独立临时副本（复制到系统临时目录）内将 `MOTOR_TORQUE_USE_POT` 改为 0，`cmake --preset Debug` / `--build --preset Debug` 均退出码 0，无新增 warning，FLASH 79740 B；验证后删除副本，交付源码仍为 `MOTOR_TORQUE_USE_POT 1`。

偏差：无。helper 为纯机械搬移，未发现边界/副作用错位或新调用者。

未验证项：实机参数未就绪/故障/停止/正常力矩/正常位置/音乐首-中-退出各路径的运行时行为未上电验证（未烧录、未驱动电机）；手动给定分支仅验证可编译与路径存在，未上电。

## Stage 5 补充验证 — 主机 A/B 行为等价（动态）

背景：原 Stage 5 记录的 8 个场景为“分支级静态核对”，未满足计划“调用实际重构代码、逐周期运行验证”的要求。本节补充动态验证。

方法：

- 在 `refactor_validation/stage5_equiv/` 建立独立主机验证桩：
  - `shim/`：最小 `stm32g4xx.h` / `stm32g4xx_hal.h` 替身（HAL 类型、`HAL_GetTick`、CMSIS 临界区替身），仅用于主机编译被测源文件。
  - `test_support.c/.h`：所有外部依赖的确定性桩、全局量定义、副作用调用顺序追踪（事件串）与计数器。
  - `driver.c`：固定时间线调用**真实** `HAL_TIM_PeriodElapsedCallback()`（内部依次 `Motor_Parameters_ControlTask1ms` → `UpdateOperatingState` → `Motor_System_Task` → LED），不使用任何复写算法。
  - `build_run.ps1`：编译两份并逐行比对。
- 被测两份：`baseline`（Stage 0 保存的 `motor_system.c`）与当前 `user/src/motor_system.c`（Stage 5 后）。
- 主机编译器：经用户授权，用 `uv` 在**仓库外**临时环境获取 `ziglang 0.16.0`（`zig cc` / clang 21）编译；临时 venv 位于系统临时目录，不进入仓库、不改变固件工具链。**正式源码本次未被修改**。

场景与周期：A 参数未就绪×3、B 故障×3、C 停止×3、D 正常力矩×5、E 正常位置×5、F 音乐首次×1、G 音乐持续×3、H 音乐退出后×3，共 26 周期。逐周期记录：系统状态、`target_q`、`target_d`、`target_torque`、`speed_pid.target`、PID_Reset / PositionLoop_Reset / Trajectory_Clear / Feedforward / PositionRun / MusicTask 计数、电流环/SVPWM 使能，以及桩调用顺序串（用于定位 early return）。

结果：**动态验证通过**。baseline 与当前输出 **34/34 行完全一致**，两份结果文件 SHA256 相同（`1612D4C530D2A6B73C14FD2E4EA5A5C5EF3D44A6B9E64BB0DCBBA41B3C27C6A9`）。

关键观察（两版一致）：

- A/B/C：状态分别为 1/5/0；每周期 `ForceSafeStop` + Task else 清理，事件 `SPOTSPOT`（四句复位各发生两次：状态更新一次、Task else 一次）。
- D 力矩：状态 3；首周期 `BEC`（电流环整定→SVPWM 使能→电流环使能），后续无外环复位；`target_q/d`、`target_torque` 固定。
- E 位置：状态 3；每周期 `ptSsF`（位置环→轨迹→速度环→前馈）；`target_q=0x3F4CCCCD(0.8)`、`speed_pid.target=0x42280000(42.0)`；无复位。
- F 音乐首次：状态 4；事件 `BSPOTm` = 切音乐带宽 + `ResetControlHistory` 一次 + `Music_Task1ms`；复位计数 +1。
- G 音乐持续：状态 4；仅事件 `m`，**无复位**。
- H 退出后：状态 3；首周期 `BptSsF`（切回控制带宽 `B`，**不触发复位**），后续 `ptSsF`。

分类结论：

- **动态验证通过**：以上 8 场景的主机逐周期可观察量与副作用顺序等价。A/B 为 Stage 0 基线 vs Stage 5 后实现；Stage 5 为纯内部 helper 拆分，若引入行为/顺序回归必在差异中体现。
- **静态核对通过**：helper 展开逐语句对照（TORQUE 完全相同、POSITION 仅 Stage 1 删除项、POT 等价）、Debug 构建、手动 `MOTOR_TORQUE_USE_POT=0` 编译。
- **未验证**：实机运行与中断时序/ADC-DMA；跨模块数值（本次仅编译 `motor_system.c`，其余模块以确定性桩替代）；无 Stage 4 独立快照，故未做“Stage 4→Stage 5”单步对比，改以计划允许的 Stage 0 基线 A/B 覆盖。

## Stage 6 — 自适应测速调度迁入速度模块（最高风险）

修改文件（仅 3 个）：

- `user/src/motor_system.c`
  - 删除 `static Motor_SpeedEstimator_GetPeriodTicks()` 与 `static Motor_UpdateSpeedEstimatorAdaptive()`。
  - fresh 分支替换为：`g_motor_system.run_data.speed_rpm = Motor_SpeedEstimator_UpdateAdaptive(g_motor_system.run_data.speed_rpm);`；stale 分支保持只写 0、不调用、不推进。
  - `Motor_AbsFloat` 保留（电位器映射仍使用）。
- `user/src/motor_speed_loop.c`
  - 新增 `#include "motor_config.h"`、`#include "as5600.h"`。
  - 新增 `static Motor_SpeedEstimator_GetPeriodTicks()`：阈值、比较与顺序原样，绝对值保留原 `(rpm < 0.0f) ? -rpm : rpm`。
  - 新增公开 `float Motor_SpeedEstimator_UpdateAdaptive(float published_speed_rpm)`：`ticks`/`dt_acc` 仍为 function static，初始化/递增/累加/比较/复位顺序与迁移前一致；未到期返回传入的 `published_speed_rpm`。
- `user/inc/motor_speed_loop.h`：新增该公开接口声明。

静态检查：

- `motor_system.c` 中旧调度函数名、`ticks`、`dt_acc` 已无残留；`Motor_AbsFloat` 仍在且被使用（行 244）。
- `motor_speed_loop.c` 不含 `motor_system.h`、不访问 `g_motor_system`（grep 无命中）。
- 全仓库仅 3 处 `Motor_SpeedEstimator_UpdateAdaptive`：速度模块定义、系统 fresh 分支调用、头文件声明。

固件构建：

- `cmake --preset Debug` / `--build --preset Debug` 均退出码 0；无新增 warning（仅原有链接 RWX）。
- RAM 4264 B 不变；FLASH 80132 → 80148 B（+16 B）：调度逻辑从系统 TU 移入速度 TU，产生跨模块调用开销。

A/B 动态回放（`refactor_validation/stage6_replay/`）：

- 参考实现 `ref_impl.c`：逐字取自 `baseline/user/src/motor_system.c` 的 `Motor_AbsFloat` / `Motor_SpeedEstimator_GetPeriodTicks` / `Motor_UpdateSpeedEstimatorAdaptive`，仅把最后一个包装成可调用的 `Adaptive_Call`，函数体未改。
- 被测实现 `new_impl.c`：仅包装并直接调用实际 `user/src/motor_speed_loop.c` 中的公开 `Motor_SpeedEstimator_UpdateAdaptive`（不复制算法）。
- 同一 `driver.c` 喂完全相同时间线；角度只由被测实现内部按到期规则读取（驱动不预读），`AS5600_ReadRawAngle` 桩记录调用 tick 与次数。
- 覆盖：initialized=0 首次调用；正向跨零 `4094,4095,0,1`；反向跨零；静止量化抖动；|speed| 在 50/200/500 的略低/等于/略高；正/负速度；连续多周期 stale 后恢复；再次 `Motor_SpeedEstimator_Init()`；未到期时 `published=0` 而 estimator `speed_rpm != 0`。
- 逐周期记录：tick、published_speed_rpm、fresh/stale、是否读取、读取 tick、角度、返回速度、estimator 内部速度、重建的 estimator `ticks`/`dt_acc`、是否调用 Update、Update 的 dt（按实现累加规则重建）、累计读取次数；浮点一律按位输出。
- 结果：**A/B 动态回放通过**，90/90 行逐位一致；两份结果文件 SHA256 相同（`59DA8757F68808C38E30B3E1A5F8B311080EBD2358B3B6D0836D18F905CF45F8`）。实际 Update 调用 tick 与 AS5600 读取 tick/次数完全一致，dt 与返回值逐周期一致；未使用任何误差阈值。

分类结论：

- **A/B 动态回放通过**：参考与迁移后实现逐周期 tick、读取位置、dt、返回值、estimator 内部状态完全一致。
- **静态检查通过**：旧实现/私有状态已从系统模块移除；速度模块无系统依赖；`Motor_AbsFloat` 保留。
- **固件构建通过**：Debug configure/build 成功，无新增 warning；RAM/FLASH 变化已记录。
- **实机未验证**：中断时序与 ADC/DMA、实机测速与闭环响应未上电；`dt` 为按实现累加规则重建（主机工具链 lld-COFF 不支持链接期 `--wrap` 拦截 `Update`），但读取 tick、返回速度与 estimator 速度为实测且逐位一致。
