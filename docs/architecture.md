# 当前有感固件架构

## 应用入口

`Core/Src/main.c` 保留 CubeMX 初始化和前台任务。
`firmware/app/motor_system.c` 负责系统初始化入口、TIM2 的 1 ms 调度和状态灯。
TIM2 保持原次序：参数辨识调度 → 生命周期状态更新 → 周期策略 → 状态灯。

| 文件 | 职责 |
|---|---|
| `app/motor_system.c` | 初始化和定时调度入口 |
| `app/motor_control.c` | 运行请求、故障锁存、模式切换、统一停机、音乐入口 |
| `app/motor_host.c` | 主机控制权、心跳、命令校验、给定与配置、诊断读取 |
| `app/motor_calibration_session.c` | 标定启动校验、配置保存和停机恢复 |
| `app/motor_runtime.c` | 反馈更新、标定进度、强拖、音乐和模式执行次序 |
| `app/motor_modes.c` | 力矩、位置、速度计算和电位器滤波 |
| `app/motor_haptic.c` | 自由、阻尼、弹簧、齿槽感、限位模式的 Iq 合成 |
| `compensation/motor_cogging.c` | 补偿表事务、CRC、插值、淡出、运行渐入和 Flash 记录 |
| `compensation/motor_friction.c` | 摩擦模型、配置校验和自由模式补偿模型 |
| `compensation/motor_friction_storage.c` | 摩擦配置 Flash 记录 |
| `communication/pc_protocol.c` | 协议解包、令牌与长度检查、调用命令接口、应答序列化 |

表中路径均相对于 `firmware/`。摩擦函数保留原 `motor_feedforward_*` 名称，头文件为
`motor_friction.h`；调用方和测试已经迁移。

## 数据和依赖边界

公开状态类型在 `app/motor_types.h`，生命周期命令在 `app/motor_control.h`，
主机命令与读取接口在 `app/motor_host.h`。`motor_system.h` 汇集这些声明，保留既有调用方的源代码兼容。
命令参数与状态返回沿用基础标量，协议表片段和诊断读取保留原缓冲区／快照边界。

应用内部的跨模块状态集中在 `app/private/motor_context.c/.h`，保留原变量类型和初始值。
仅应用文件包含该内部头；协议、控制算法和设备驱动通过公开接口访问应用。
应答缓存由主机模块独占；解回绕历史由周期模块独占；电位器滤波量由模式模块独占；
标定原配置由标定会话独占。前台与 ISR 的共享访问、原临界区和读取次序保持原样。

这次拆分保留了应用策略之间必要的共享状态，没有新增快照原子性、并发任务或动态分配。
补偿模块独立维护模型和配置；模式计算负责根据运行模式、方向、限流和标定阶段决定如何叠加补偿。

## 有感控制配置

本工程固定使用 MT6826S。已删除 `FOC_PROFILE_SENSORLESS`、`BOARD_SENSORED_CONTROL_ENABLE`、
`BOARD_AS5600_REFERENCE_ENABLE` 及相关条件分支，删除无感观测器和 AS5600 参考角代码。
主循环和板级初始化中原有的空参考角调用也已移除。
CubeMX 外设配置、引脚、控制周期、控制公式、限幅、协议命令值和 Flash 分区保持原值。

CMake 和 Keil 工程按新目录更新。Keil 的非活动 OLED／调试源码保留在导航中并设为不参与构建，
与当前 CMake 活动固件源集合一致；本次实际编译使用 Arm GNU Toolchain。
