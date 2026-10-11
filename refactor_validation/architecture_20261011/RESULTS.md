# 有感架构重构验证

日期：2026-10-11。直接基线：`2f6fb3d022037055b2bc99174480761adcf9c91a`。

## 改动

- 自定义固件迁入 `firmware` 的职责目录；PC 分析脚本迁入 `host/tools`。
- `motor_system.c` 从 1710 行缩为 51 行，负责初始化入口和 TIM2 调度。
- 生命周期、主机命令、标定会话、运行周期、闭环模式、触觉模式分开实现。
- 摩擦模型文件改名为 `motor_friction.c/.h`，原 `motor_feedforward_*` 函数名称和行为保留。
- 齿槽模型和摩擦 Flash 存储各自独立。
- 应用内部共享状态集中在 `app/private`；协议和设备模块不包含内部头文件。
- 删除有感／无感配置宏、相关 `#if/#else/#endif`、无感构建 preset、无感观测器、
  AS5600 驱动和参考角空调用。固定采用原有感分支。
- 更新 CMake、Keil 源码导航与依赖、使用文档和现有标定测试路径。

## 已执行

| 检查 | 结果 |
|---|---|
| Arm GNU Toolchain 11.2.1，Debug 有感完整清理重建 | 68 步，通过，无编译警告 |
| 全部 31 个项目 C 单元，包括未链接的 OLED／调试代码 | `-Werror` 编译通过 |
| 29 个独立头文件 | `-Werror` 语法检查通过；字体数据头除外 |
| 原系统的 63 个函数体 | 原有感分支的有效语句一致；触觉块重新展开后逐 token 对照 |
| 22 个迁移 C 单元 | 分配节、数据、重定位、公开符号与直接基线目标文件一致 |
| 板级初始化 | 原有感语句与顺序一致，仅移除原来为空的参考角初始化调用 |
| 命令、协议序列化和实际应答 | 3338 个 ARM 模拟用例通过；45 项状态观察一致 |
| 八种模式、输入来源、方向、角度回绕、目标换向、心跳、故障、强拖 | 8132 个 ARM 控制／状态步骤与直接基线一致 |
| 有效标定会话和停机配置恢复 | 8 次会话、104 项检查通过，包含模式 0/1/4/5 和两个方向 |
| 目录引用、Keil XML、`git diff --check` | 检查通过 |

命令模拟沿用此前的 `af1212e` 比较夹具，仅执行当前仍存在的有感接口。
运行周期和有效标定会话直接比较本次重构前的 `2f6fb3d` 固件。
无感专属 188 项旧用例不再适用于当前工程。

## 资源

实际 CMake 有感固件使用 RAM **27768 B**、Flash **105452 B**。
直接基线为 RAM 27752 B、Flash 105448 B，分别增加 **16 B** 和 **4 B**。
验证专用 ELF 强制保留原本会被链接裁剪的命令入口，其资源数值不代表实际固件。
没有动态分配、额外周期任务或新的补偿数据副本。

## 验证边界

Unicorn 执行实际 Cortex-M4 代码。时间、原始传感器输入、参数就绪状态、物理输出和
部分 Flash 保存边界使用确定性桩。它能检查模型中的返回值、状态演化、PI 与补偿输出，
不能确认异步抢占、实际 ISR 执行耗时、串口链路和电机物理响应。
未执行 Keil 编译、烧录或电机实测。新增跨文件函数调用和共享符号的布局变化须在硬件上确认时序裕量。

协议帧布局、状态数值、控制系数、限幅、PWM 配置及 Flash 记录布局保留；协议中原有的
`__DATE__ / __TIME__` 编译信息自然随重建更新。目标文件对照固定 `SOURCE_DATE_EPOCH=0`，
避免编译时间字符串造成误报。

## 文件和重跑

`migration.json` 记录目录映射、内部入口改名和删除文件。
`source_equivalence.json`、`legacy_command_equivalence.json`、`runtime_equivalence.json`、
`calibration_session_equivalence.json` 是重跑时生成的本地检查结果，已在 Git 中忽略。

源码和 ARM 基线夹具在忽略目录 `.codex_tmp/architecture_validation/baseline` 与 `baseline.elf`；
旧命令夹具和 Unicorn 依赖在 `.codex_tmp/style_validation`。这些本机生成文件不进入 Git。
本机重跑顺序：

```powershell
python refactor_validation/architecture_20261011/verify_architecture.py
python refactor_validation/architecture_20261011/build_command_elf.py
python refactor_validation/style_20261011/verify_commands.py --sensored-only `
  --baseline .codex_tmp/style_validation/commands_baseline.elf `
  --current .codex_tmp/architecture_validation/current.elf `
  --report refactor_validation/architecture_20261011/legacy_command_equivalence.json
python refactor_validation/architecture_20261011/verify_runtime.py
python refactor_validation/architecture_20261011/verify_calibration_sessions.py
```

先用实际 Arm 工具链配置和构建 `.codex_tmp/architecture_validation/build`。
旧 `style_20261011` 的目录迁移前源码／目标文件比较工具与报告属于此前阶段；
当前架构使用本目录的验证工具。
