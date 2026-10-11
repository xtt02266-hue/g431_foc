# G431 FOC 规范迁移与回归结果

## 最终接口迁移（2026-10-11）

本节是最终结果。下文“第一阶段历史记录”仅记录此前格式整理，不能作为最终接口或固件全字节一致的结论。

- 自有公开函数改为 snake_case，结构体/枚举类型改为 `_t`；同步工程调用及相关标定测试名称。
- 电机命令返回 `uint8_t` 状态码，保留 `MOTOR_CMD_*` 的 0～10 数值。模式输入使用与原 ARM 枚举宽度及协议字节一致的 `uint8_t`，校验后才写入内部枚举。
- 触觉、摩擦、齿槽、电流采样、辨识参数加载及观测器配置命令使用标量参数；结构体用于内部状态与校验。
- Iq、速度和事务开始命令移除输出指针，成功后通过独立标量接口读取结果。Iq/rpm 的已接受值由前台保存、ISR 不覆盖，保持停机抢占时原应答值的含义。
- 音乐控制使用曲目 ID。独立数据接口登记任意只读音符缓冲区，保留有限播放、循环、休止符、时长及 DDS 行为，不复制曲目。
- 必要的数据接口保留指针并注明用途：协议表片段、采样队列、配置读取和诊断快照；沿用原同步方式。HAL 回调保留规定签名。
- OLED 内部辅助函数改为 static。`Core/Src/main.c` 仅在 USER CODE 区迁移四个调用名称。

**兼容性：** C 源码接口已改变，外部调用方需按 `public_renames.json` 和当前头文件迁移。
协议命令、字段和错误码、Flash 记录格式、控制公式、参数、限幅、滤波、PWM 及外设配置未改变。
旧配置指针的 NULL 请求不再是标量 API 可表达的输入；数值非法仍按原规则拒绝。

自定义曲目在前台先调用 `motor_songs_register_score(song, note_count)`，再调用
`motor_system_play_song(MOTOR_SONG_CUSTOM, play_count)`；曲目内存须保持有效直至播放结束。

### 最终验证

| 检查 | 结果 |
|---|---|
| 有感 Debug 清理后完整重建 | 62 个步骤通过，无警告 |
| 27 个自有 C 文件 × 两种配置 | 54 组均编译通过，无警告 |
| 29 个接口头独立包含 | `-Werror -fsyntax-only` 全部通过 |
| 对象代码差异范围 | 646 组函数/配置的机器码及重定位一致，其他差异仅在列出的接口迁移点 |
| 新旧 ARM 镜像仿真执行 | 3526 个场景通过，返回值、46 项状态观察及协议应答一致 |
| 格式/命名/差异检查 | clang-format、UTF-8 无 BOM/无 Tab、公开函数和类型命名、`git diff --check` 通过 |

覆盖模式字节、权限/停机优先级、NaN/Inf 及配置边界、保存成功/失败及忙状态、观测器配置、辨识参数加载、曲目序列与 DDS、实际协议分派及应答序列化。
补测了已接受 Iq/rpm 在模拟停机抢占后的应答一致性，以及事务号回绕。
Flash 保存结果和硬件时间使用确定性边界桩；只排除 C 标准不规定的配置结构体填充字节，实际成员逐字节比较。

最终固件 RAM **27752 B**（比原始基线增加 **8 B**），Flash **105448 B**（增加 **948 B**）。
电流环、ADC ISR、1 ms 周期控制及相关控制计算函数的对象代码/重定位一致，新增开销位于前台命令和曲目启动接口。

`control_code_equivalence.json` 验证差异范围；`command_equivalence.json` 保存最终仿真结果。
`interfaces_units_comparison.json` 是原始完整对象差异报告，其整体等价字段为 false，因为接口已迁移；编译和头文件检查均通过。
`names_equivalence.json` / `names_units_equivalence.json` 属于纯公开名称迁移阶段，其他旧等价报告属于第一阶段。

### 截图中的辅助脚本

`.codex_tmp/style_validation/` 保存一次性迁移工具、原始基线、对象、构建目录及仿真依赖，已在 Git 忽略范围内；迁移脚本不能盲目重跑。
`refactor_validation/style_20261011/` 保存可复查的验证工具、映射及报告。
这些 Python 文件都不编译进 STM32 固件。Unicorn 2.1.4 仅安装在临时目录，没有改动系统 Python 环境或固件依赖。

最终复查先执行 `verify_units.py` 生成对象（全对象一致性返回非零是已记录的接口差异），
再执行 `verify_control_code.py`、`build_command_elfs.py`、`verify_commands.py`；仿真输入是临时目录的 `commands_baseline.elf` / `commands_current.elf`。

未烧录、未启动电机、未进行实机时序/外设验收；未执行 Keil 或历史 Zig 回放。
代码对比和仿真支持已覆盖场景的行为保持结论，不能保证所有异步中断组合或硬件行为已验证。

---

## 第一阶段历史记录（接口尚未迁移时）

日期：2026-10-11。目标工程：`D:\WorkDocument\g431_foc`，分支 `codex/sensored`。
开始时工作区干净，Git 基线为 `af1212e`。原始自有源码和基线 ELF 保存在
`.codex_tmp/style_validation/`，未覆盖原有 `refactor_validation/baseline/` 历史资料。

本轮执行用户要求的“功能完全不变先”：对项目自有代码做行为保持整理。
依据为 `D:\WorkDocument\gimbal\19.gimbal_task\.agents\skills\embedded-c-development\SKILL.md`
及其更新后的 `references/embedded-c-guidelines.md`。

## 修改范围

- `user/src/` 的 27 个 C 文件与相关 `user/inc/` 头文件：统一四空格、分支大括号、函数括号、运算符空格和换行。
- 348 个项目函数均具备用途注释；补充头文件的参数单位、失败结果、调用上下文和缓冲区约束。
- 补充控制/标定/通信宏的用途及单位，记录 ISR、前台、DMA 和共享状态的原有访问方式。
- 59 个私有函数/私有类型标识符规范化；对应映射见 `private_renames.json`。
- 修正与源码不一致的注释，包括异步角度采样、10 mA 自由模式阻尼边界、音乐停播时正常目标透传，以及现有本地模式/给定来源条件。
- `oled.h` 增加独立需要的 `<stdint.h>`；两个头文件保护宏移除保留标识符双下划线。
- 两个编码器验证脚本同步更新私有函数提取名称，保持原测试场景。
- 新增根目录 `.clang-format`，仅应用到自有 C/头文件；字体资产保持原样。

CubeMX 生成代码、厂商驱动、启动文件、链接脚本、工程配置与 `.ioc` 无变更。
控制公式、数值常量、条件判定顺序、状态机、临界区、DMA 发布次序、协议字段与 Flash 记录布局保持基线实现。

## 本轮兼容性范围

既有公开函数名称、类型、参数和结构体字段顺序保持兼容。这包含当前电机命令接口中的
枚举返回/参数和配置结构体指针，以及算法结构体返回接口。
因此，本轮没有将这些既有接口全部迁移为标量，也没有宣称整个历史工程已完全满足所有目标接口规则。
后续接口迁移需要单独检查调用方、非法值/空指针语义、错误码、协议应答及执行时间；不能把它混入本轮已证明等价的改动。
新增命令接口仍应采用规范要求的基础标量形式。

## 已执行验证

验证工具：Arm GNU GCC 11.2.1、CMake 4.3.3、Ninja。
采用当前机器可用工具链，新建独立验证构建目录，未沿用记录中的失效工具路径。

| 验证 | 结果 |
|---|---|
| 修改前有感 Debug 完整构建 | 通过，无警告 |
| 最终有感 Debug 清理后完整重建 | 62 个构建步骤通过，无警告 |
| 最终 ELF 的 12 个运行区段对比 | 地址、大小、标志、对齐和装载字节一致，例外仅为下述编译时间串 |
| 27 个自有 C 文件 × 两种开发配置 | 54 组编译前后对比全部通过，无警告 |
| 对象文件对比范围 | 代码/数据区、零初始化 RAM 区、重定位目标、公开符号及其大小均一致；私有名称按明确映射还原 |
| 29 个头文件独立包含 | `-Werror -fsyntax-only` 全部通过；字体资产不作为接口头检查 |
| 格式与差异检查 | clang-format `--dry-run --Werror`、`git diff --check` 通过；无 Tab、无 UTF-8 BOM |

RAM 为 **27744 B**，Flash 为 **104500 B**，与修改前一致。
摩擦、齿槽和电机参数专属区布局未变。

完整固件的 `.rodata` 包含协议原有的 `__DATE__ " " __TIME__` 构建信息。
编译时间自然变化，因此对比时仅明确归一化该唯一 21 字节字符串，不忽略任何指令或其他常量。
对象文件对比使用 `SOURCE_DATE_EPOCH=0` 固定日期与时间，代码/数据与重定位逐项完全一致。
详细结果见 `sensored_equivalence.json`、`units_equivalence.json`。

## 复核命令

在项目根目录执行，`python` 应指向可用的 Python 3：

```powershell
cmake --build .codex_tmp/style_validation/build --clean-first
python refactor_validation/style_20261011/verify_equivalence.py .codex_tmp/style_validation/baseline.elf .codex_tmp/style_validation/build/g431_foc.elf --allow-build-timestamp
python refactor_validation/style_20261011/verify_units.py --baseline .codex_tmp/style_validation/baseline --build-dir .codex_tmp/style_validation/build --renames refactor_validation/style_20261011/private_renames.json --report .codex_tmp/style_validation/recheck.json
```

这些命令依赖仍保留的本轮原始源码、基线 ELF 与当前机器工具链。
私有函数名称已改变；如使用调试器按名称观察或设置断点，需要使用映射后的名称。

## 尚未执行

未使用 Keil 构建，未烧录、未启动电机、未执行连接硬件的验收。
本轮没有运行既有 Zig 主机回放：当前环境未找到 Zig/native C 运行工具链。
行为保持结论依据同一 Arm 工具链的机器码、数据、重定位和布局对比，不能描述为实机验证。
源码中独立于风格的既有功能/同步问题不在本轮修复范围内。
