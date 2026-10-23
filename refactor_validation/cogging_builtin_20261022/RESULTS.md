# 固件内置齿槽表与触觉模式验证（更新至2026-10-23）

目标为 STM32G431CBUx；使用 Arm GNU/CMake 的 Debug 产物，不涉及实际下载或电机运行。

- 基线构建成功：Flash 107096 B，RAM 27792 B。
- 内置表及默认参数阶段构建成功：Flash 108236 B，RAM 27792 B。
- 全模式接入后 Debug 增量构建成功，本次编译无警告：Flash 108652 B，RAM 27792 B；
  程序区上限 124928 B，未增加静态 RAM。
- 三种位置触觉曲线调整后 Debug 增量构建成功，本次编译无警告：Flash 109840 B，RAM 27800 B，
  比全模式接入阶段增加 1188 B Flash、8 B RAM，程序区上限仍为 124928 B。
- 实际 ARM ELF 在 Unicorn 中执行，`verify_builtin.py` 的 62 项断言全部通过。
- 全模式接入阶段 `verify_modes.py` 的 614 项断言全部通过：覆盖 8 模式的 192 组补偿启闭/方向/速度矩阵、
  60 组原 FREE/SPEED 旧 ELF 对照、总限流、PI 余量与条件积分、主机力矩总输出斜率、
  补偿长期不累加到基础命令、电位器 ±0.2 A 总边界、渐入/切换/重启、特殊状态和 6 种标定会话。
  逐点定位阶段保留摩擦辅助，跟随真实目标进入正式采样后两项补偿均为零。
- 10月22日触觉阶段 `verify_modes.py` 的 845 项断言全部通过，连同齿槽的 62 项共 907 项。
  在原回归上新增弹簧静区、渐硬与双向恢复，档位迟滞、跨档/跨零、短角及长累计角阈值固定位置，
  软墙接触连续性、向外阻尼和向内退墙，20 A/s 总斜率、重复 START 与停机重新捕获中心。
  实际 ARM 回放曾复现阈值处静止后多跳档，保留失败证据；最终版本的 12 个对应方向/角度边界用例均通过。
- 10月23日逻辑复查修正连续机械角逐拍float累加及浮点档心在长时间运行后的分辨率损失，
  改为64位机械计数、整数相对位置和有界整数档内相位；相对角遥测在短临界区读取完整整数差。
  另将测速累计/历史改为uint32_t模计数并显式解码窗口短距离，消除长期同向运行的有符号溢出。
  本阶段未修改刚度、阻尼、补偿配置、电流限值、20/10/5 ms测速窗口及控制周期。
- 本轮Debug增量构建成功，未产生警告：Flash 110712 B（88.62%），RAM 27840 B（84.96%）。
  相比10月22日触觉阶段增加872 B Flash、40 B RAM；程序区上限仍为124928 B。
- 本轮 `verify_modes.py` 的913项断言通过；原845项场景迁移到可实现的整数编码器网格，新增68项。
  新增覆盖巨大正负绝对位置后的中心重捕获、±1计数及相对角遥测、死区/墙外恢复力增量，
  正反向1024圈解回绕后档内1计数响应，以及20/10/5 ms测速窗口跨INT32/UINT32边界。
  同一ELF的齿槽62项也全部通过，总计975项，无失败。
- 默认配置按用户反馈已测试的参数更新为：启用、增益 0.90、限幅 0.080 A、
  100～200 rpm 淡出、相位偏移 0；重新构建及 62 项离线断言均通过。
  用户的实测反馈与本次离线验证分别记录，本次未执行新的上板验证。
- 覆盖内置表加载、旧记录回退、匹配记录恢复禁用状态、保存来源标签、上传事务失败与成功、
  停机输出、500 ms 渐入、速度淡出、限幅及单圈环绕。Flash 物理接口与时钟为模拟边界。
- 当前 ELF 常量表位于 `0x0801A6D0`，占 1024 B；与候选 CSV 的 512 个协议量化值逐点一致，
  CRC32 为 `0x8706A889`，修订号为 10。
- HEX 与 ELF 的全部下载数据逐字节相同，地址范围为 `0x08000000` 至 `0x0801B078`（末址不含），
  不包含摩擦、齿槽或电机参数专用页。

产物：`build/Debug/g431_foc.elf`、`build/Debug/g431_foc_builtin_cogging.hex`。

ELF SHA256：`7eedb836b24728b23be84669f49f919198e03f86cf83e07d81e24ea0b041de12`。

HEX SHA256：`d72b8873723cda3c563809a19341b737b666425508ae460665e5ec5289e417f9`。

本地复核使用 Python 3.13、既有 Unicorn 2.1.4 和 pyelftools 0.33，未安装新环境。
运行验证脚本时需要可导入这两个依赖，并保留本地候选 CSV
`calibration_results/hybrid_512_20261022/HYBRID_UNVALIDATED_512.csv`。
详细断言与下载检查报告保留在 `.codex_tmp/builtin_lut_20261022/`，不加入源文件交付。
默认参数阶段的断言报告为该目录下的 `default_params_090_verification.json`。
全模式阶段的 `builtin_verification.json`、`mode_verification.json` 和 `download_manifest.json`
保留在 `.codex_tmp/all_modes_compensation_20261022/`，修改前源码及 ELF 在其 `baseline/` 子目录。
10月22日触觉阶段的 `builtin_verification_final.json`、`profile_verification_final.json` 和 `download_manifest.json`
在 `.codex_tmp/haptic_profiles_20261022/`，本阶段修改前源码与 ELF 在其 `baseline/` 子目录。
本轮逻辑复查的 `builtin_verification.json`、`mode_verification.json` 和 `download_manifest.json`
在 `.codex_tmp/haptic_logic_audit_20261023/`，本轮修改前源码与ELF在其`baseline/`子目录。
默认基础刚度 Spring/Detent/Limit 为 0.12/0.40/0.80 A/rad；三模式响应斜率由 1 改为 20 A/s，
保留原最大电流及已确认的齿槽补偿配置。以上参数与非线性曲线仍待实机验证。
复核最新全部模式与触觉轨迹时运行：

```powershell
python refactor_validation/cogging_builtin_20261022/verify_modes.py --elf build/Debug/g431_foc.elf --baseline .codex_tmp/all_modes_compensation_20261022/baseline/build/Debug/g431_foc.elf --report .codex_tmp/haptic_logic_audit_20261023/mode_verification.json
```

此前 pyOCD 枚举结果为没有可用调试探针，本轮也未烧录、未验证实际 Flash 操作、中断时序或补偿收益。
全部正常闭环模式已接入补偿，但仍遵循原使能开关；摩擦默认禁用，新模式须分别实测低速与反向稳定性。
TORQUE 最终 Iq 增加补偿，命令和目标力矩遥测仍表示基础给定；协议字段和 Flash 布局未改。
SPRING/DETENT/LIMIT 模型按本轮授权调整；FREE/DAMPING 模型、补偿参数、硬件初始化和控制周期未改。
索引 0～4 仍是估算点，不能因编译或模拟通过改称完整实测合格表。Keil 工程未单独构建，
其既有链接配置未调整；本次可下载产物来自已检查程序区边界的 GNU 构建。
