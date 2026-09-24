# HapticKnobStudio 上下位机适配改动方案

版本：方案 v1.0  
日期：2026-09-15  
上位机：`D:/WorkDocument/HapticKnobStudio`  
下位机：`D:/WorkDocument/g431_foc`（STM32G431）  
性质：待实施设计文档。本文描述拟议改动，不表示功能已经实现或通过真机测试。

## 1. 目标与实施边界

将现有上位机原型改造成当前 G431 项目的专用调试工具，实现以下闭环：

1. 上位机正确显示固件遥测，通道名称、单位、状态与实际固件一致。
2. 下位机接收并校验上位机命令，允许选择位置、力矩和触觉工作模式。
3. 模式选择、启动、停止、参数修改均有明确应答及实际状态回读。
4. 上位机给定与电位器给定有明确控制权，不互相覆盖。
5. 停机、故障、失联时统一撤销输出；重连不能恢复旧转矩。
6. 保持现有 20 kHz 电流环架构，串口协议及 UI 不进入快速控制中断。

第一阶段实现位置模式（电位器目标）、力矩模式（电位器或上位机 Iq）、Free；第二阶段加入 Damping、Spring、Detent、Limit。只有完成验收的模式才能通过能力位向 PC 开放。

本次不新增上位机速度闭环模式、上位机位置目标轨迹、无感闭环切换、在线固件升级、任意内存访问或任意 Flash 写入。音乐和参数辨识继续作为独占流程处理，第一版不新增远程启动音乐及辨识命令。PID 在线整定可后续扩展，不能让当前方案依赖它才能交付。

## 2. 当前代码基线与必须纠正的问题

以下结论来自本地工作目录，而非上位机 README 所描述的历史版本。实施时保留现有未提交改动，不通过重置仓库恢复旧代码。

### 2.1 遥测现状

USART2 配置为 921600、8N1。`user/inc/vofa_usart.h` 定义 `VOFA_TX_MAX_FLOAT_COUNT=7U`；`user/src/motor_debug.c` 实际发送以下 7 路：

| CH | 字段 | 单位 | 固件来源 |
|---|---|---|---|
| 0 | 滤波后电位器给定 | count | `Motor_System_GetDebugPotTarget()` |
| 1 | 实际进入电流 PI 的目标 Iq | A | `g_foc_state.pi_q.target` |
| 2 | 实测 Iq | A | `g_foc_state.park.q` |
| 3 | 实测 Id | A | `g_foc_state.park.d` |
| 4 | 机械转速 | rpm | `g_motor_system.run_data.speed_rpm` |
| 5 | q 轴 PI 输出 | V | `g_foc_state.pi_q.output` |
| 6 | 电机状态 | enum | `g_motor_system.state` |

一帧为 `7 × float32 LE + 00 00 80 7F`，共 32 字节。当前发送函数使用 5 ms 门限，实际周期受前台任务影响。旧注释中出现的 2 ms 不作为设计依据。

上位机固定 20 路、84 字节，且以 mA/旧索引解释数据。因此需要同时修改解析器、通道、状态栏、绘图默认项、CSV 和文档，不能只改帧长。

### 2.2 控制现状

- `MotorControlMode` 已有 POSITION=0、TORQUE=1，默认 TORQUE。
- `MOTOR_TORQUE_USE_POT=1`，当前手动 Iq 接口默认拒绝上位机给定。
- 固件已有系统级模式、Iq、N·m、启停与故障接口，应在其基础上扩展。
- `g_run_requested` 初始化为 1；参数缺失时存在自动辨识路径。
- 串口 RX 仍为 TODO，当前没有接入上位机命令。
- 上位机 `firmware_adapter` 直接写 `g_foc_state.target_q`，限流和状态管理另起一套，不宜原样合入。
- 原接入说明在速度计算后提前返回，会绕过后续音乐和正常控制状态处理，应废弃该接入片段。
- PC 限流输入上限 5 A、适配层 3 A、现有力矩给定上限 1.5 A，相互不一致。

## 3. 总体架构

```text
PC UI：输入意图、显示已生效状态、绘图与记录
  ↓ 命令对象                         ↑ ACK / STATUS / TELEMETRY
PC 通信会话：序号、待确认命令、超时、心跳
  ↓ 帧编码                           ↑ 帧解析
串口后台线程 ←→ USART2 DMA
  ↓ RX 环形缓冲
固件前台：帧解析、会话检查、命令入队、统一 TX 调度
  ↓ 已校验的控制命令
固件 1 kHz：命令提交 → 安全检查 → 状态机 → 模式计算 → 统一电流出口
  ↓ Id / Iq 目标
现有 20 kHz FOC → SVPWM
```

核心原则：只有系统控制层决定谁可以输出；各模式计算模块返回目标值，不独立启停 PWM、不绕过公共限流、不自行管理故障恢复。

## 4. 运行状态、工作模式与控制权

### 4.1 保留 MotorState 的现有编号

| 值 | 状态 | 含义 |
|---|---|---|
| 0 | STOPPED | 停止闭环输出 |
| 1 | WAIT_PARAMETERS | 等待有效电机参数 |
| 2 | IDENTIFYING | 参数辨识独占执行 |
| 3 | SENSORED_RUN | 有感正常控制 |
| 4 | MUSIC | 音乐独占执行 |
| 5 | FAULT | 故障锁存 |

模式被选中不等于正在运行。上位机必须同时显示实际模式与 MotorState。

### 4.2 扩展模式，保留已有 0/1 编号

| 值 | 模式 | 给定来源 | 行为 |
|---|---|---|---|
| 0 | POSITION | POT | 保留现有电位器→轨迹→位置/速度环→Iq 链 |
| 1 | TORQUE | POT 或 HOST | 电位器力矩映射或 PC 指定 Iq |
| 2 | FREE | INTERNAL | 闭环使能时 Id=0、Iq=0，不等同 STOP |
| 3 | DAMPING | INTERNAL | 根据速度生成反向阻尼 |
| 4 | SPRING | INTERNAL | 根据相对中心位置及速度生成恢复力 |
| 5 | DETENT | INTERNAL | 根据连续角度生成等间隔档位 |
| 6 | LIMIT | INTERNAL | 根据连续相对角度产生软限位力 |

给定来源枚举：POT=0、HOST=1、INTERNAL=2。仅允许表内组合；其他组合返回 `INVALID_COMBINATION`。不保留模糊的 Legacy 名称。旧协议的 mode=0（Legacy）不能直接解释为新协议的 POSITION，因此新协议使用不同版本和帧头。

### 4.3 控制权与给定来源分开

控制权 Owner：LOCAL=0、HOST=1。它表示谁可以改变运行配置，不表示目标来自哪里。HOST 持有控制权时仍可选择 TORQUE+POT，用于观察本地电位器行为；此时失联仍按 Host 会话规则停机。

- 默认 Owner=LOCAL。串口连接或 HELLO 查询不自动取得控制权。
- PC 点击“接管控制”，仅在 STOPPED 且无故障、无辨识/保存忙时获得 HOST 控制权。
- HOST 控制权期间，本地普通模式/启动操作返回占用；本地 STOP 始终有效。
- LOCAL 控制权时拒绝 PC 的 START、模式/参数写入；STOP 和只读查询仍有效。
- 释放控制权先停机并清零，保留所选模式用于显示；切回 LOCAL 后默认选择 TORQUE+POT，仍不自动启动。
- 当前编译宏改为默认来源配置，例如 `MOTOR_DEFAULT_INPUT_SOURCE=POT`；不能继续用条件编译删掉 HOST 分支。
- 已有本地控制接口保留调用形式；内部改走统一校验与提交路径，避免形成第二条可绕过控制权的路径。

### 4.4 启动与模式切换规则

第一版采用“停止后切换”，不实现带转矩热切换：

1. RUNNING 时收到模式切换请求，拒绝为 `MUST_STOP_FIRST`；不静默停机，也不自动重新启动。
2. STOPPED 时校验模式能力、来源组合、参数及传感器条件。
3. 在 1 kHz 控制提交点原子更新模式和来源，清零旧给定，复位外环、触觉及斜率限制历史。
4. 应答包含实际模式、来源与配置版本号。重复选择相同配置不得隐式改变中心或再次清零。
5. PC 单独发送 START。固件确认参数有效、编码器新鲜、无故障、HOST 会话有效后，启用已选模式。
6. TORQUE+HOST 启动输出为零，PC 必须等运行状态确认后重新发送 Iq。停止期间的非零 Iq 写入返回 `NOT_RUNNING`。
7. SPRING/DETENT/LIMIT 在每次成功 START 时以当时连续角度捕获中心；在一个运行周期内中心保持不变。
8. STOP、故障、失联后均不自动恢复运行；CLEAR_FAULT 不设置运行请求。

### 4.5 上电及辨识策略

新增明确的 Studio 调试构建配置：关闭自动运行请求和无参数自动辨识；上电有参数则 STOPPED，无参数则 WAIT_PARAMETERS。无参数时 UI 显示“需本地完成辨识”，第一版不远程自动触发。

原独立电位器演示行为作为显式本地配置保留，不能成为 Studio 默认。调试配置还需禁止辨识完成后的自动启动路径。不要只改 `g_run_requested` 而漏掉参数模块中的自动辨识。

STOP 的系统语义覆盖所有输出所有者。对现有辨识补充取消接口：撤销开环输出、停止后续辨识推进，丢弃未完成结果；保留上次已验证参数，无旧参数则 WAIT_PARAMETERS。不能仅调用闭环停机而允许下一次辨识任务重新写 PWM。已进入 Flash 保存的操作不宣称可中途撤回，应确保保存期间电机已停，结束后不自动运行。

## 5. 双向协议 v2

### 5.1 协议与兼容策略

保留 921600、8N1。新固件 Studio 配置默认使用双向 v2；上行遥测、状态和 ACK 全部封装，不与裸 JustFloat 混发。

上位机连接配置提供：`Studio v2`、`Legacy JustFloat 7`、`Legacy JustFloat 20`。旧固件配置仅观测，禁用控制按钮。由用户明确选择协议，禁止根据几个浮点数自动猜通道定义。

新固件可通过构建选项选择 `LEGACY_JUSTFLOAT_7` 供 VOFA 使用；该构建禁用 PC 控制命令。第一版不做运行时切换上行协议，防止切换边界丢失导致误解析。

### 5.2 帧格式

所有多字节整数和 float32 使用 little-endian；浮点采用 IEEE754，不发送 C 结构体原始内存。

| 字段 | 大小 | 说明 |
|---|---|---|
| SOF | 2 | 固定 `A5 5A` |
| Version | 1 | 固定 2 |
| Type | 1 | COMMAND=1、RESPONSE=2、STATUS=3、TELEMETRY=4 |
| Seq | 2 | 命令序号，应答原样回传；遥测与周期状态各自编号 |
| Length | 2 | Payload 长度，最大 256 |
| Payload | 0～256 | 类型对应内容 |
| CRC16 | 2 | 覆盖 Version 到 Payload，不含 SOF |

CRC16-CCITT-FALSE：poly=0x1021、init=0xFFFF、RefIn/RefOut=false、xorout=0，CRC 结果小端发送；标准测试字符串 `123456789` 的结果应为 0x29B1。最大帧长 266 字节。所有数据字段逐个显式序列化。

解析要求：允许任意拆包、粘包、前导噪声；长度超限直接拒绝；CRC 错误逐字节重新寻找帧头；半帧 100 ms 未补齐时丢弃候选帧；错误帧不执行命令、不刷新心跳。坏 CRC 不回 NACK，避免噪声触发应答风暴。

### 5.3 会话与命令公共结构

COMMAND Payload：`command_id:u8 + session_token:u32 + command_body`。无会话查询及 STOP 使用 token=0。HELLO 返回固件版本、能力、当前参数定义和设备标识；HELLO 本身不分配控制权。

CLAIM 成功后返回非零 token，由固件按启动期间单调递增的会话计数生成。token 用于防止当前串口会话的过期写入，不是安全认证。接管、释放、超时及软复位均清命令队列和重复应答缓存；PC 检测设备重启后重新握手，不沿用旧会话。MCU 重启后的 token 不承诺跨重启全局唯一，不把它当作抗恶意重放机制。

普通 RESPONSE Payload：`command_id:u8 + result:u8 + session_token:u32 + config_revision:u32 + response_body`。

结果码固定：OK=0、INVALID_LENGTH=1、INVALID_VALUE=2、UNSUPPORTED=3、NOT_OWNER=4、MUST_STOP_FIRST=5、NOT_READY=6、FAULT_ACTIVE=7、BUSY=8、NOT_RUNNING=9、INVALID_COMBINATION=10、SEQ_CONFLICT=11、QUEUE_FULL=12。

写入命令的 OK 表示已在控制边界提交，不只是收到了字节。START 应答仅在状态机实际进入 SENSORED_RUN 后返回 OK；失败则返回原因及当前 STATUS 快照。每次成功改变配置（不含心跳、读取、瞬时 Iq）递增 config_revision。

### 5.4 命令表

| ID | 命令 | Body / 返回内容 | 条件与语义 |
|---|---|---|---|
| 0x01 | HELLO | 空 / 设备能力信息 | 任意控制权，只读 |
| 0x02 | GET_STATUS | 空 / STATUS 内容 | 任意控制权，只读 |
| 0x03 | CLAIM_CONTROL | 空 / 新 session_token | 仅 STOPPED、无故障、无忙任务，已有 HOST 则 BUSY |
| 0x04 | RELEASE_CONTROL | 空 / STATUS | 当前 HOST；先停机清零再释放 |
| 0x05 | HEARTBEAT | 空 / STATUS | 当前 HOST 会话；更新存活时间 |
| 0x10 | START | 空 / STATUS | 当前 HOST，执行第 4.4 节检查 |
| 0x11 | STOP | 空 / STATUS | 无需控制权，清输出并阻止自动重启 |
| 0x12 | CLEAR_FAULT | 空 / STATUS | 当前 HOST 或 LOCAL 状态下 token=0；恢复条件满足才清故障，不启动 |
| 0x20 | SET_MODE | mode:u8、source:u8 / 实际 mode、source | 当前 HOST，仅 STOPPED |
| 0x21 | SET_IQ | iq_a:f32 / 实际接受 iq_a:f32 | 当前 HOST，TORQUE+HOST 且运行 |
| 0x22 | SET_IQ_LIMIT | limit_a:f32 / 实际 limit_a:f32 | 当前 HOST，仅 STOPPED |
| 0x23 | SET_HAPTIC_PARAMS | 见下文 / 生效参数 | 当前 HOST，仅 STOPPED |
| 0x24 | GET_PARAMS | 空 / 全部参数快照 | 任意控制权，只读 |

CLEAR_FAULT 在 LOCAL 下允许恢复但不能自动取得 HOST 权限，避免故障状态无法 CLAIM 导致无法恢复的死锁。任何会导致机械动作的命令都不享受该例外。

触觉参数 Body 固定 22 字节：`spring_k:f32 + damping_b:f32 + detent_k:f32 + limit_k:f32 + limit_half_range_deg:f32 + detent_count:u16`。一次验证完整参数集，通过后整体替换，不能逐字段生效。

HELLO 返回体固定字段顺序：`fw_major:u8, fw_minor:u8, fw_patch:u8, schema_id:u16(=1), device_uid:12 bytes, mode_mask:u32, feature_mask:u32, hard_iq_limit_a:f32, host_iq_limit_a:f32, max_payload:u16(=256), heartbeat_timeout_ms:u16(=500)`。mode_mask 对应模式编号；feature_mask bit0=Host控制、bit1=触觉参数、bit2=连续角度，其他位保留为0。未完成模式对应能力位必须为0。

GET_PARAMS 返回体：`mode:u8, source:u8, iq_limit_a:f32, iq_slew_a_per_s:f32, 22字节触觉参数`。模式/限流/触觉写入应答分别返回其命令体同结构的生效值。STATUS 类型的主动包与 RESPONSE 内嵌 STATUS 使用同一内容编码。

### 5.5 超时、重复与 STOP 优先级

- PC 普通命令一次仅保留一个待确认请求，ACK 超时 200 ms，最多重试 2 次，使用相同 token/Seq/内容；不自动换序号重做 START。
- HEARTBEAT 每 100 ms 一次，独立于普通请求发送；固件 500 ms 无有效心跳停机、清零、释放 HOST 并记录 HOST_TIMEOUT。普通遥测和其他命令不能替代心跳。
- CLAIM 成功时启动 500 ms 计时。已见过的重复 HEARTBEAT 可回相同应答，但不再次延长租期。
- 固件缓存最近 32 个命令的 token、Seq、完整命令内容和应答；相同请求重传仅重放结果；相同 token/Seq、不同内容返回 SEQ_CONFLICT。心跳需使用递增序号；保留其最近已接受序号以拒绝旧心跳续期。
- PC 在 16 位 Seq 用完前停止并重新建立会话，不在同一会话回绕复用。PC 普通命令和心跳共用序号分配器。
- STOP 不等普通队列，收到完整且校验通过的 STOP 后设置专用停止标志，下一控制周期优先执行。清除未提交运动命令，待处理请求返回 NOT_RUNNING 或 BUSY；释放 HOST 并使旧 token 失效。
- STOP 后 PC 丢弃尚未发送的运行命令、停止普通重试；再次运行必须重新接管。token=0 的重复 STOP 可直接重复安全停机并回答当前状态，无需使用运动命令缓存。
- 不能宣称串口 STOP 是硬件急停。软件受串口传输、前台调度和 CPU 健康状况限制，台架验证需测量实际延迟。

## 6. 遥测与状态定义

### 6.1 遥测 schema 1

TELEMETRY Payload：`schema_id:u16(=1) + tick_ms:u32 + sample_seq:u32 + 10 × f32`，共 50 字节，整帧 60 字节。采样时间来自 MCU，不用 PC 串口批次接收时间代替。

| 索引 | 字段 | 单位 |
|---|---|---|
| 0 | filtered_pot | count |
| 1 | iq_pi_target | A |
| 2 | iq_actual | A |
| 3 | id_actual | A |
| 4 | mechanical_speed | rpm，编码器机械方向 |
| 5 | uq_pi_output | V |
| 6 | mechanical_angle | rad，单圈 [0,2π) |
| 7 | continuous_angle | rad，展开的机械角度 |
| 8 | iq_applied_target | A，系统最终给定 |
| 9 | relative_center_angle | rad，触觉相对中心角 |

原 MotorState 从浮点通道移至 STATUS 中的整数状态。机械角度与转速保持同一个机械方向；电磁方向转换仅在控制算法出口执行。非触觉模式的 relative_center_angle 定义为 0。

默认采样/发送 200 Hz，UI 绘图 20 Hz，状态卡片 10 Hz。默认一秒遥测约 12 kB，明显低于 921600/8N1 的理论约 92.16 kB/s。第一版不开放任意高采样率设置。

tick_ms 按 u32 自然回绕；PC 按无符号差展开。sample_seq 每次采样递增，即使发送丢弃也递增，用于区分采样与传输丢失。设备重启或新连接建立新的记录会话。

### 6.2 STATUS 固定结构

按顺序发送：`tick_ms:u32, state:u8, mode:u8, source:u8, owner:u8, run_requested:u8, parameters_status:u8, angle_valid:u8, reserved:u8(=0), fault_bits:u32, stop_reason:u8, reserved2:3 bytes(=0), config_revision:u32, iq_limit_a:f32, rx_crc_errors:u32, rx_overflows:u32, tx_telemetry_drops:u32`。

fault_bits：bit0=ENCODER_STALE、bit1=PARAMETER_ERROR、bit2=INVALID_CONTROL_VALUE、bit3=ANGLE_AMBIGUOUS；保留其他位为0。已有故障锁存与这些位统一维护，不虚构硬件没有提供的故障检测。

stop_reason：NONE=0、USER_STOP=1、HOST_TIMEOUT=2、FAULT=3、RELEASE=4、BOOT=5。超时属于停止原因，不伪装成传感器硬件故障；故障清除后保留最后停止原因，直到下一次成功启动。

STATUS 默认 10 Hz，模式/运行/故障变化后优先补发最新快照。PC 不从按钮选中状态推断实际模式，也不从 Iq=0 推断已关 PWM。

当前无母线实测采样、温度来源时，UI 不显示虚构实时值。配置母线电压如需展示应标注“配置值”。

## 7. 固件具体改动

### 7.1 模块划分

- `pc_protocol.h/.c`：v2 编解码、CRC、帧解析、协议错误计数。
- `host_session.h/.c`：token、心跳、重复请求、控制命令队列、应答关联。
- `motor_haptic.h/.c`：连续角度、中心、触觉参数和 Iq 计算；不启停 PWM。
- `motor_system.h/.c`：统一模式/来源/控制权、状态许可、限流与命令提交。
- `vofa_usart.h/.c`：收发传输服务；保留 legacy 发送实现，v2 构建使用统一 TX 仲裁。
- `motor_debug.h/.c`：从 OLED 调试函数拆出遥测快照和调度，不让串口采样依赖显示刷新。

不要同时编入旧 `firmware_adapter/host_control.c` 和新的系统实现。上位机目录内适配代码改为指向固件正式实现的说明，避免维护两份行为不同的副本。

### 7.2 RX/TX 与并发

当前 RX 为 normal DMA。第一版保留模式，使用 256 字节 DMA 暂存和 2048 字节软件环形缓冲；回调只拷贝收到的数据、更新索引、重新启动 DMA，不运行控制算法，不执行 Flash/OLED。

主循环有界解析，每次最多处理 512 字节，再返回其他后台任务；解析出的控制请求进入 8 项固定队列，队列满返回 QUEUE_FULL，不静默覆盖。环形缓冲溢出丢弃未解析内容、重置同步并计数；不会执行残缺帧。DMA 重启窗口可能丢字节，由 CRC、命令重传与溢出/错误统计兜底；台架若出现持续丢包，再改 circular DMA，不把首版可靠性建立在“永不丢字节”假设上。

1 kHz 回调先处理 STOP/会话超时，再提交最多一个普通控制命令，再更新状态与运行所选模式。原参数辨识推进前也必须检查停止/取消标志，保证停止优先于独占流程。

TX 仅一个调度入口调用 HAL_UART_Transmit_DMA。至少配置 8 项应答队列和 2 个固定 DMA 帧缓冲，应答优先、最新 STATUS 次之、遥测最后。应答资源不足时不得执行新的有副作用请求；STOP 仍执行，尽力应答并可通过状态查询确认。DMA 完成前不得复用正在发送的内存。

CPU 只在短临界区复制队列索引/小配置快照；不在长时间关中断区域计算 CRC、三角函数或打包整帧。共享多字段参数使用命令提交和快照，不能仅靠 volatile。

遥测在 1 kHz 任务每 5 次制作一次系统快照，由前台编码发送；20 kHz 变量复制只做必要的短快照保护，不宣称不同采样域天然同一时刻。测量新增中断耗时，不能使 1 ms 任务超期或破坏 20 kHz FOC 调度。

### 7.3 公共控制接口

新增类型：`MotorInputSource`、`MotorControlOwner`、`MotorCommandResult`、`MotorControlSnapshot`、`MotorHapticParams`。

对外新增带返回结果的统一命令提交入口；协议层不直接操作 `g_foc_state`。已有 `Motor_System_SetControlMode`、`SetTorqueCurrent`、`SetTorqueNm` 保持源代码兼容，通过内部共享校验函数实现，忙或权限不符时返回拒绝。

模式分支只在 SENSORED_RUN 执行。音乐继续独占；IDENTIFYING 不进入正常模式计算。公共运行数据（最终 Iq、估算目标力矩、实际模式）由系统统一更新。

公共最终电流出口检查有限值、模式许可和限幅。POSITION 原计算结果也通过此出口；辨识和音乐保持各自现有幅值约束，不盲目套用普通闭环 Iq 斜率逻辑。

### 7.4 限流与参数规则

- 固件硬上限沿用当前配置 1.5 A，作为代码限制而非硬件额定安全能力声明。
- HOST 会话的默认 Iq_limit=1.10 A；接管时恢复默认值，PC 读取确认后可显式修改。
- `SET_IQ_LIMIT` 范围 0.02～固件硬上限，越界拒绝，不静默写成另一值；PC 控件范围由 HELLO 更新。
- `SET_IQ` 必须为有限值，超出当前会话限流则限幅，ACK 返回真正接受值。
- HOST 控制所有正常模式都受会话限流约束；LOCAL+POT 保留原电位器调试幅值配置，同时受硬上限约束。
- 普通运行 Iq 采用 1.0 A/s 默认变化率限制，并回读该配置；第一版斜率参数只读。STOP/故障/超时直接安全停止，不为“平滑”延迟关断。
- 触觉范围：Spring/Detent/Limit K 为 0～5 A/rad；Damping B 为 0～1 A/(rad/s)；Limit 半范围 5～180°；档位数 1～128。
- 默认触觉参数：Spring K=0.03、Damping B=0.002、Detent K=0.05、Limit K=0.05、半范围=90°、档位数=24。均先做有限值和边界检查。
- NaN/Inf 参数整条拒绝；运行计算产生非有限值时清零并锁存 INVALID_CONTROL_VALUE。
- PC 写入仅作用于 RAM，断电不保存；UI 明确显示“本次运行参数”。未来 Flash 保存需独立命令及停机条件。

### 7.5 触觉坐标和公式

AS5600 counts 转换成机械角度 θ。连续角度使用相邻有效样本的最短角差累计，每 1 ms 更新；方向与机械转速一致。`uvw_dir` 必须为 ±1，否则触觉 START 返回 NOT_READY，不默认为任意方向。

设机械方向电流等效量为 i_mech，最终 `Iq = uvw_dir × i_mech`。阻尼速度使用未经 uvw_dir 翻转的机械转速转换为 rad/s，避免重复反向。

- FREE：i_mech=0。
- DAMPING：i_mech=-Bω。
- SPRING：i_mech=-K(θ_cont-center)-Bω。
- DETENT：spacing=2π/N，nearest=center+round((θ_cont-center)/spacing)×spacing；i_mech=-K(θ_cont-nearest)-Bω。
- LIMIT：rel=θ_cont-center；超上限时 i_mech=-K(rel-half)-Bω，超下限时 i_mech=-K(rel+half)-Bω，区间内为0。

所有模式最终经过有限值检查、斜率限制和限流。Limit/Spring 不使用 WrapPi 压回单圈，跨 ±180°不允许恢复力翻转。软限位不能保证轴绝不越界，UI 使用“软限位”名称。

单圈传感器展开无法识别两次采样之间超过半圈的运动。若有效样本丢失超过既有 10 ms 新鲜度限制则停机；遇到恰好半圈等方向不确定跳变时锁存 ANGLE_AMBIGUOUS，禁止继续累加。重新启动前重新建立角度基准。不能把软件连续角度描述为断电绝对多圈位置。

### 7.6 初始化与工程集成

协议、会话、队列和模式数据先初始化，再启动串口接收与 TIM2。现有 `VOFA_Init()` 早于 `Motor_System_Init()`，需要调整或确保此期间命令不能执行。

主循环新增协议/发送后台任务，并保留 AS5600 恢复和参数后台任务。CMakeLists.txt 与 Keil 工程同时登记新增源文件、包含目录和构建宏，保持两条构建路径一致。

## 8. 上位机具体改动

### 8.1 通信层与会话层

- `protocol.py` 分离 v2 帧编解码、命令对象和旧 JustFloat 解析；所有命令长度和结果码集中定义。
- 新增 session/controller 层，负责 HELLO、能力、token、序号、心跳、命令确认和 UI 状态同步，窗口不再自行拼接 SET_MODE+START+IQ。
- 串口读写均在后台线程；发送使用队列，STOP 优先，写入失败更新连接状态。检查短写和异常，不能把一次 `write()` 返回等同 MCU 执行成功。
- close/open 每次使用独立的停止事件和连接代号，旧线程不得影响新会话。线程退出统一关闭串口、发出断开信号；退出超时不启动共享同一对象的新 RX 线程。
- 收到 telemetry 批量传给 GUI，状态卡片按定时器读取最新快照，不逐帧刷新所有控件。
- 断连/超时标记遥测过期、帧率归零、取消待确认命令、使控制按钮失效。自动刷新端口不自动启动或接管。

### 8.2 UI 工作流程

建议左侧为连接与控制区，右侧为波形区；控制区可滚动，避免小屏遮挡 STOP。

1. 连接：协议选择、串口、波特率、连接状态、固件版本、能力。
2. 控制：接管/释放、实际 Owner、模式和来源下拉框、独立“应用模式”、START、STOP、Clear Fault。
3. 参数：设定值与已生效值并列；待确认显示处理中，拒绝显示原因。
4. 状态：实际 MotorState、实际模式、故障位、最后停止原因、编码器有效性、数据更新时间。
5. 模式专属参数：TORQUE 显示 Iq；触觉模式显示对应参数。POSITION 首版明确显示“电位器目标”。

操作序列固定为：连接并握手 → 若需先 STOP → 接管 → 写入限流并确认 → 应用模式并确认 → START 并确认 → TORQUE+HOST 手动发送 Iq。

改变模式控件不发送命令；点击“应用模式”才写入。输入框保留用户草稿，但不得在重连、接管、启动或模式切换后自动重放草稿 Iq。运行期间模式/限流/触觉参数应用按钮禁用，并显示需先停止。

手动断开和关闭窗口，在持有 HOST 时先发送 STOP，异步等待至多 200 ms 后关闭；即使应答未返回也不能阻塞 GUI。最终兜底依靠固件 500 ms 心跳超时。串口断开的瞬间不显示“已安全停机”，只能显示连接丢失/状态未知。

### 8.3 波形、单位和记录

- 建立 channel schema 数据源，统一字段 key、名称、单位、索引、绘图组和 CSV 标题。
- 默认三组图：Iq目标/实测/Id（A）、机械转速（rpm）、q轴PI输出（V）；触觉时增加角度/中心相对角度。
- legacy 7/20 各自独立 schema；状态卡片仅访问存在的字段。20 路兼容配置保留旧单位，不混用新 schema。
- 实时缓存默认 60 秒（200 Hz 时12000点）；清空操作同时清 deque 和曲线对象。
- 暂停仅冻结画面，持续接收和记录；UI 文案注明该含义。
- 帧率按 PC 当前单调时钟统计最近1秒，1秒无数据时显示0；字节率亦有过期归零逻辑。
- 连续记录写入独立后台队列，不依赖画面缓存；导出缓存与持续记录是两个按钮。
- 记录 CSV 包含 PC接收时间、MCU时间、sample_seq、模式/状态及通道数据；另写同名 JSON 元数据记录固件版本、协议、schema、限流与初始参数。
- 模式/参数变化追加独立事件日志，不把连接时参数当作整个记录期间始终有效。
- 磁盘写入失败或队列满明确停止记录并报告，不静默丢弃后仍声称完整记录。
- v2 CRC 合格但含 NaN/Inf 的遥测标记异常；图上用断点表示，不转成0。旧 JustFloat 没有 CRC，UI 明确其只读兼容与完整性限制。

## 9. 实施顺序与阶段交付

| 阶段 | 主要交付 | 通过条件 |
|---|---|---|
| P0 基线观测 | 7路兼容、schema统一、状态与单位修正、掉线/清空修复 | 当前旧固件可稳定看波形和导出 |
| P1 协议基础 | v2编码/解析、HELLO、STATUS、遥测、TX仲裁、只读连接 | 双向拆包/粘包/CRC测试通过 |
| P2 控制闭环 | 接管、STOP/START、POSITION/TORQUE/FREE、运行时来源、ACK、心跳、统一限流 | PC能切换实际模式，断连/故障不恢复旧转矩 |
| P3 触觉模式 | 连续角度、Damping/Spring/Detent/Limit、方向统一 | 两种uvw_dir与跨圈台架测试通过 |
| P4 调试体验 | 分组波形、持续记录、事件日志、打包与文档 | 长时记录和打包环境运行通过 |

P1/P2 新协议支持必须上下位机成对提交和验证；不能先发布能发送运动命令但无确认/心跳的新 UI。P3 未完成前不开放对应能力位及按钮。每阶段保留可复现构建和协议测试数据。

## 10. 测试与验收矩阵

### 10.1 无硬件测试

| 分类 | 测试输入 | 预期 |
|---|---|---|
| CRC | 标准字符串、Python/C同一命令 | 0x29B1及完全一致的帧字节 |
| 帧解析 | 在每个字节位置拆包；多帧粘包 | 帧完整解析且不重复执行 |
| 错误恢复 | 噪声、错CRC、超长Length、半帧超时后合法帧 | 不执行坏命令，能恢复同步 |
| Legacy | 7路32字节与20路84字节数据 | 只在选中对应profile解析，索引/单位正确 |
| 权限 | LOCAL写模式、过期token、无控制权START | 返回明确拒绝，输出不改变 |
| 模式 | 全部合法/非法模式来源组合 | 与第4.2节一致 |
| 确认 | ACK丢失、重复SET_MODE、Seq冲突 | 不重复副作用，不重新捕获中心 |
| 顺序 | START排队后收到STOP，再到达旧SET_IQ | STOP优先，旧会话不能重新出力 |
| 故障恢复 | FAULT且无法CLAIM时执行CLEAR_FAULT | 条件满足可清故障，仍不自动运行 |
| 参数 | NaN/Inf/负限流/越界档位 | 原子拒绝或按既定Iq限幅规则回读 |
| 时间 | tick回绕、重复时间、重连、重启 | 时间轴与日志会话不串联错误 |
| GUI | RX异常、清空、暂停、无数据 | 连接与图形状态正确，GUI不卡死 |
| 记录 | 磁盘错误、队列满 | 明确告警，停止记录，无完整性误报 |

建议为 C 协议和纯触觉计算提供主机侧测试桩，与 Python 共用 golden vectors。测试不可依赖电机实际转动。

### 10.2 固件构建与实时性

- CMake 与 Keil 各完成一次构建，检查新增告警、Flash/RAM占用与栈余量。
- 静态检查：UART回调无Flash/OLED/控制公式；命令层不直接改PWM；共享配置有提交边界。
- RX洪泛、TX繁忙、应答队列满时测量1 kHz/20 kHz执行时间和控制周期抖动，记录实际最大值。
- 明确验证 STOP 解析完成到控制撤销的调度目标为下一个1 ms控制周期；端到端延迟单独测量，不能拿该目标替代整条链路结果。
- 人为暂停前台任务验证 HOST 心跳超时仍由1 kHz路径执行；Flash等全局阻塞的实际限制在测试报告中说明。

### 10.3 真机测试顺序

1. 先仅供电、观测通信，核对7路兼容及v2的Iq/Id/速度/状态。
2. 有参数和无参数分别上电，确认Studio配置不自动运行/辨识。
3. STOPPED下接管，选择TORQUE+HOST，限流0.10 A；START后不发Iq应保持零给定。
4. 依次发送 +0.03、0、-0.03、0 A，核对ACK实际值、系统最终目标与PI目标。
5. 运行时请求切换模式应拒绝；STOP后重新接管并切FREE，再START验证零Iq闭环行为。
6. 切换TORQUE+POT、POSITION+POT，确认PC Iq不会覆盖电位器，模式不互相残留积分。
7. 非零Iq时拔线、关闭程序、终止PC进程，确认超时停机，重连仍停机且旧Iq不重放。
8. 编码器过期进入故障；恢复传感器、清故障后仍停止。
9. 本地辨识过程中STOP，确认后续控制周期不会恢复开环PWM。
10. 验证uvw_dir=+1/-1的机械恢复方向；Spring/Limit跨单圈零点与±180°不反向；Detent跨圈连续。
11. 连续记录30分钟并核查丢帧统计、内存、CPU、日志和断连恢复。

### 10.4 最终交付判据

- 上位机能命令下位机切换所有已声明支持的工作模式，并回读实际模式。
- 不支持、条件不符或参数错误的命令都有可解释的拒绝结果。
- START与模式选择独立；STOP/故障/失联撤销输出；重连/清故障不自动恢复。
- 当前工作模式仅一个控制分支产生Iq，电位器与HOST不会交替覆盖目标。
- PC显示值、协议单位和固件实际生效值一致。
- 输出上下位机版本、协议说明、测试结果和可运行PC包，不仅提供界面截图。

## 11. 文档与发布要求

- 更新上位机 README：当前7路事实、v2能力、模式/启动操作顺序和构建方式。
- 重写 firmware_adapter/INTEGRATION.md：删除旧提前return接入，指向固件正式模块及本方案。
- 固化验收通过的Python依赖版本并保留依赖清单；打包脚本以自身目录为工作目录，失败立即停止，不无条件显示Build finished。
- 检查源文件/批处理首行多余反斜杠：Python/C中的续行与批处理解释不同，删除生成残留，避免打包脚本启动错误。
- 协议常量和schema变更必须同步更新Python、C、golden vectors和文档；破坏性变化升级协议版本或schema，不复用旧含义。
- 回滚方式为成对恢复对应固件与上位机版本；旧固件搭配新版PC使用Legacy只读profile。不要通过重新启用不校验会话的旧运动命令来实现兼容。

## 12. 实施前约定

本方案默认采用：Studio启动停机、显式控制权、停止后切模式、1.10 A会话默认限流、100 ms心跳/500 ms超时、200 Hz遥测、参数仅RAM生效。它们是拟议工程默认值，实施者应按本文实现并在台架验证后记录调整，不能把它们当作已确认的硬件安全规格。

本文不要求重新实现已工作的FOC变换、电流PI、编码器DMA或参数辨识算法。改动重点是给现有系统增加一致的控制入口、协议、状态反馈和上位机工作流程。
