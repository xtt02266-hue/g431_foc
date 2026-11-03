# PC 工具

`tools/analyze_motor_history.py` 分析历史 CSV 的速度、电流和位置谐波。
脚本从 `docs/diagnostics` 迁移，内容保持不变；需要 NumPy 和 pandas。

PC GUI HapticKnobStudio 位于独立工程，本次没有修改其代码。
下位机串口收发和协议实现位于 `firmware/communication`。

## 电流环 Bode 分析

`tools/current_loop_bode.py` 是离线分析工具，依赖 Python 3 与 NumPy，不需要 Matplotlib、SciPy 或 MATLAB。
它不连接串口、不修改固件、不烧录、不启动电机。以下命令在仓库根目录执行：

```powershell

```

默认从当前固件配置读取单相 R/L、采样周期、反馈滤波系数和 PI 缩放系数；比较当前普通控制整定参数与
150、300 Hz，并假设额外 PWM 生效延迟为一个采样周期。默认报告在 `tmp/current_loop_bode/report.html`，
可直接用浏览器打开；同目录还有四张独立 SVG、`model_response.csv` 和 `summary.json`。
再次运行会覆盖所选输出目录内的这些同名文件，输入 CSV 不修改。

比较延迟假设，或加入音乐模式的 2000 Hz 整定参数：

```powershell
python host/tools/current_loop_bode.py --bandwidths 75,150,300 --delays 0,1,2 --show
python host/tools/current_loop_bode.py --bandwidths 75,300,2000 --output tmp/bode_music --show
```

R/L 使用单相单位 Ω、H，采样率单位 Hz。可以显式覆盖模型参数：

```powershell
python host/tools/current_loop_bode.py --resistance 5.16 --inductance 0.00238 --sample-rate 20000 --alpha 0.25
```

这些参数同时用于 PI 整定和被控对象，不表示已完成电机参数辨识。脚本的默认配置读取依赖当前源码形式；
固件整定公式、积分时序或转换规则变化后，应同步维护模型与测试。

### 模型定义与结果边界

- 静止转子、固定电角度下的单轴小信号模型；相电流低通在此条件下等效为 dq 轴低通。
- PI 按固件的“先累加本拍积分，再输出”顺序实现：`C(z) = Kp + Ki*Ts/(1-z^-1)`。
- 电机采用 RL 精确零阶保持离散模型：`P(z) = b*z^-1/(1-a*z^-1)`，
  `a = exp(-R*Ts/L)`，`b = (1-a)/R`。
- 反馈滤波为 `H(z) = alpha/(1-(1-alpha)*z^-1)`；额外延迟为 `z^-d`。
  RL 模型已经含 ZOH，不再额外叠加半拍 ZOH；`d` 只是假设的额外整拍延迟，须用硬件时序确认。
- 开环为 `L = C*P*z^-d*H`。图中的闭环为 PI 目标到滤波后反馈电流的 `L/(1+L)`；
  到未滤波物理电流的 `C*P*z^-d/(1+L)` 另存于 CSV。
- JSON/报告包含开环所有已找到的 0 dB 交越与相位裕度、负奇数倍 180° 交越与增益裕度、
  特征多项式极点判稳、稳定模型的反馈闭环 −3 dB 带宽。裕度只在所选频段搜索，未找到不表示无限裕度。
- 不稳定模型的代数频响也会画出，但不能解释为实际可建立的稳态响应，不报告其闭环带宽。
- 未包含机械运动、反电动势变化、角度延迟/误差、dq 耦合、ADC 量化噪声、电压饱和、
  母线变化、死区/压降或电压补偿。程序不据此自动给出可直接上板的 PI 推荐值。

### 叠加实测数据

如果已有逐频点扫频结果，可以在闭环图上叠加。UTF-8 CSV 必须包含：

```csv
frequency_hz,gain_db,phase_deg
```

每行一个频点，频率须为正且不能重复；相位单位为度，负值表示滞后。`gain_db` 为
`20*log10(滤波后反馈电流基波幅值/实际送入PI的目标电流基波幅值)`，相位也按同样的输入/输出定义。
这些值应来自小信号扫频的稳态同步测量，不能直接填入普通遥测的瞬时电流。

```powershell
python host/tools/current_loop_bode.py --measured path/to/measured_bode.csv --show
```

黑色曲线为用户提供的实测闭环数据；程序不从它直接推断开环稳定裕度，也不检查其采集质量。
原始时域 CSV 的幅相提取、MCU 正弦注入、同步采集和串口控制尚未实现。当前普通遥测约每 5 ms
尝试发送，且目标/反馈未在同一 ADC 周期锁存，不能据此可靠测量数百 Hz 的电流环频响。

离线验证入口：

```powershell
python -m unittest discover -s host/tools -p test_current_loop_bode.py -v
```

验证包括独立逐拍时域模型与频响结果对照、理想 RL/PI 极限、模型不稳定案例、CSV 输入边界和报告生成。
通过这些检查不表示 MCU 执行时间、PWM 时序或实际电机已验证。
