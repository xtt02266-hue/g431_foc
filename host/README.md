# PC 工具

`tools/analyze_motor_history.py` 分析历史 CSV 的速度、电流和位置谐波。
脚本从 `docs/diagnostics` 迁移，内容保持不变；需要 NumPy 和 pandas。

PC GUI HapticKnobStudio 位于独立工程，本次没有修改其代码。
下位机串口收发和协议实现位于 `firmware/communication`。
