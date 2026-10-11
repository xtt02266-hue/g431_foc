# G431 有感 FOC

当前工程固定使用 MT6826S SPI 编码器进行有感控制。

## 目录

```text
Core/                       CubeMX 生成的程序入口和外设配置
Drivers/                    STM32 HAL / CMSIS
firmware/
  app/                      系统调度、生命周期、主机命令、周期策略、标定会话
    private/                应用内部共享状态和辅助接口
  control/                  电流环、速度环、位置环、轨迹、SVPWM
  compensation/             齿槽、摩擦模型与摩擦 Flash 存储
  communication/            PC 协议解析、序列化、UART DMA
  calibration/              参数辨识、采集、编码器自校准
  drivers/                  MT6826S、编码器接口、ADC 输入、OLED
  platform/                 板级初始化和电流偏置校准
  algorithm/                PID
  config/                   电机参数和控制常量
  music/                    音乐调制和曲目
  diagnostics/              观测与调试输出
host/tools/                 PC 历史数据分析脚本
docs/                       使用说明、协议、架构说明
refactor_validation/        离线回归和验证记录
```

代码导航和共享状态边界见 [架构说明](docs/architecture.md)。PC GUI HapticKnobStudio 位于独立工程，
本仓库中的 `communication` 是下位机通信代码，`host` 是 PC 端工具。

## 构建

配置 Arm GNU Toolchain、CMake 和 Ninja 后：

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

`sensored-debug` 也是有感构建入口。当前目录已删除无感构建开关和兼容分支。
本次重构的检查结果见 [验证记录](refactor_validation/architecture_20261011/RESULTS.md)。
