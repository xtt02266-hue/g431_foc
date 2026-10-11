# 当前有感工程与构建入口

仓库根目录的 `codex/sensored` 固定使用 MT6826S SPI 编码器，已经删除有感／无感
配置切换、无感观测器和 AS5600 参考角代码。

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

原编辑器 Debug 构建、烧录和调试路径仍为 `build/Debug/g431_foc.elf`。
`sensored-debug` 可构建到独立目录；`sensorless-debug` 已删除。
需要 Arm GNU Toolchain、CMake 和 Ninja。

既有 `.worktrees/sensorless` 是独立 Git worktree，其代码和提交未在这次重构中修改。
当前根目录构建不再承担该工作目录的兼容配置。

新目录及模块边界见 [架构说明](architecture.md)，协议数值和控制参数沿用原有感代码。
