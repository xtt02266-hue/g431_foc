# 有感 / 无感开发配置

## 分支和工作目录

- `main`：保留拆分前的基线。
- `codex/sensored`：仓库根目录，默认构建 sensored，MT6826S SPI 控制。
- `codex/sensorless`：`.worktrees/sensorless`，默认构建 sensorless，AS5600 I2C 参考角度。

第二个目录是 Git worktree，共享历史，有独立的代码检出和构建目录。
分别用编辑器打开两个目录，原 Debug 构建、烧录和调试任务使用各自的
`build/Debug/g431_foc.elf`。分支默认值位于 `cmake/development-profile.cmake`。

## 构建

每个目录使用现有编辑器 Debug 构建任务，或执行：

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

也可以明确指定配置，输出到独立的 `build/<preset>`：

```powershell
cmake --preset sensored-debug
cmake --build --preset sensored-debug
cmake --preset sensorless-debug
cmake --build --preset sensorless-debug
```

显式配置不会改变原 Debug 烧录任务的固件路径。切换同一目录的默认配置后，
清除旧 Debug 缓存或使用显式 preset；缓存 `FOC_PROFILE` 优先于分支默认值。
需要 Arm GNU Toolchain、CMake 和 Ninja，现有 `.vscode` 已配置本机路径。

## 硬件和角度分层

`user/inc/board_profile.h` 管理控制 / 参考源使能；CubeMX 外设与引脚沿用当前板卡。
有感配置使用 MT6826S SPI1，保留原控制和辨识流程。
无感混合配置不初始化或轮询SPI编码器，使用AS5600缓存角度进行有感控制，
并在主循环处理I2C恢复。
AS5600 使用现有 I2C1：PA15=SCL、PB7=SDA，7 位地址 0x36。

参考角度接口：

```c
float reference_angle;
uint8_t valid = Motor_AngleReference_GetElectricalAngle(
    pole_pairs, as5600_direction, as5600_mechanical_offset_rad, &reference_angle);
```

偏置单位为机械弧度，方向为 +1 / -1，极对数必须非零。
AS5600是当前控制角度源，其零点和方向由现有参数辨识流程标定并保存。
返回0表示参考不可用，输出参数保持原值；FOC通过`Motor_Encoder`抽象读取同一AS5600角度。
20 ms 新鲜度门限只用于参考有效性，高速角度比较仍需补偿传输和采样延迟。

## 无感分支的当前边界

现有反电动势 / PLL 模块仍是观测器。工程尚无完整的无感启动、加速、
锁定切换、失锁停机和估算速度反馈流程，因此无感配置是开发基础，不能无感闭环运行。
当前分支采用混合验证模式：AS5600 继续负责有感闭环换相和参数辨识，
同时作为参考角；反电动势/PLL观测器在有感电流环运行后并行启用。
MT6826S在此配置中不初始化也不读取。
遥测 schema 5 输出无感估算角、AS5600参考角及各自有效标志，便于实机对比。
无感角尚未接管 Park/InvPark；工程仍无完整的无感启动、平滑切换和失锁恢复流程。

## 同步共用修改

共用修复在一个分支提交后，在另一分支 `git cherry-pick <commit>`。
分支默认配置单独提交，避免同步时覆盖另一分支的默认值。
worktree 内提交与普通仓库相同，不要通过复制文件同步两个目录。
