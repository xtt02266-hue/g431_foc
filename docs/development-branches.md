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
无感配置不初始化或轮询 SPI 编码器，编译 AS5600 驱动并在主循环处理 I2C 恢复。
AS5600 使用现有 I2C1：PA15=SCL、PB7=SDA，7 位地址 0x36。

参考角度接口：

```c
float reference_angle;
uint8_t valid = Motor_AngleReference_GetElectricalAngle(
    pole_pairs, as5600_direction, as5600_mechanical_offset_rad, &reference_angle);
```

偏置单位为机械弧度，方向为 +1 / -1，极对数必须非零。
AS5600 零点须独立标定，不能复用 Flash 中的 MT6826S 零点。
返回 0 表示参考不可用，输出参数保持原值；参考接口未接入 FOC 控制。
20 ms 新鲜度门限只用于参考有效性，高速角度比较仍需补偿传输和采样延迟。

## 无感分支的当前边界

现有反电动势 / PLL 模块仍是观测器。工程尚无完整的无感启动、加速、
锁定切换、失锁停机和估算速度反馈流程，因此无感配置是开发基础，不能无感闭环运行。
有感启动接口返回失败，电流闭环不能使能，自动和手动有感辨识被禁止，
系统维持 STOPPED。AS5600 只提供参考，不作为无感控制反馈。
观测器默认关闭；后续须配置电阻、电感、极对数并加入无感启动状态机。
Flash 中现有参数可读取，无感配置不会自动辨识或保存新辨识参数。

## 同步共用修改

共用修复在一个分支提交后，在另一分支 `git cherry-pick <commit>`。
分支默认配置单独提交，避免同步时覆盖另一分支的默认值。
worktree 内提交与普通仓库相同，不要通过复制文件同步两个目录。
