# 任务 4/5/6 启动补偿（2026-10-04 开发版）

本次针对“小车刚启动时小球明显偏移”增加启动角度补偿。15:17初版已由用户烧录，任务4/小车模式3 TIME试车仍有明显位移，同一上电周期事后读到新增启动项累计触发数为0。当前17:12 Debug为新增RAM记录版，前馈参数仍为下表值，待重新烧录。试车断开ST-Link、结束后保持供电由助手读取，见 [记录版说明](LAUNCH_TRACE_README.md)。`Firmware/H_BallBalance_CBoard_Final.elf` 仍保存原比赛固件。

## 1. 为什么按实际加速度触发

现场操作是先启动 Pi 任务，再单独按 3507 小车键。Pi 的 `RUN_ACTIVE` 只让 C 板等待起步，不能直接触发倾角；否则补偿可能在小车出发前已经退出。

参考 [good1example 的 balance_pid.c](https://github.com/good1example/2026-H-balance/blob/ca760e90be740240d7c9a9ae9f11dcda17f8202a/user_driver/balance_pid.c) 中“检测小车启动后叠加短时角度，并逐渐撤掉”的做法。该程序有 PA31 启动信号，使用 2° 补偿、100 ms 延时和 5 s 线性退出。当前 3507 没有向 C 板传送启动信号，因此采用 BMI088 实测起步；电机角度符号和退出时间按本机结构设置，未照搬参考程序参数。

触发条件同时满足：

- 合法的 T4/T5/T6 上下文，`CONTROL_ENABLED` 与 `RUN_ACTIVE` 有效，且未 DONE/TIMEOUT。
- 球控处于 `Balancing`，视觉测量有效且年龄不超过 80 ms。
- IMU 已标定、快照有效、加速度有限；距离最后一个新 IMU 输出不超过 100 ms。
- 连续两个**不同 IMU 输出计数**对应的过滤后前向加速度均不小于 `0.06 m/s²`。重复读取同一快照不能累计。

这条路径可在球发生较大位移前对车体加速度作出补偿，但不能在小车电机动作之前预倾。采样、过滤和最终电机跟随会引入延迟。若试车显示触发始终偏晚，下一步应增加小车实际启动信号，而不是继续加大角度。

## 2. 当前参数与合成方式

参数集中在 `Application/BallControlConfig.h` 的 `kVehicleLaunch*` 部分。

| 参数 | 初始值 | 含义 |
|---|---:|---|
| `kVehicleLaunchFeedforwardEnabled` | `true` | 是否启用新增启动项 |
| `kVehicleLaunchAngleOffsetDeg` | `-2.0°` | 起步后的附加电机角度；负角使球向车头运动 |
| `kVehicleLaunchAccelerationThresholdMps2` | `0.06 m/s²` | 过滤后前向加速度触发门限 |
| `kVehicleLaunchRequiredNewSamples` | `2` | 连续符合门限的新输出数 |
| `kVehicleLaunchHoldMs` | `100 ms` | 满幅保持时间，从第二个符合门限的输出开始计时 |
| `kVehicleLaunchReleaseMs` | `700 ms` | 保持结束后线性减到零的时间 |
| `kVehicleLaunchMaxTotalOffsetDeg` | `4.0°` | 启动项有效时，PD 与各前馈合成后的总偏移上限 |

若未提前取消，以触发时刻为 `t=0`：前 100 ms 输出 `-2°`；100～800 ms 线性回到 0；800 ms 后不再输出启动项。

合成顺序为：

```text
视觉 PD + 原有加速度前馈（或任务4停车补偿）+ 新启动项
    → 按当前总角度上限限幅
    → 加机械水平角 1.2461 rad
    → 原有安装角变化率、跟随误差与绝对角保护
    → CAN 电机命令
```

启动窗口内总偏移最多 `±4°`；窗口结束后恢复原有动态上限，通常为 `±3°`。原有静摩擦脱困和任务4停车补偿仍按各自条件工作。`4°` 是总偏移上限，不是额外叠加 4°。实际发送角仍受 `25°/s` 变化率、`0.75°` 跟随误差、速度暂停条件以及 `1.10～1.40 rad` 命令范围约束。

原 PD、T4 启动/刹车增益 10、T5/T6 加速度增益 1、T4 停车后 `+4° / 300 ms / 700 ms` 均保留。T3 不启用新增启动项。

## 3. 取消与轮次处理

启动项有效期间，只要过滤后加速度变成负值，立即撤销启动项，让原刹车前馈工作；加速度回到零时可以继续定时退出。IMU 失效或过期、视觉失效、退出 Balancing、控制失能或任务结束也撤销启动项。取消后，同一轮不能因 IMU 恢复再次触发。

轮次身份使用 `(pi_session_id, run_id, task_id)`。每轮通常只触发一次，新轮次重新等待。固定保存最近 8 个已经触发的身份，防止近期 A→B→A 的身份回退导致重放；这是有限历史，不保证跨超过 8 轮的历史回退，也不跨 C 板重启保存。

## 4. Live Watch 与试调

| 变量 | 观察内容 |
|---|---|
| `vehicle_launch_debug_state` | `0` 禁用；`1` 等待；`2` 连续采样确认；`3` 补偿中；`4` 本轮已用完；`5` 已取消 |
| `vehicle_launch_debug_active` | 本周期是否应用启动项 |
| `vehicle_launch_debug_offset_deg` | 新增启动项的角度，不包含 PD 和原 IMU 项 |
| `vehicle_launch_debug_elapsed_ms` | 补偿期间距触发时刻的时间 |
| `vehicle_launch_debug_trigger_count` | 本次 C 板运行累计触发数，不是每轮清零 |
| `imu_feedforward_debug_accel_mps2` | 用于触发和取消的过滤后加速度 |
| `imu_vehicle_output_count` | IMU 新输出计数 |
| `imu_feedforward_debug_angle_offset_deg` | 原 IMU/停车补偿项 |
| `imu_feedforward_debug_total_offset_deg` | 合成并限幅后的总偏移 |
| `ball_control_debug_last_sent_angle_rad` | 最终实际发送的安装角 |

按实际操作顺序试车并记录：

1. 先启动 Pi 的 T4，保持小车静止几秒。启动项应为零、触发数不增加；PD 保持闭环。
2. 再按小车键。记录正加速度越过门限、状态进入 3、最终命令角开始下降三个时刻，以及小球最大位移与退出后的过冲。
3. 用同一轮继续行驶和刹车。启动项不能重放；一出现负加速度应归零，原刹车/停车补偿仍可工作。
4. 新开一轮确认可以再次触发，再分别验证 T5、T6。目标误差按题目要求检查，不能只看触发计数。

调角时每次改变 `0.5°`，重新编译后再试同一路线：若仍向车尾偏，且触发及时、总角未长期限幅，可把 `-2°` 调到 `-2.5°`；若提前向车头过冲，先减小为 `-1.5°`。保持/退出时间用于调整补偿持续长度。若总角已在 `-4°` 限幅，加大单项角度不会继续增加实际命令；先判断触发延迟、加速度和电机跟随情况。静止时误触发则先检查标定、振动与门限。

设 `kVehicleLaunchFeedforwardEnabled=false` 可对照原有路径。全部参数是编译时常量，Live Watch 修改诊断变量不会改变补偿配置。

## 5. 构建与验证记录

```powershell
Set-Location F:\260801_电赛省赛\H_BallBalance_CBoard
. .\Tools\Enter-CBoardEnv.ps1
cmake --preset Debug
cmake --build --preset Debug
.\Tests\run_competition_launch_feedforward_tests.ps1
.\Tests\run_competition_imu_feedforward_tests.ps1
.\Tests\run_competition_imu_feedforward_integration_tests.ps1
```

2026-10-04 已确认：

- 新增启动补偿 17 项主机测试通过，覆盖异步起步、重复快照、门限/连续采样、保持/退出、双向限幅、失效取消、轻刹车、原停车补偿、轮次回退和毫秒计时回绕。
- 原竞争任务 IMU 前馈 8 项、球控 35 项、任务上下文 19 项，以及加速度估计器、IMU 快照、IMU 运行监测、QD4310 命令和相关集成/保护检查通过。
- 完整 Debug ARM 构建、链接通过。Flash 使用 88,872 B，RAM 使用 26,152 B。球控任务入口静态栈帧为 1,200 B；这不等于真机任务栈余量验收。
- 新固件：`build/Debug/H_BallBalance_CBoard.elf`，1,987,868 B，SHA256：`AD08062C06A36D22F95EF1F4D56CDA58754E4AF9A7D1106CFAF53A8F0A941D07`。
- 原冻结固件 SHA256 仍为 `643176AF12E70AC2327C730CB123167D9BF984194ADADF010FD06FD06969C10F`，未覆盖。
- 用户执行 `Tools/Flash-CBoard.ps1` 后提供的日志确认15:17初版烧录、校验和复位完成。随后任务4视频显示首段相对首帧约5.3cm向车尾位移，后段还有较大往返过冲；这是25cm像素比例估计，未应用Pi五点标定，也未同步实际发车时刻。事后原固件Flash与15:17 ELF逐字节一致，且累计触发数为0。该信息只能确认新增启动项未触发，尚不能确定是哪道运行条件阻止。

仓库已有一处无关测试不一致：`run_balllink_receiver_tests.ps1` 的旧断言要求 100 ms 超时，但接收器既有配置为 110 ms；修改前已出现 `duplicate link times out after 100 ms` 失败。本次未修改接收器或这组旧断言，不能把本次结果称为全仓库测试全部通过。

修改前源码和相关资料备份在 `Documentation/Backups/2026-10-04-before-startup-feedforward.zip`。本机恢复 `F:\HBall_CBoard_Tools` 到随工程工具目录的 junction，并在 Windows GCC 参数中添加 `-no-canonical-prefixes`，以免链接器把 ASCII 工具路径展开成中文路径后找不到运行库；依据 [GCC 官方目录选项](https://gcc.gnu.org/onlinedocs/gcc/Directory-Options.html)。若迁移后的旧 CMake 缓存仍引用失效工具路径，可加载环境后执行 `cmake --fresh --preset Debug` 再构建。

15:17初版已由用户烧录；当前17:12记录版的构建、内存和验证证据见 [记录版说明](LAUNCH_TRACE_README.md)，尚未烧录。`Tools/Flash-CBoard.ps1` 优先选择当前 Debug ELF，不会自动编译。
