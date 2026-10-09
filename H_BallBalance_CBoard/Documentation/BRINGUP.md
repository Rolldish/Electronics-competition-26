# H 题 C 板、BallLink 与滚球控制真机手册

本文对应 `H_BallBalance_CBoard` 当前固件：

- C 板通过 CAN1 1 Mbps 控制 1 个 `can.id=0` 的 QD4310；
- `VisionCommTask` 只接收 Pi→C 的单向 BallLink；
- `BallControlTask` 以 5 ms/200 Hz 运行并独占 QD4310 命令；
- 5 帧可靠反馈后预置水平角，重试使能并经 3 帧 enabled 确认后保持水平；
- 连续 10 个新鲜视觉观测后进入滚球 PD；
- 视觉软故障限斜率回水平，电机/CAN 硬故障零输出并失能；
- BMI088 200 Hz 纵向加速度已经接入 T4/T5/T6 正式前馈；READY、T2/T3 和失效状态
  仍保持纯视觉 PD 或安全回平。
- 2026-10-04 增加 T4/T5/T6 实际起步检测与短时启动补偿，现场步骤见第17节；新增参数尚未真机验收。
- 2026-10-04 17:12新增RAM记录版，支持断开ST-Link试车、结束后保持C板供电再由助手读数；该版本待烧录，见 [记录版说明](LAUNCH_TRACE_README.md)。

当前装车配置为 `kBallControlCommissioned=true`，使用已经完成第一轮确认的中心
角、方向和限位。机械结构重装、零位复核或方向复核期间必须临时改回 `false`，
此时通信、IMU、CAN 反馈和 Live Watch 可以运行，但电机不会自动使能。
`SimVisionMotorTest` 仍是独立回归固件，不负责替正式配置保存机械参数。
当前参数、任务完成度和最后固件证据统一参见 `Documentation/CURRENT_PROGRESS.md`。

## 1. 当前验证边界

原比赛基线的三端联调与任务1～6真机结果见 `CURRENT_PROGRESS.md`；下文较早日期的
记录保留当时的调试背景。2026-10-04 新增启动补偿已通过17项主机测试、相关控制与
IMU回归、集成/保护检查和完整 ARM 构建；用户日志已确认烧录和校验成功，实际起步效果
仍待真机验收。原 BallLink 接收器
测试仍有100/110 ms超时断言不一致；本次验证范围与证据详见
[启动补偿说明](STARTUP_FEEDFORWARD.md)。

此外已经完成脱离水管和小车的真实硬件台架联调：树莓派以 50 Hz 发送模拟球
位置/速度，数据经过正式 BallLink、状态机和 PD 计算后由 CAN1 驱动 QD4310
短摆杆双向运动；观察到 `Balancing(5)`、`fault=0`、电机使能为 1、反馈持续
增加且 CAN 发送错误为 0。测试专用幅度为 `±3°`、临时机械保护为 `±4°`。
这证明独立控制链路已跑通，但没有水管、真实视觉和钢球动力学，不属于真实稳球。

禁止用虚拟 Pi、串口回环、在线模拟器或伪造在线数据宣称真机完成。本手册下面的
联调步骤必须由用户拿实际树莓派、C 板、QD4310、摆杆和钢球执行。

## 2. 安全要求

- 第一次电机动作前拆下钢球，摆杆周围留出空间，保证能立即断电；
- 烧录、改线、拆线时关闭 QD4310 电源并拔掉电机 Type-C；
- 不带电插拔 CAN、UART、SWD 或电机电源；
- 两块控制板分别正常供电，任何信号连接都要共地；
- 人工改造杜邦线必须固定并做应力释放；通信计数停止或错误计数增加时先停机查线；
- 异常高速、剧烈振动、撞限位、发热、异味或冒烟时立即物理断电；
- KEY 在当前控制固件中是紧急停机输入，不是启动键；
- 软件失能依赖 CAN 正常，不能替代物理断电能力。

## 3. 打开工程、编译和烧录

本机中文用户目录可能让 Cortex-Debug 的 GDB/MI 脚本路径变成 `?`。ST-Link 调试
使用：

```text
F:\260801_电赛省赛\Open-CBoard-Debug.cmd
```

它会打开：

```text
F:\260801_电赛省赛\H_BallBalance_CBoard
```

VS Code 任务：

- `C Board: Build Debug`：ARM Debug 编译；
- `C Board: Run Host Tests`：运行 BallLink、当前 BallControl 主机测试集、车辆加速度、
  IMU 运行时监控、QD4310 命令保护和 RTOS/IWDG 安全检查；
- `C Board: Flash with ST-Link`：编译、烧录、校验并复位。

短摆杆模拟视觉台架固件手工命令：

```powershell
. .\Tools\Enter-CBoardEnv.ps1
cmake --preset SimVisionMotorTest
cmake --build --preset SimVisionMotorTest --parallel
.\Tools\Flash-SimVisionMotorTest.ps1
```

刷写前必须结束正在运行的 VS Code 调试会话。若 OpenOCD 报：

```text
libusb_open() failed with LIBUSB_ERROR_ACCESS
```

先停止调试并确认没有旧进程占用 ST-Link：

```powershell
Get-Process openocd,arm-none-eabi-gdb -ErrorAction SilentlyContinue |
  Stop-Process
```

再重新刷写。该错误属于 ST-Link 被 OpenOCD/GDB 占用，与 USB-TTL→C 板 UART
线束接触不良是两类不同问题。

若任务列表没更新，执行 `Developer: Reload Window`。需要手工终端环境时运行：

```powershell
. .\Tools\Enter-CBoardEnv.ps1
```

## 4. ST-Link 与 QD4310

### 4.1 ST-Link

| ST-Link | C 板 |
|---|---|
| SWDIO | SWDIO |
| SWCLK | SWCLK |
| GND | GND |
| VTref/Vref | 3.3 V 参考脚 |

C 板必须独立供电。VTref 默认只是目标电平参考，不要把用途不明的 3.3 V 针脚当作
C 板电源。

### 4.2 QD4310

电机上位机确认：

```text
can.id = 0
can.baud_rate = 1000000
pid.speed = 0.00400 / 0.000100 / 0
pid.angle = 1200 / 0 / 0.500
limit.current = 1.65 A
limit.speed = 1000 rpm
```

| 项目 | 数值 |
|---|---|
| CAN 波特率 | 1 Mbps |
| 控制帧 | `0x400` |
| 反馈帧 | `0x500` |

保持 CANH→CANH、CANL→CANL、C 板与电机共地。

当前 C 板 CAN 驱动不能读取或写入 QD4310 内部 PID。网页上位机“设置”只修改
当前配置，“储存”还需要设备二次确认：点击“储存”后等待确认弹窗，再点“确认”
发送 `y`。当前实物曾在断电重启后回到速度环 `0.00300/0.000390/0`、角度环
`1200/0/0`，并出现严重抖动，因此持久化仍属于未关闭问题。每次上电按以下顺序：

1. 让 C 板保持停止/未控制，Type-C 只接上位机；
2. 点击“读取参数”，逐项与上面的基线比较；
3. 若不一致，重新“设置”，再次“读取参数”确认本次运行已生效；
4. 点击“储存”并完成二次确认，记录终端成功信息；
5. 失能、完全断电、重新上电，再次读取参数；只有这一遍仍一致才算持久化通过；
6. 持久化未通过期间，每次上电都必须重复读参，之后断开上位机再启动 C 板。

如果以前只点击了“储存”、没有在设备确认弹窗继续，则重启丢失符合未完成储存
流程的现象；重新按第 4～5 步验证即可。如果已经明确完成二次确认，终端也显示
成功，但完全断电重启后仍恢复旧值，则暂记为 QD4310 固件/非易失存储问题，不能
用 C 板滚球外环参数补偿，也不能声称已经永久保存。

## 5. 启动调试与 Live Watch

1. 正式固件选择 `C Board: Debug with ST-Link`；台架固件选择
   `C Board: Debug SimVisionMotorTest with ST-Link`。
2. 第一次按 `F5`，完成构建/下载并停在 `main()`。
3. 第二次按 `F5`，让 FreeRTOS 继续运行。
4. 打开调试侧栏的 `CORTEX LIVE WATCH`。
5. 只添加需要的标量变量；不要观察大数组或会修改芯片状态的表达式。

Live Watch 配置每秒最多刷新 10 次。它不是任务周期，也不是传感器采样率。

正式固件启动约 500 ms 的 IWDG，并设置了调试暂停冻结位。因此停在断点时 IWDG
也暂停，不会因为查看变量而复位；继续运行后仅 `BallControlTask` 有权喂狗。
如果该任务不再调度、FreeRTOS 栈溢出、动态内存分配失败或进入致命错误路径，
固件停止喂狗并由 IWDG 复位。不要在调试器里手工调用喂狗函数。

正式固件 LED 含义：

| LED | 含义 |
|---|---|
| 蓝灯常亮 | BootSafe、WaitMotor、EnablePending 或 VisionRecovery，正在等待/恢复 |
| 绿灯常亮 | LevelHold，已保持机械水平 |
| 绿灯闪烁 | Balancing，正在滚球闭环 |
| 红灯常亮 | ConfigRequired、控制硬故障或致命 RTOS/初始化错误 |

红灯不等于一定发生硬故障。正式装车配置当前已 commissioned；如果人为关闭
commissioning，`ball_control_debug_state=2`、`ball_control_debug_fault=0` 会以
红灯显示 `ConfigRequired`。正常正式启动若出现红灯，结合 `state`、`fault` 和
`safety_debug_fatal_code` 判断，不能只凭灯色。

看门狗/致命错误诊断变量：

| 变量 | 含义 |
|---|---|
| `safety_debug_watchdog_refresh_count` | `BallControlTask` 成功喂狗的累计次数；固件正常运行时持续增加 |
| `safety_debug_fatal_code` | 当前启动期间的致命错误码；`0` 表示未进入致命错误路径 |
| `safety_debug_reset_flags` | 上一次复位原因的 RCC 原始标志；读取后固件会清除硬件复位标志 |

若怀疑看门狗复位，复位后先观察 `safety_debug_reset_flags`；不要指望
`safety_debug_fatal_code` 跨复位保留。

## 6. BMI088 检查

### 6.1 原始 9 个变量

```text
imu_debug_accel_x_mps2
imu_debug_accel_y_mps2
imu_debug_accel_z_mps2
imu_debug_accel_norm_g
imu_debug_sample_count
imu_debug_status
imu_debug_driver_error
imu_debug_dma_error_count
imu_debug_overrun_count
imu_debug_recovery_count
imu_debug_last_transfer_error_source
```

| 变量 | 单位/取值 | 注释 |
|---|---|---|
| `imu_debug_accel_x_mps2` | m/s² | BMI088 板载 X 轴加速度，包含重力分量；正负方向由板上坐标系决定 |
| `imu_debug_accel_y_mps2` | m/s² | BMI088 板载 Y 轴加速度，包含重力分量 |
| `imu_debug_accel_z_mps2` | m/s² | BMI088 板载 Z 轴加速度，包含重力分量 |
| `imu_debug_accel_norm_g` | g | 三轴合加速度模长除以标准重力；静止时应接近 `1.0` |
| `imu_debug_sample_count` | 次 | 成功发布的原始加速度样本累计数；正常运行时应持续增加 |
| `imu_debug_status` | 0～3 | IMU 任务状态，具体编号见下表 |
| `imu_debug_driver_error` | 驱动错误码 | BMI088 最近一次初始化或运行错误；正常应为 `0` |
| `imu_debug_dma_error_count` | 次 | SPI DMA 错误累计数；正常应保持不变，通常为 `0` |
| `imu_debug_overrun_count` | 次 | 新采样覆盖尚未处理数据的累计次数；持续增加说明任务处理不及时 |
| `imu_debug_recovery_count` | 次 | IMU 运行时错误后完成停止/恢复流程的累计次数 |
| `imu_debug_last_transfer_error_source` | 0～6 | 最近一次运行时传输错误来源：0=None、1=HAL 回调、2=启动 Busy、3=启动 Error、4=恢复失败、5=传输超时、6=样本超时 |

`imu_debug_status`：

| 值 | 状态 |
|---:|---|
| 0 | Initializing |
| 1 | Ready |
| 2 | InitError |
| 3 | RuntimeError |

静止时一个轴约为 ±9.8 m/s²、模长接近 1 g、样本计数持续增加。改变板子朝向时
重力分量应在三轴间转移。

运行中一次 SPI 传输超过 20 ms，或连续 100 ms 没有新样本，都会令状态进入
`RuntimeError(3)`、使 `imu_vehicle_valid=0`，停止运行模式，等待 500 ms 后重新
初始化。恢复成功后 `imu_debug_status` 回到 `Ready(1)`，样本计数继续增加；
`imu_debug_recovery_count` 和最近错误来源保留用于定位。反复恢复说明 SPI、DRDY
中断、供电或任务调度仍有问题，不能把旧加速度当成在线数据。

### 6.2 车辆纵向候选量

```text
imu_vehicle_status
imu_vehicle_calibrated
imu_vehicle_valid
imu_vehicle_calibration_samples
imu_vehicle_output_count
imu_vehicle_baseline_x_mps2
imu_vehicle_baseline_y_mps2
imu_vehicle_baseline_z_mps2
imu_vehicle_noise_mps2
imu_vehicle_deadband_mps2
imu_vehicle_forward_raw_mps2
imu_vehicle_forward_filtered_mps2
```

| 变量 | 单位/取值 | 注释 |
|---|---|---|
| `imu_vehicle_status` | 0～3 | 车辆纵向加速度估算状态，编号见下文 |
| `imu_vehicle_calibrated` | 0/1 | 静止基线是否标定成功；`1` 表示已完成 |
| `imu_vehicle_valid` | 0/1 | 当前纵向输出是否可用；状态正常且数据通过检查时为 `1` |
| `imu_vehicle_calibration_samples` | 次 | 本次静止标定已经累计的样本数 |
| `imu_vehicle_output_count` | 次 | 标定完成后有效纵向输出累计数；正常运行时应持续增加 |
| `imu_vehicle_baseline_x_mps2` | m/s² | 静止标定得到的板载 X 轴基线，包含固定安装姿态下的重力投影 |
| `imu_vehicle_baseline_y_mps2` | m/s² | 静止标定得到的板载 Y 轴基线 |
| `imu_vehicle_baseline_z_mps2` | m/s² | 静止标定得到的板载 Z 轴基线 |
| `imu_vehicle_noise_mps2` | m/s² | 标定期间估计的纵向噪声幅度 |
| `imu_vehicle_deadband_mps2` | m/s² | 根据噪声计算的零点死区；小于该幅度的变化按零处理 |
| `imu_vehicle_forward_raw_mps2` | m/s² | 沿所配置车头方向投影并扣除静止基线后的原始纵向值 |
| `imu_vehicle_forward_filtered_mps2` | m/s² | 对原始纵向值做死区和滤波后的加速度；T4/T5/T6 正式前馈使用该值 |

`imu_vehicle_status`：0=Calibrating、1=Ready、2=CalibrationRejected、
3=InvalidConfig。上电后保持整车静止约 2–3 s；标定通过后滤波纵向值应在 0 附近。

当前值仍可能混入俯仰引起的重力投影。最终装车后必须重新确认车头轴向和符号。
该值已经进入 `BallControlTask` 的 T4/T5/T6 前馈门控；T2/T3、READY、视觉恢复和
IMU失效时前馈为零。首轮静止 O 点基线仍按纯视觉 PD 解释。

## 7. 单向 BallLink 接线

通信测试先保持：

```text
kBallControlCommissioned=false  # 关闭自动使能安全门，仅观察通信和调试变量
QD4310 可以断电                 # 单独测试 UART 时无需让摆杆动作
```

全部断电后只接：

```text
树莓派 USB → CH340 USB-TTL
USB-TTL TX  → C板外接串口针脚3：PG9/USART6_RX
USB-TTL GND → C板外接串口针脚1：GND
```

本版联调不要接：

```text
USB-TTL RX → C板 PG14/USART6_TX
USB-TTL VCC/3.3V/5V → C板任意供电脚
```

串口为 115200-8-N-1、无流控、3.3 V TTL。树莓派使用：

```text
/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
```

不得使用已经产生 NE/FE/CRC 错误的 GPIO14 `/dev/serial0` 路径，也不得在正式
配置中写死可能随插拔变化的 `/dev/ttyUSB0`。当前物理口约定为 WiFi 使用左上
USB 3.0，USB-TTL 使用右下 USB 2.0；同一个 USB-TTL 换物理口时 by-id 通常不变。
C 板保留 PG14/USART6_TX 和 `UART_MODE_TX_RX` 硬件能力，但本版运行时不主动
发送，不回传状态、不发送 ACK。

本次联调发现人工改造的 USB-TTL 杜邦线存在疑似接触不良：Pi 发送仍保持约
50 Hz 时 C 板接收计数停止，触碰线束后恢复，并累计 UART、CRC 和丢弃字节错误。
看门狗仍刷新且致命码为 0，优先指向物理连接而不是程序卡死。正式安装水管或
上车前，必须更换为可靠焊接、压接或带锁扣连接，增加应力释放，并重新执行静止
和轻微晃动条件下的 60 秒零错误测试。

## 8. BallLink 冻结协议检查

唯一业务帧：

```text
A5 5A | 02 | 10 | sequence | 0x18 | 24字节payload | CRC16_LE
完整帧：32字节
频率：50 Hz
payload格式：<IIIhhBBhBBH
```

24 字节载荷依次是：

```text
pi_session_id
capture_timestamp_ms
capture_to_send_delay_us
x_0p1mm
v_mmps
confidence
vision_flags
target_x_0p1mm
task_id
control_flags
run_id
```

| 字段 | 类型/单位 | 注释 |
|---|---|---|
| `pi_session_id` | `uint32` | 树莓派进程会话号；进程重启时生成新值，使 C 板丢弃旧会话历史 |
| `capture_timestamp_ms` | `uint32`，ms | 图像采集时刻，使用 Pi 自身单调时钟；只在同一会话内比较 |
| `capture_to_send_delay_us` | `uint32`，µs | 从图像采集到即将发串口帧的处理延迟 |
| `x_0p1mm` | `int16`，0.1 mm | 球的位置，摆杆控制端为正；无球时必须为 `0` |
| `v_mmps` | `int16`，mm/s | 球速度，正方向与 `x` 一致；速度无效时必须为 `0` |
| `confidence` | `uint8`，0～255 | 识球置信度；无球时为 `0` |
| `vision_flags` | `uint8` 位标志 | 球有效、速度有效、相机已标定和处理降级状态 |
| `target_x_0p1mm` | `int16`，0.1 mm | C 板位置环使用的目标位置；O 点通常为 `0` |
| `task_id` | `uint8` | 当前任务编号，由树莓派定义并随完整快照发送 |
| `control_flags` | `uint8` 位标志 | 控制允许、运行状态、任务完成/超时及目标锁存状态 |
| `run_id` | `uint16` | 测试轮次编号；新轮次必须改变，促使 C 板清旧观测门 |

当前 C 板只把 `task_id` 发布到诊断变量，不用它选择控制参数或改变状态机。

CRC-16/CCITT-FALSE：poly `0x1021`、init `0xFFFF`、xorout `0`，覆盖
`VERSION|TYPE|SEQUENCE|LENGTH|PAYLOAD`，帧尾低字节先发。

### 8.1 必须先通过的 CRC 向量

```text
A5 5A 02 10 2A 18
78 56 34 12
04 03 02 01
98 3A 00 00
F4 01
06 FF
DC
07
F4 01
03
03
2A 00
F0 68
```

CRC 应为 `0x68F0`，线上字节为 `F0 68`。树莓派编码出的 32 字节必须逐字节一致。

### 8.2 视觉状态

`vision_flags`：

```text
0x01 BALL_VALID
0x02 VELOCITY_VALID
0x04 CAMERA_CALIBRATED
0x08 PROCESSING_DEGRADED
```

| 标志 | 注释 |
|---|---|
| `BALL_VALID` | 当前画面检测到可用钢球 |
| `VELOCITY_VALID` | `v_mmps` 是可用速度估计；未置位时 C 板忽略速度项 |
| `CAMERA_CALIBRATED` | 像素到毫米和视觉 O 点标定已经完成 |
| `PROCESSING_DEGRADED` | 当前图像处理降级；置位时测量不进入闭环 |

`control_flags`：

```text
0x01 CONTROL_ENABLED
0x02 RUN_ACTIVE
0x04 TASK_DONE
0x08 TASK_TIMEOUT
0x10 TARGET_LATCHED
```

| 标志 | 注释 |
|---|---|
| `CONTROL_ENABLED` | 允许视觉测量进入球控；清除时 C 板回机械水平 |
| `RUN_ACTIVE` | 当前任务轮次正在执行，供状态记录使用 |
| `TASK_DONE` | 树莓派判断本轮任务已经完成 |
| `TASK_TIMEOUT` | 树莓派判断本轮任务已经超时 |
| `TARGET_LATCHED` | 本轮目标已经锁存，不应随临时界面输入漂移 |

无球时仍以 50 Hz 发合法帧，但 `BALL_VALID=0`、`VELOCITY_VALID=0`、
`x=v=confidence=0`。同一图像重复广播时采集时间戳不变、处理年龄继续增加。

进入 `Balancing` 前必须持续置位 `CONTROL_ENABLED`。只有位置、标定、置信度、
年龄等条件满足但没有该标志时，测量仍不会通过球控门。

Pi 与 C 板时钟未同步，禁止直接相减。C 板使用
`ceil(capture_to_send_delay_us/1000)`、完整帧本地接收后经过的时间和固定约
`3 ms` 串行传输补偿估算测量年龄。同一会话内相同 `capture_timestamp_ms` 是
重复图像，不增加观测计数。

DMA/IDLE 回调为每批字节记录实际到达 tick；字节经过分包、粘包或环形缓冲回绕
后，完整帧仍使用末字节 tick 作为本地接收时刻。环形缓冲/解析器溢出或 UART
错误会丢弃未完成流、清解析状态并立即发布 NoLink，禁止错误前后的半帧拼接。

## 9. BallLink Live Watch

先添加核心变量：

```text
vision_debug_status
vision_debug_x_0p1mm
vision_debug_v_mmps
vision_debug_target_x_0p1mm
vision_debug_confidence
vision_debug_flags
vision_debug_task_id
vision_debug_control_flags
vision_debug_run_id
vision_debug_measurement_age_ms
vision_debug_measurement_update_count
vision_debug_last_session_id
vision_debug_rx_frame_count
vision_debug_accepted_frame_count
vision_debug_valid_measurement_count
```

| 变量 | 单位/取值 | 注释 |
|---|---|---|
| `vision_debug_status` | 0～3 | BallLink 当前状态，具体编号见下表 |
| `vision_debug_x_0p1mm` | 0.1 mm | 球的当前位置；正方向为摆杆控制端，`+500` 表示 `+50.0 mm` |
| `vision_debug_v_mmps` | mm/s | 球沿同一坐标轴的速度；正负方向应与 `x` 一致 |
| `vision_debug_target_x_0p1mm` | 0.1 mm | 树莓派给出的球位置目标；`0` 为视觉 O 点 |
| `vision_debug_confidence` | 0～255 | 识球置信度；`0` 完全不可信，`255` 最高，当前闭环门限为 `180` |
| `vision_debug_flags` | 位标志 | `BALL_VALID`、`VELOCITY_VALID`、相机标定和处理降级标志的组合 |
| `vision_debug_task_id` | 无符号整数 | 树莓派给出的当前任务编号，只作为任务上下文记录 |
| `vision_debug_control_flags` | 位标志 | 控制允许、运行中、任务完成、任务超时和目标锁存标志的组合 |
| `vision_debug_run_id` | 无符号整数 | 当前测试轮次编号；改变时 C 板清除上一轮连续有效观测 |
| `vision_debug_measurement_age_ms` | ms | 当前视觉测量的估算真实年龄；`80 ms` 仍有效，超过 `80 ms` 不作为有效闭环测量 |
| `vision_debug_measurement_update_count` | 次 | 新采集图像被接受的累计数；重复时间戳不会增加 |
| `vision_debug_last_session_id` | 无符号整数 | 最近接受的树莓派进程会话号；Pi 进程重启后应改变 |
| `vision_debug_rx_frame_count` | 帧 | 通过流解析和 CRC 检查的完整帧累计数 |
| `vision_debug_accepted_frame_count` | 帧 | 通过字段、时序和协议检查的合法帧累计数；重复或乱序帧不计入 |
| `vision_debug_valid_measurement_count` | 次 | 被接收器认定为新测量的累计数；不等于当前连续闭环门计数 |

错误排查再添加：

```text
vision_debug_rx_byte_count
vision_debug_crc_error_count
vision_debug_range_error_count
vision_debug_protocol_error_count
vision_debug_timestamp_error_count
vision_debug_sequence_gap_count
vision_debug_duplicate_count
vision_debug_out_of_order_count
vision_debug_timeout_count
vision_debug_discarded_byte_count
vision_debug_parser_overflow_count
vision_debug_dma_overflow_count
vision_debug_uart_error_count
vision_debug_session_change_count
vision_debug_run_change_count
```

| 变量 | 单位 | 注释 |
|---|---:|---|
| `vision_debug_rx_byte_count` | 字节 | 从 DMA 环形缓冲取出并送入解析器的累计字节数 |
| `vision_debug_crc_error_count` | 次 | CRC16 校验失败的完整候选帧累计数 |
| `vision_debug_range_error_count` | 次 | `x/v/target/delay` 越界或标志组合非法的累计数 |
| `vision_debug_protocol_error_count` | 次 | 消息类型、版本、长度或载荷解码不符合冻结协议的累计数 |
| `vision_debug_timestamp_error_count` | 次 | 同一会话内采集时间戳异常倒退的累计数；正常回绕不应误报 |
| `vision_debug_sequence_gap_count` | 帧 | 根据 sequence 推算的累计缺帧数 |
| `vision_debug_duplicate_count` | 次 | 收到相同 sequence 的重复帧累计数 |
| `vision_debug_out_of_order_count` | 次 | 收到倒序或过旧 sequence 的累计数 |
| `vision_debug_timeout_count` | 次 | 状态从在线转为 `NO_LINK` 的累计次数，不是每个超时周期都增加 |
| `vision_debug_discarded_byte_count` | 字节 | 为寻找帧头而丢弃的噪声或错位字节数 |
| `vision_debug_parser_overflow_count` | 字节 | 解析器容量不足时丢弃的累计字节数；出现后当前链路立即失效 |
| `vision_debug_dma_overflow_count` | 次 | DMA 接收环形缓冲装满的累计次数；出现后清流并进入 `NO_LINK` |
| `vision_debug_uart_error_count` | 次 | USART6 硬件错误回调累计数；出现后重启接收并清除未完成帧 |
| `vision_debug_session_change_count` | 次 | `pi_session_id` 变化累计数 |
| `vision_debug_run_change_count` | 次 | 同一会话内 `run_id` 变化累计数 |

`vision_debug_status`：

| 值 | 状态 | 含义 |
|---:|---|---|
| 0 | NO_LINK | 还没收到合法帧或合法帧静默超过 110 ms |
| 1 | MEASUREMENT_INVALID | 链路在线但球/标定/置信度/控制标志不可用 |
| 2 | MEASUREMENT_VALID | 当前测量可作为控制候选 |
| 3 | STALE | 最近测量真实年龄超过 80 ms |

Pi v1.6.11 与 C 板的新鲜度合同如下：

- Pi 仅在测量年龄不超过 `70 ms` 时发送有效球位置和速度；
- 测量年龄超过 `70 ms` 且不超过 `200 ms` 时，Pi 仍可发送无效心跳，但会清除
  `BALL_VALID/VELOCITY_VALID`，把 `x/v/confidence` 置零并设置处理降级标志；
- 这种无效心跳只能维持 `LinkOnline`，不得更新 C 板的新测量计数、连续有效计数或
  恢复计数，也不得让控制器使用上一次有效坐标；
- 测量年龄超过 `200 ms` 时 Pi 不再编码该快照。合法帧静默超过 `110 ms` 后，C 板
  进入 `NO_LINK`；
- C 板正常控制测量年龄上限为 `80 ms`。`81～110 ms` 期间保持当前控制状态但将控制量
  回到机械水平，`111 ms` 起进入视觉恢复状态。短暂掉线（不超过 `300 ms`）恢复需要
  连续 `3` 张不同时间戳的有效新图像；首次进入、长时间掉线或系统级故障需要连续
  `10` 张。

## 10. 单向通信真机测试

### A. 只启动 C 板

预期：

```text
vision_debug_status=0
vision_debug_rx_frame_count=0
```

C 板本版不会主动发送任何 UART 数据，但保留的 TX 硬件能力无需删除。

### B. Pi 发送无球帧

预期：

```text
vision_debug_status=1
rx_frame_count和accepted_frame_count增加
valid_measurement_count不增加
x/v/confidence为0
```

### C. Pi 发送固定有效快照

建议：

```text
x=+500
v=-250
target_x=+500
confidence=220
vision_flags=0x07
task_id=3
control_flags=0x03
run_id=42
```

预期相应 Live Watch 字段完全一致，`vision_debug_status=2`，
`vision_debug_measurement_update_count` 随新图像增加。

再发 `x=-500、target_x=-500`，确认负数。人工把球放在 QD4310 一侧，确认真实
坐标为正。

串口助手若循环发送完全相同的 32 字节，序号和采集时间戳也会重复：
`rx_byte_count/rx_frame_count` 会增加，但第一帧后主要增加
`duplicate_count`，`accepted_frame_count` 和新测量计数不会逐帧增加。这是重复帧
诊断，不是 UART 丢包。手工改动序号时，跳号会增加 `sequence_gap_count`；同一
会话的采集时间戳倒退会增加 `timestamp_error_count`。

要做连续接收测试，请在项目目录运行：

```powershell
.\Tools\Send-BallLinkContinuous.ps1 -PortName COM18 -IntervalMs 20 -DurationSeconds 60
```

树莓派运行：

```bash
python3 Tools/Send-BallLink-RaspberryPi.py --port /dev/serial0 --interval-ms 20 --duration-seconds 60
```

持续时间设为 `0` 可一直发送。两个脚本都会递增 sequence 和采集时间戳，但不会
自动重连；串口断开、被占用或写入失败后，排除原因并重新启动脚本。

### D. 重复、会话和任务

1. 继续发送新序号但保持相同采集时间戳，确认同一图像不会增加
   `measurement_update_count`。
2. 重启 Pi 进程，`pi_session_id` 改变，C 板清旧视觉历史。
3. 改变 `run_id`，C 板清上一轮连续有效观测。

### E. 错误和超时

1. 破坏 CRC：CRC 错误计数增加，错误帧不被接受。
2. 发越界 x/v/target/delay：范围错误计数增加。
3. 停止发送：超过 110 ms 后进入 NO_LINK，超时计数增加。
4. 恢复合法 50 Hz：自动恢复，无需复位 C 板。
5. 用分包、粘包、噪声和环形缓冲回绕检查末字节到达 tick；触发 ring/parser
   溢出或 UART 错误时，必须立即 NO_LINK，随后从新的合法帧头恢复。

本节必须使用真实树莓派和串口线，不做回环或虚拟在线测试。

### F. 模拟视觉→C板计算→QD4310短摆杆台架测试

本测试使用真实树莓派、USB-TTL、C 板、CAN1 和 QD4310，但不安装水管、钢球
和小车。树莓派发送的是模拟摄像头 `x/v/confidence/flags`，不是电机角度；
C 板仍执行正式协议解析、10 帧门槛、状态机和 PD 计算。

准备：

1. 固定电机外壳，清空摆杆运动区域，保证可以立即断电；
2. 上电后让短摆杆停在希望作为临时中心的位置；
3. 烧录并调试 `SimVisionMotorTest`；
4. 先确认：

   ```text
   ball_control_debug_sim_level_captured=1
   ball_control_debug_sim_level_stable_frames=5
   ball_control_debug_state=4
   ball_control_debug_fault=0
   ball_control_debug_motor_enabled=1
   ball_control_debug_motor_feedback_count持续增加
   ball_control_debug_tx_error_count=0
   ```

上传脚本：

```powershell
scp "F:\260801_电赛省赛\H_BallBalance_CBoard\Tools\Simulate-BallControl-RaspberryPi.py" `
  pi@192.168.10.155:/home/pi/Desktop/
```

正式场景模拟：

```bash
python3 /home/pi/Desktop/Simulate-BallControl-RaspberryPi.py
```

角度限位扫描：

```bash
python3 /home/pi/Desktop/Simulate-BallControl-RaspberryPi.py \
  --limit-scan --stage-seconds 5
```

看到提示后准确输入 `ARM MOTOR TEST`。当前扫描使用约
`8.3/16.7/25/30 mm` 的模拟球偏差，对应约 `1/2/3/3°` 的控制请求。
`SimVisionMotorTest` 专用限制是：

```text
控制偏移：±3°
临时机械保护：±4°
速度保护：20 rpm
电流保护：1 A
```

本次实机已观察到正负方向运动和 `Balancing(5)`，最大幅度符合当前裸摆杆需求。
没有记录精确实际偏转角，因此不能把“看起来合适”写成已标定的最终限位。
`fault` 非零、实际角度意外超过临时范围、剧烈运动或线束再次失联时立即
`Ctrl+C`，必要时按 KEY 并物理断电。

## 11. 机械水平零位与方向

### 11.1 读取机械水平零位

物理水管、连杆或电机支架只要重新安装，台架测试自动捕获的临时零点就全部作废。
本节必须在最终物理结构搭好后重新执行，且应先在不上车的静止支架上完成，再进行
真实钢球闭环和整车测试。

1. 首次标定或机械重装时先临时设置 `kBallControlCommissioned=false`，拆下钢球。
2. 连接并上电 QD4310；电机保持失能。
3. 人工把摆杆调到真实机械水平，用水平仪或可靠基准检查。
4. 在 Live Watch 观察：

   ```text
   ball_control_debug_actual_angle_rad
   ball_control_debug_motor_feedback_count
   ```

   - `ball_control_debug_actual_angle_rad`：QD4310 当前反馈的绝对角，单位 rad；
   - `ball_control_debug_motor_feedback_count`：有效电机反馈帧累计数，应持续增加。

5. 等角度稳定，记录多次读数，取一致值写入
   `Application/BallControlConfig.h`：

   ```cpp
   kMechanicalLevelAngleRad = 实测值  // 摆杆真实水平时的 QD4310 绝对角，单位 rad
   ```

6. 重新编译烧录，确认调试变量中的机械零位等于填写值。

#### 无水平仪时：静止小球漂移法

这种方法得到的是当前安装、轨道摩擦和局部形变共同作用下的“有效无漂移角”，不能
证明几何意义上的绝对水平，但足以作为首次闭环前的保守水平参考。机构重新安装后必须
重做。

1. 轨道两端加挡球，小车保持静止，确保能够立即断电；暂不运行视觉闭环。
2. 使用当前 `kMechanicalLevelAngleRad` 启动，在 `LevelHold(4)`，或
   `VisionRecovery(6)` 且 `ball_control_debug_angle_offset_deg` 已回到 `0` 后再放球。
3. 从 O 点及 O 点两侧约 20 mm 处分别无初速度释放钢球，每个位置重复 3 次；记录
   5 秒后的位移和方向，不能用手推球。
4. 若钢球持续向车尾正方向漂移，按当前已验证机械关系把水平角减小
   `0.002～0.005 rad`；若持续向车头负方向漂移，则把水平角增大相同幅度。机械关系
   改变后不得沿用这个增减方向，必须先重新确认角度方向。
5. 每次只改 `kMechanicalLevelAngleRad`，重新编译烧录并等待角度稳定后再测；不得同时
   修改 Kp、Kd、最大倾角或变化率。
6. 三个起点的 5 秒平均绝对漂移不大于 5 mm，且没有固定方向超过 5 mm 的连续漂移，
   可作为首次闭环水平参考。若轨道局部不直导致无法达标，记录漂移最小的角度并明确
   标注为“近似水平”，首次闭环继续使用 `±3°` 限幅和 `25°/s` 变化率限制。

当前已定机械水平角为 `1.2461 rad`。只有水管、连杆或电机安装关系改变时才重新
执行本节标定；重新标定时记录角度、每次起点、5 秒位移、漂移方向和电机抖动。

### 11.2 验证角度方向

不要凭电机安装方向猜 `kAngleSign`。本项目不提供自动点动，方向校验必须依赖
项目外的 QD4310 上位机或专用受限工具。保持钢球拆下、机构运动范围无遮挡，
在机械水平附近分别命令很小的正、负绝对角变化，观察摆杆哪一端抬高。动作幅度
不得超过当前 ±3° 保守范围，并保证能立即物理断电。

然后人工放球做极小角度方向确认：正的控制偏移必须产生预期的球运动方向。方向
相反时修改：

```cpp
kAngleSign = -1.0F  // 控制偏移方向；若球的响应方向相反就在 +1 与 -1 间切换
```

重新编译后再验证。当前装车结果是：机械水平角 `1.2461 rad`，
`kAngleSign=+1`，角度增大使球向车尾、角度减小使球向车头；物理安全范围
`0.45～1.55 rad`，命令范围 `1.10～1.40 rad`。机械关系改变后这些结果全部作废，
重新确认以前 `kBallControlCommissioned` 必须保持 `false`。

### 11.3 打开安全门

确认并记录：

- 机械水平绝对角；
- 正角度和球坐标正方向；
- QD4310 ID/CAN 和急停；
- ±3° 内机构无碰撞；
- KEY 按下能进入硬故障停机。

之后才改：

```cpp
kBallControlCommissioned = true  // 零位、方向和安全限位真机确认后才允许自动使能
```

重新编译、烧录。不要在机构未装好或零位仍是占位值 `0.0F` 时打开。

## 12. BallControlTask Live Watch

添加：

```text
ball_control_debug_state
ball_control_debug_fault
ball_control_debug_mechanical_level_rad
ball_control_debug_actual_angle_rad
ball_control_debug_target_angle_rad
ball_control_debug_angle_offset_deg
ball_control_debug_target_position_0p1mm
ball_control_debug_position_0p1mm
ball_control_debug_velocity_mmps
ball_control_debug_valid_measurements
ball_control_debug_measurement_update_count
ball_control_debug_motor_feedback_count
ball_control_debug_tx_error_count
ball_control_debug_motor_enabled
ball_control_debug_motor_speed_rpm
ball_control_debug_motor_current_a
ball_control_debug_enable_confirmations
ball_control_debug_control_elapsed_ms
ball_control_debug_task_id
ball_control_debug_run_id
```

| 变量 | 单位/取值 | 注释 |
|---|---|---|
| `ball_control_debug_state` | 0～7 | 球控状态机当前状态，编号见下表 |
| `ball_control_debug_fault` | 0～10 | 当前锁存的硬故障码，编号见故障码表；进入 Fault 后需排查并重新上电 |
| `ball_control_debug_mechanical_level_rad` | rad | 配置文件中写入的机械水平绝对角，是所有回平动作的基准 |
| `ball_control_debug_actual_angle_rad` | rad | QD4310 反馈的摆杆当前绝对角 |
| `ball_control_debug_target_angle_rad` | rad | C 板本周期准备发送给 QD4310 的绝对目标角 |
| `ball_control_debug_angle_offset_deg` | ° | 相对机械水平的控制偏移；当前限制在 `±3°` |
| `ball_control_debug_target_position_0p1mm` | 0.1 mm | 当前球位置目标；来自树莓派快照 |
| `ball_control_debug_position_0p1mm` | 0.1 mm | 当前球位置反馈；来自树莓派视觉 |
| `ball_control_debug_velocity_mmps` | mm/s | 当前球速度；速度无效时控制器发布为 `0` |
| `ball_control_debug_valid_measurements` | 0～10 | 当前连续有效的新视觉观测数；达到 `10` 才允许进入 Balancing |
| `ball_control_debug_measurement_update_count` | 次 | 控制任务看到的视觉新测量累计编号，用于识别重复图像 |
| `ball_control_debug_motor_feedback_count` | 帧 | QD4310 有效反馈帧累计数；电机在线时应持续增加 |
| `ball_control_debug_tx_error_count` | 次 | QD4310 发送失败及 CAN 错误的合计诊断数；运行中增加会触发硬故障 |
| `ball_control_debug_motor_enabled` | 0/1 | QD4310 反馈的实际使能状态，不是 C 板主观发送的使能命令 |
| `ball_control_debug_motor_speed_rpm` | rpm | QD4310 反馈转速；绝对值超过 `60 rpm` 连续 3 帧触发故障 |
| `ball_control_debug_motor_current_a` | A | QD4310 反馈电流；绝对值超过 `1.5 A` 连续 3 帧触发故障 |
| `ball_control_debug_enable_confirmations` | 0～3 | EnablePending 中连续收到 `enabled=1` 的独立反馈帧数 |
| `ball_control_debug_control_elapsed_ms` | ms | 本次控制更新距离上次更新的真实时间；正常约 `5 ms`，`0` 时斜率不推进，超过 `200 ms` 报故障 |
| `ball_control_debug_task_id` | 无符号整数 | 当前视觉快照携带的任务编号，仅用于诊断，不选择控制参数 |
| `ball_control_debug_run_id` | 无符号整数 | 当前视觉快照携带的测试轮次编号 |

状态码：

| 值 | 状态 | 行为 |
|---:|---|---|
| 0 | BootSafe | 上电先零输出和失能 |
| 1 | WaitMotor | 等至少 5 帧可靠电机反馈 |
| 2 | ConfigRequired | commissioning 未打开，保持失能 |
| 3 | EnablePending | 预置水平角，100 ms 重试使能并等 3 帧确认 |
| 4 | LevelHold | 绝对角度模式保持机械水平 |
| 5 | Balancing | 视觉滚球 PD |
| 6 | VisionRecovery | 视觉失效，限斜率回机械水平 |
| 7 | Fault | 硬故障，零输出并失能 |

故障码：

| 值 | 故障 | 含义 |
|---:|---|---|
| 0 | None | 无故障 |
| 1 | EmergencyStop | KEY 紧急停机 |
| 2 | CanTransmit | CAN 命令发送错误 |
| 3 | MotorFeedbackTimeout | 电机反馈超时 |
| 4 | InvalidMotorFeedback | 电机反馈非法 |
| 5 | MechanicalAngleLimit | 实际角度超过机械安全限值 |
| 6 | MotorEnableTimeout | 使能等待超过 1000 ms |
| 7 | MotorUnexpectedlyDisabled | 运行中电机反馈掉使能 |
| 8 | MotorOverspeed | 速度绝对值超过 60 rpm 连续 3 帧 |
| 9 | MotorOvercurrent | 电流绝对值超过 1.5 A 连续 3 帧 |
| 10 | ControlLoopTiming | 控制周期超过 200 ms |

数值来自当前 `BallBalanceController.h`；更换固件后应重新核对，不要只凭旧截图判断。

## 13. 分阶段稳球真机测试

### 阶段 1：配置安全门回归

机械重装或需要检查安全锁时，临时令 `kBallControlCommissioned=false`：

```text
state最终为ConfigRequired(2)
电机保持失能
CAN反馈、视觉和IMU仍可观察
```

### 阶段 2：只保持水平

完成第 11 节并打开安全门，但 Pi 发 `CONTROL_ENABLED=0` 或不发送：

```text
BootSafe→WaitMotor→EnablePending→LevelHold
目标角逐步到机械水平
```

正式代码确认收到 5 帧可靠反馈后先跟随当前实际角，再进入
`EnablePending(3)`；使能确认后才按 `25°/s` 向中心角移动，命令与实际角最大
相差 `0.75°`，速度超过 `30 rpm` 时暂时继续跟随实际角。
使能命令最多每 100 ms 重试一次，1000 ms 内必须得到连续 3 帧
`ball_control_debug_motor_enabled=1`，随后进入 `LevelHold(4)`。确认没有突跳、
撞限位或持续 CAN 错误。

### 阶段 3：进入滚球 PD

1. 钢球放在 O 点附近。
2. Pi 发 `target_x=0`，先清除 `VELOCITY_VALID` 并令 `v=0`。
3. 置位并持续保持 `CONTROL_ENABLED`，再连续发送新的有效观测；
   `valid_measurements` 应从 0 增到 10。
4. 第 10 个新观测后进入 `Balancing(5)`。
5. 小幅移动球，确认目标角方向正确且偏移受限。
6. 再启用有效速度和 D 项观察阻尼。

### 阶段 4：视觉恢复

分别执行：

- 清除 `CONTROL_ENABLED`；
- 遮挡球；
- 降低置信度；
- 停止 Pi 发送。

控制器应进入 `VisionRecovery(6)`，目标角按限斜率回机械水平，电机保持位置环；
不得保持最后一个非零补偿角，也不应因视觉软故障直接失能。

### 阶段 5：目标阶跃

依次测试：

```text
0 → +100 → 0 → -100（0.1 mm单位）
```

确认方向和稳定性后再逐步扩大到 `±500`，记录超调、稳定时间和最大误差。不要
一开始直接做 ±5 cm 满幅测试。

### 阶段 6：硬故障

在能够立即断电且机构安全的条件下验证 KEY 急停、反馈超时或 CAN 故障。还要用
受控方法验证使能超时、运行中掉使能、速度绝对值超过 60 rpm 连续 3 帧、电流
绝对值超过 1.5 A 连续 3 帧和控制周期超过 200 ms。预期进入 `Fault(7)`，发送
零速度/零电流并失能；安全命令只按 100 ms 限频重发。硬故障恢复按重新上电和
完整安全检查处理。

控制周期检查中，`ball_control_debug_control_elapsed_ms=0` 时状态和斜率不得推进；
斜率计算使用的 elapsed 最大为 20 ms；超过 200 ms 必须报
`ControlLoopTiming(10)`，不能用一次大 dt 跳到远处目标角。

首轮 O 点回中阶段使用纯视觉 PD。当前正式固件已经把 BMI088 前馈接入 T4/T5/T6，
但必须通过任务、`RUN_ACTIVE`、Balancing 和 IMU 新鲜度门控；轴向、重力残差和增益
仍要在整车真机中逐步确认。

当前阶段边界：

- 已完成：机械水平 `1.2461 rad`、方向/限位、正式视觉真实 `x/v` 和第一轮 O 点回中；
- 已实现待验收：任务3～6上下文、T4/T5/T6 IMU 前馈、任务4停车后
  `+4°/300 ms/700 ms` 补偿退出；
- 下一步：先验证任务4新停车补偿，再完成失效状态、任务3、任务5和任务6真机记录；
- 尚未关闭：QD4310 PID 断电持久化、可靠线束、3507直接状态通信和整车比赛验收。

## 14. 完成判据

通信真机通过：

- 只接 Pi TX→C RX+GND，50 Hz 32 字节持续接收；
- CRC 向量一致，x/v/target/task/flags/run_id 正确；
- 无球、重复图像、会话/轮次变化和错误帧行为正确；
- 80 ms 测量仍有效、81 ms 测量失效，合法帧静默超过 110 ms 后链路失联符合预期；
- 分包/粘包/噪声/环形回绕下末字节 tick 正确，ring/parser 溢出或 UART 错误
  立即 NoLink 并清流；
- C 板没有发送 UART 数据。

机械与稳球真机通过：

- 水平零位和角度方向已记录；
- 默认安全门、5 帧反馈、预置水平角、100 ms 使能重试、1000 ms 超时和 3 帧
  enabled 确认正确；
- 水平保持、KEY 急停、掉使能、过速/过流、控制周期和限频安全命令保护正确；
- 10 个新观测门槛正确；
- 视觉失效回水平、硬故障失能；
- O 点和小目标阶跃稳定后，才逐步验收 ±5 cm。

人工改造 USB-TTL 线束在更换可靠连接器前只能用于临时台架，不满足最终机械与
整车验收条件。换线后必须复测 60 秒零错误，并在不影响机构安全的前提下做轻微
晃动测试；接收停止或 UART/CRC/丢弃字节计数增加均判为失败。

代码、主机测试、静态检查和 ARM 编译只是交付前置证据，不属于以上真机判据。

## 15. CompetitionTaskGuard 现场检查

职责边界先冻结：Pi 负责目标和任务流程；C 板不生成 T3 目标。Pi 负责 T3 的
`O→+500→-500` 和 T6 指定位置锁存；C 板仅校验。T6 现场输入必须带
`TARGET_LATCHED`、目标在 `[-1000,+1000]`（0.1 mm）内，同一会话同一
`run_id` 不得改变目标。

正式 200 Hz 路径读取一次视觉快照，调用 `CompetitionTaskGuard`，复制输入后只把
`effectiveControlFlags` 写入 `control_flags`；禁止改写 Pi 的
`target_position_0p1mm`。READY 前有 `CONTROL_ENABLED` 和健康视觉即可闭环，
`RUN_ACTIVE` 不是闭环门；DONE/TIMEOUT 继续保持末目标。

Live Watch 加入九项：

- `competition_task_debug_state`
- `competition_task_debug_error`
- `competition_task_debug_context_valid`
- `competition_task_debug_task_id`
- `competition_task_debug_run_id`
- `competition_task_debug_target_0p1mm`
- `competition_task_debug_raw_control_flags`
- `competition_task_debug_effective_control_flags`
- `competition_task_debug_reject_count`

错误枚举为 `None`、`UnknownTask`、`InvalidRunId`、`InvalidIdleContext`、
`MissingControlEnable`、`ContradictoryTerminalFlags`、`TerminalWithoutRunActive`、
`TaskTargetMismatch`、`MissingTargetLatch`、`RunIdentityMismatch`。

现场按以下顺序检查：

1. 用 `vision_debug_*` 确认 Pi 原始 task/run/target/flags；
2. 比较 raw/effective flags 和 state/error/reject count；
3. READY 不置 `RUN_ACTIVE` 时确认仍可进入闭环；
4. DONE/TIMEOUT 确认保持末目标；
5. 注入非法 active T3 `target=0`，确认进入 `VisionRecovery`、持续发送限斜率回平角、
   不触发电机 disable；视觉失效和通信失联也应走同一软恢复路径；
6. 恢复合法上下文，前 9 个新视觉测量保持恢复态，第 10 个才回 `Balancing`；
7. 无 3507 状态时不得由 C 板宣告 T4/T5/T6 底盘任务完成。

### 15.1 任务 3 真机帧序列与切换判据

任务 3 全程必须保持同一 `vision_debug_pi_session_id` 和
`competition_task_debug_run_id`。依次观察：

| 阶段 | 目标 | 原始/有效标志 | 校验状态 |
|---|---:|---:|---|
| 准备/O 点 | `0` | `0x01/0x01` | 1（准备） |
| 去车尾 +5 cm | `+500` | `0x03/0x03` | 2（运行） |
| 去车头 -5 cm | `-500` | `0x03/0x03` | 2（运行） |
| 成功 | `-500` | `0x07/0x07` | 3（完成） |
| 5.0 s 超时 | `-500` | `0x0B/0x0B` | 4（超时） |

Pi 在 O、`+500`、`-500` 三处都使用同一切换标准：目标误差不大于 `3 mm`、速度
绝对值不大于 `10 mm/s`、连续 3 张不同的新鲜视觉图满足；任一新图不满足立即清零。
重复 BallLink 包不得增加计数。C 板不新建任务 3 PID，不生成目标、不判断完成，仍用
`error=target_x-x` 和同一组控制参数。2026-08-01 已接受基线为 `Kp=0.25`、
`Kd=0.145`；当前源码试验值为 `Kd=0.15`。机械水平仍是 `1.2461 rad`，正常限幅
仍为 `±3°`。

### 15.2 视觉与任务失效真机验收表

测试前先让球控进入 `Balancing(5)` 并形成一个很小的非零控制偏移。每种失效只改变
一个条件，轨道两端必须有挡球。软失效的共同通过条件是：

- `ball_control_debug_state=6`、`ball_control_debug_fault=0`；
- `ball_control_debug_angle_offset_deg` 按 `25°/s` 限制逐步回到 0；
- `ball_control_debug_motor_enabled=1`，不得发送硬失能；
- 不再沿用失效前的非零控制偏移。

| 测试 | 操作 | 额外观察 | 恢复条件 |
|---|---|---|---|
| Pi 停止发送 | 停止 Pi 发送但保持 C 板和电机上电，等待超过 110 ms | `vision_debug_status=0`，`vision_debug_timeout_count` 只增加 1 次 | 恢复合法帧后连续 10 个新测量才回 `Balancing(5)` |
| 无球 | 取走钢球，Pi 继续发送无球帧 | `vision_debug_status=1`，`BALL_VALID=0`、置信度为 0 | 重新识别球后连续 10 个新测量才回 `Balancing(5)` |
| 测量过期 | 令测量年龄为 81 ms，链路仍保持在线 | `vision_debug_status=3`；80 ms 对照帧仍应有效；81～110 ms 控制量回水平，111 ms 起进入视觉恢复 | 年龄恢复到不大于 80 ms；短暂掉线重新累计 3 个新测量，超过 300 ms 或系统级故障重新累计 10 个新测量 |
| 低置信度/未标定/处理降级 | 每次只触发一个标志条件 | `vision_debug_status=1`，原始标志能说明具体原因 | 条件清除后重新累计 10 个新测量 |
| 非法任务上下文 | T3 Active 时发送 `target=0` | `competition_task_debug_context_valid=0`、error 非 0、effective flags 的 `CONTROL_ENABLED` 被清除 | 恢复合法 T3 目标后重新累计 10 个新测量 |

每一行记录触发前状态、越过对应门限后的状态、最大回平步进、是否保持使能和恢复所用
新测量数；缺少任一记录都不能把真机失效行为标为完成。

以上代码与主机/模拟/ARM 验证不等于真机验收。机械水平、PD 真机整定、真实钢球
闭环以及任务 3～6 整车结果仍需逐项现场验证。

## 16. T4/T5/T6 正式 BMI088 前馈现场检查

正式路径由 `CompetitionImuFeedforward` 在 C 板 200 Hz 球控周期内门控。Pi 只提供
`task_id`、目标位置和运行标志；C 板 BMI088 提供实测纵向加速度。3507 当前没有向
C 板发送启动、加速、匀速、减速或停车阶段的正式协议，因此不得把 3507 内部模式或
阶段变量当成运行时输入，也不得由 C 板据此宣告任务完成。

前馈默认配置启用，但只有以下条件同时满足时才实际应用：任务上下文合法，任务为
T4/T5/T6，有效标志同时包含 `CONTROL_ENABLED` 和 `RUN_ACTIVE`，球控状态为
`BallControlState::Balancing`，视觉有效且测量年龄不超过80 ms，BMI088 已标定，
且 IMU 快照有效、有限并保持新鲜。
T4/T5/T6 的 PREPARE/READY 仍允许纯视觉 PD 闭环，但前馈为零；T2/T3、任务上下文
非法、视觉恢复、回机械水平及故障状态的前馈也必须为零。

IMU 无效或超过 `100 ms` 没有新样本时，只撤掉 IMU 和启动前馈，视觉 PD 继续工作，不清除
`CONTROL_ENABLED`，也不触发电机失能。视觉、BallLink 或任务上下文失效时，则沿用
现有软恢复：忽略前馈并按 `25°/s` 限斜率回到机械水平。IMU 恢复为新鲜有效快照后，
原 IMU 前馈可以恢复；已取消的启动项同一轮不重放。合成顺序固定为先将
`PD 偏置 + IMU 前馈偏置 + 启动项` 按当前动态角度上限限幅，
再进入现有安装角变化率和绝对角安全限制。普通上限仍为 `±3°`；任务4停车补偿保持/
退出阶段临时允许总偏移到4°；新增启动窗口也临时允许总偏移到 `±4°`，结束后恢复原动态上限。

当前机械水平角为 `1.2461 rad`、`Kp=0.25`。已接受阻尼基线是 `Kd=0.145`，当前源码
试验值是 `Kd=0.15`。普通控制总偏置为 `±3°`，安装角变化率为 `25°/s`；任务4
停车补偿阶段和新增启动窗口临时允许4°；原静摩擦脱困逻辑也保留。新增启动参数尚未真机验收。

### 16.1 Live Watch 诊断

正式前馈联调至少同时观察：

| 变量 | 含义 |
|---|---|
| `imu_feedforward_debug_configured_enabled` | 功能是否在固件配置中默认启用；不等于本周期已应用 |
| `imu_feedforward_debug_gate_reason` | 本周期门控结果：0 已应用；1 配置禁用；2 任务上下文非法；3 任务不支持；4 未使能控制；5 尚未 RUN_ACTIVE；6 非 Balancing；7 IMU 未就绪；8 IMU 无效；9 IMU 非有限值；10 IMU 过期；11 控制输出非法 |
| `imu_feedforward_debug_ready` | IMU 估计器已就绪且完成标定 |
| `imu_feedforward_debug_valid` | 当前保留快照有效 |
| `imu_feedforward_debug_sample_fresh` | 当前样本未超过新鲜度门限 |
| `imu_feedforward_debug_sample_age_ms` | 当前样本年龄，单位 ms |
| `imu_vehicle_output_count` | IMU 估计器发布的新样本计数，应随采样持续增长 |
| `imu_feedforward_debug_accel_mps2` | 用于正式门控的过滤后纵向加速度 |
| `imu_feedforward_debug_angle_offset_deg` | IMU 前馈单项偏置 |
| `imu_feedforward_debug_pd_offset_deg` | 合成前视觉 PD 偏置 |
| `imu_feedforward_debug_total_offset_deg` | 按当前动态上限限幅后的 PD+原前馈+启动项总偏置 |
| `imu_feedforward_debug_target_angle_rad` | 合成后、安装角限斜率前的绝对目标角 |
| `imu_feedforward_debug_invalid_cycle_count` | 因 IMU 未就绪、无效或非有限值而撤前馈的累计周期数 |
| `imu_feedforward_debug_stale_cycle_count` | 因 IMU 样本过期而撤前馈的累计周期数 |

原有 `ball_control_debug_imu_feedforward_mode` 只表示
`HBALL_IMU_FEEDFORWARD_TEST` 独立固件模式，不能代替正式路径的配置与门控诊断。

### 16.2 分级真机步骤

1. 架空小车，选择 T4/T5/T6 并停在 READY，确认纯视觉 PD 可闭环，且
   `imu_feedforward_debug_gate_reason=5`、前馈偏置为零。
2. 置 `RUN_ACTIVE` 后手动前后加速车体，确认正向加速度产生负前馈角，负向加速度产生
   正前馈角；同时确认样本计数持续增长、样本年龄不越过 `100 ms`。
3. 架空运行 3507 模式 2 和模式 3，记录起步、匀速、减速、停车阶段的 IMU 连续性。
   行为参考为：模式 2 约 5 s 加速、15 s 匀速、7.6 s 减速；模式 3 约 2 s 加速、
   5 s 匀速、1 s 减速。模式 1 当前为速度阶跃，需单独检查冲击。
4. 落地无球低速测试，确认 `imu_feedforward_debug_total_offset_deg` 不超过当前动态角度
   上限，最终安装角命令仍满足 `25°/s` 变化率限制。
5. 加装两端挡球后低速放球，先验收真实 O 点闭环和 PD 真机整定，再逐步增加车体加速度。
6. 最后依次验收任务 4 的 A→B、任务 5 的整圈 O 点保持、任务 6 的整圈指定位置保持。

T6 指定目标由 Pi 在创建本轮任务时锁存，C 板只校验 `TARGET_LATCHED`、目标范围
`[-1000,+1000]`（0.1 mm 单位）以及同一 Pi 会话、同一 `run_id` 内目标不可变化；
C 板不自行生成指定位置。上述代码、主机测试、静态检查和 ARM 构建结果均不等于任何
一项真机验收完成。

### 16.3 任务4停车补偿专项

当前源码参数位于 `Application/BallControlConfig.h`：

```cpp
kTask4StartFeedforwardGain = 10.0F;
kTask4BrakeFeedforwardGain = 10.0F;
kTask4MaxFeedforwardOffsetDeg = 20.0F;
kTask4BrakeDetectAccelerationMps2 = -0.15F;
kTask4StopDetectAccelerationMps2 = -0.05F;
kTask4PostStopOffsetDeg = 4.0F;
kTask4PostStopHoldMs = 300U;
kTask4PostStopReleaseMs = 700U;
```

`20°` 是任务4普通前馈控制器的人工配置上限，普通 PD+前馈合成仍受球控动态总限幅，
不能把它当作实际命令角。刹车过程中先检测加速度低于 `-0.15 m/s²`，再在加速度恢复
到 `-0.05 m/s²` 以上时触发停车补偿：输出 `+4°`、保持 `300 ms`，然后在
`700 ms` 内线性减到零。新的任务、非 T4、任务/通信/IMU失效都会清除该状态。

现场只改一个参数后重新编译和烧录，依次记录：是否触发、触发是否过早/过晚、球的
最大位移、是否反向过冲、700 ms退出后能否由 PD 重新回中。该状态机已编译，但截至
2026-08-06尚未收到真机通过记录。

## 17. T4/T5/T6 启动补偿专项（2026-10-04）

本次按“先启动 Pi 任务，再单独按小车键”的现场顺序设计。Pi 的 `RUN_ACTIVE` 只让
`VehicleLaunchFeedforwardController` 等待；两个不同的新鲜 IMU 输出的正向加速度
连续达到 `0.06 m/s²` 才触发。初始参数为 `-2°`、保持100 ms、线性退出700 ms，
启动期间 PD+原前馈+启动项总偏移限制为 `±4°`。最终安装角仍通过既有跟随、限斜率
和绝对角保护。参数集中在 `Application/BallControlConfig.h` 的 `kVehicleLaunch*`。

至少观察 `vehicle_launch_debug_state`、`vehicle_launch_debug_active`、
`vehicle_launch_debug_offset_deg`、`vehicle_launch_debug_elapsed_ms`、
`vehicle_launch_debug_trigger_count`、`imu_feedforward_debug_accel_mps2`、
`imu_feedforward_debug_total_offset_deg` 与 `ball_control_debug_last_sent_angle_rad`。
状态值为0禁用、1等待、2连续确认、3补偿中、4已用完、5已取消；触发数为累计值。

1. Pi ACTIVE 后静止等待几秒，确认启动项保持零、累计触发数不增加。
2. 按小车键，记录加速度、补偿触发、最终命令角三个时刻，以及球的最大偏移和反向过冲。
3. 同一轮补偿退出后不能重放；过滤后加速度出现负值时立即撤掉启动项，原刹车前馈继续工作。
4. 确认 IMU 失效只撤前馈、PD继续；视觉/任务失效沿用安全回平；恢复后启动项不在同一轮重新触发。
5. 新开一轮确认能再次触发，再分别验证 T5/T6 和原 T4 停车补偿。

详见 [启动补偿说明](STARTUP_FEEDFORWARD.md) 中的调参顺序、最近8轮去重范围、构建命令
和验证记录。实际起步检测包含采样/过滤延迟，若触发明显太晚，需要小车启动信号才能提前预倾。
