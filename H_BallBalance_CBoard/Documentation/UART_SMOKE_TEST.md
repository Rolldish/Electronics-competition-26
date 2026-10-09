# C 板最小 UART 测试

测试固件只初始化系统时钟、RGB LED 和 USART6，不启动 FreeRTOS、滚球控制、
CAN、IMU、DMA 或正式 BallLink 协议。

本文件只用于最小 `PING/PONG` 双向排线测试。正式 BallLink 是 Pi→C 单向链路，
使用树莓派 USB→CH340 USB-TTL→C 板 PG9，只接 USB-TTL TX 和 GND；不要把
本文件中的 RX/PONG 接线照搬到正式滚球通信。

截至 2026-08-06，本测试仍只用于排线诊断；正式 BallLink 合法帧静默超时已经统一为
`110 ms`，任务4前馈和停车补偿不参与本测试。正式进度见 `CURRENT_PROGRESS.md`。

## 编译与烧录

在项目目录的 PowerShell 中执行：

```powershell
. .\Tools\Enter-CBoardEnv.ps1
cmake --preset UartTest
cmake --build --preset UartTest
.\Tools\Flash-UartTest.ps1
```

烧录并复位后：

- 蓝灯常亮：最小测试固件正在运行；
- 每收到一次完整 `PING\r\n`：绿灯翻转一次；
- C 板同时从 PG14 回发 `PONG\r\n`；
- 红灯亮：HAL 报告 UART 收发错误。

如果烧录时报：

```text
libusb_open() failed with LIBUSB_ERROR_ACCESS
```

表示 ST-Link 正被 VS Code 调试会话、OpenOCD 或 GDB 占用。先停止调试，再执行：

```powershell
Get-Process openocd,arm-none-eabi-gdb -ErrorAction SilentlyContinue |
  Stop-Process
```

该错误与 UART 杜邦线接触不良无关。

## 使用串口助手单独检查 C 板

必须使用 3.3 V TTL 转串口模块，不要使用 RS-232 电平，也不要连接 VCC：

- 模块 TX → C 板 PG9（USART6_RX）
- 模块 RX ← C 板 PG14（USART6_TX）
- 模块 GND ↔ C 板 GND

串口助手设置为 `115200`、8 数据位、无校验、1 停止位、无流控。

以 ASCII 方式发送 `PING` 并开启“追加 CRLF”，或者直接以十六进制发送：

```text
50 49 4E 47 0D 0A
```

应收到：

```text
PONG
```

对应十六进制为：

```text
50 4F 4E 47 0D 0A
```

## 当前线束风险和排查方法

模拟视觉电机联调时，人工改造的 USB-TTL→C 板杜邦线曾发生间歇性失联：

- Pi 仍显示约 50 Hz 发送；
- C 板 `vision_debug_rx_byte_count`、`rx_frame_count` 和
  `accepted_frame_count` 停止增加；
- 触碰线束后立即恢复；
- 恢复期间 UART、CRC 和丢弃字节计数累计增加；
- C 板看门狗刷新正常、致命码为 0，初步排除控制程序卡死。

因此，若最小 UART 测试也出现绿灯不翻转或 PONG 间歇丢失，按以下顺序排查：

1. 断电后重新插紧 TX、RX、GND，确认端子没有松针或虚焊；
2. 检查 USB-TTL 与 C 板可靠共地；
3. 固定线束并消除 USB-TTL 自重造成的拉扯；
4. 先静止连续发送，再轻微晃动线束，观察是否复现；
5. 最终更换为焊接、正规压接或带锁扣连接器。

树莓派正式单向链路当前物理口约定为 WiFi 使用左上 USB 3.0、USB-TTL 使用右下
USB 2.0，软件路径为：

```text
/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
```

换物理 USB 口通常不要求修改 by-id 路径。不要把 `/dev/ttyUSB0` 写死进正式配置。

## 恢复正式固件

最小 UART 固件与正式固件使用不同构建目录。测试结束后必须先重新配置并构建
正式 Debug，再烧录，避免把旧 ELF 当成当前正式固件：

```powershell
. .\Tools\Enter-CBoardEnv.ps1
cmake --preset Debug
cmake --build --preset Debug --parallel
.\Tools\Flash-CBoard.ps1
```

也可以在 VS Code 中执行 `C Board: Flash with ST-Link`，该任务会先运行
`C Board: Build Debug`。恢复后按正式调试流程选择
`C Board: Debug with ST-Link`：第一次 `F5` 下载并停在 `main()`，第二次 `F5`
继续运行 FreeRTOS。

正式固件启用 IWDG，但配置了调试暂停冻结；停在 `main()` 或普通断点时看门狗
同步暂停，继续运行后仅 `BallControlTask` 喂狗。因此 F5 调试与断点兼容。注意
正式固件的红灯含义不同于最小 UART 固件。当前装车配置已经 commissioned；
只有人为关闭安全门时才会出现 `ConfigRequired(2)` 且 `fault=0`。无论哪种配置，
都不能仅凭红灯判断 UART 错误。

如果下一步要继续短摆杆模拟视觉测试，而不是恢复正式安全固件，则执行：

```powershell
. .\Tools\Enter-CBoardEnv.ps1
cmake --preset SimVisionMotorTest
cmake --build --preset SimVisionMotorTest --parallel
.\Tools\Flash-SimVisionMotorTest.ps1
```

该测试固件会自动捕获临时零点并使能电机；必须固定电机、拆下钢球并清空摆杆
运动区域。正式物理结构装好后仍需重新标定水平零点，测试固件的临时零点不可复用。
