# K230 视觉模块

本目录是送药小车的 CanMV K230 视觉部分，与仓库中的 `small_car/`
MSPM0G3507 控制工程配合使用。

主要功能：

- KPU 数字 1～8 检测；
- 红线巡线与路口判断；
- 通过 UART1 向 G3507 发送数字、巡线误差和路口信息；
- 提供红线标定、采图和独立巡线调试工具。

## 目录

```text
K230_Vision/
├── main.py
├── mp_deployment_source/
│   ├── deploy_config.json
│   └── best_AnchorBaseDet_can2_5_s_20260722141453.kmodel
├── drug_car/
│   └── config.py
├── tools/
│   ├── calibrate_red.py
│   ├── capture_detection_images.py
│   └── red_line_debug.py
└── docs/
```

训练图片、标注数据和训练过程文件仅保留在本地，没有上传到本仓库。

## 部署到 K230

将以下内容复制到 SD 卡：

```text
K230_Vision/main.py                 -> /sdcard/main.py
K230_Vision/mp_deployment_source/   -> /sdcard/mp_deployment_source/
```

目录名和文件名需要保持不变。`main.py` 会读取
`/sdcard/mp_deployment_source/deploy_config.json`，再加载配置中指定的
`.kmodel` 文件。

程序使用 UART1，TX 为 GPIO9，RX 为 GPIO10，波特率为 115200。

## 调试工具

- `tools/calibrate_red.py`：红色 LAB 阈值标定。使用时同时复制
  `drug_car/` 到 `/sdcard/drug_car/`。
- `tools/capture_detection_images.py`：采集完整画面，用于后续训练。
- `tools/red_line_debug.py`：不启动 KPU 的独立红线巡线调试。

串口数据格式和红线标定记录见 `docs/`。正式运行前请在实际车辆、
光照环境和串口连线条件下完成验证。
