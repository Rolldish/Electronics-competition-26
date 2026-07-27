# -*- coding: utf-8 -*-
"""
K230 数字识别 + 红线巡线一体化程序。

程序在同一个 PipeLine 中完成：
  1. 在 RGB888P 通道上使用 KPU 检测数字；
  2. 在原生 640x480 RGB565 通道 1 上检测红线与路口；
  3. 将数字框、巡线结果和调试信息叠加到 LCD；
  4. 通过 UART1 向 G3507 发送巡线、路口和数字消息。

请在 CanMV IDE 中运行。
SD 卡需要存放：
  /sdcard/mp_deployment_source/deploy_config.json
  以及配置文件指定的 .kmodel 模型文件。
"""

import os, gc, time, math
from libs.PlatTasks import DetectionApp
from libs.PipeLine import PipeLine
from libs.Utils import *
from machine import UART, FPIOA
from media.sensor import CAM_CHN_ID_1

# ==================== AI 数字检测配置 ====================
# 模型类别编号到实际数字的映射。
# 例如模型输出类别 0，实际代表数字 6。
DIGIT_MAP = {0: 6, 1: 1, 2: 2, 3: 3, 4: 4, 5: 5, 6: 7, 7: 8}

# LCD 显示模式。
display_mode = "lcd"
# AI 检测链路采集的 RGB888P 图像尺寸：[宽, 高]。
# 注意：这不是模型自身的输入尺寸；模型输入尺寸由下方 model_size 指定。
RGB888P_SIZE = [1280, 720]
# SD 卡中的部署资源根目录，不是完整模型文件路径。
ROOT_PATH = "/sdcard/mp_deployment_source/"

# 读取部署配置，再由“根目录 + 配置中的相对路径”拼出真正的模型路径。
deploy_conf = read_json(ROOT_PATH + "/deploy_config.json")
kmodel_path = ROOT_PATH + deploy_conf["kmodel_path"]
labels = deploy_conf["categories"]              # 模型类别名称列表
conf_thr   = deploy_conf["confidence_threshold"]  # 置信度阈值
nms_thr    = deploy_conf["nms_threshold"]       # NMS 重叠框抑制阈值
model_size = deploy_conf["img_size"]            # 模型实际输入尺寸
model_type = deploy_conf["model_type"]          # 检测模型类型
anchors = []
if model_type == "AnchorBaseDet":
    # AnchorBaseDet 使用三组特征层锚框，DetectionApp 需要一维锚框列表。
    anchors = deploy_conf["anchors"][0] + deploy_conf["anchors"][1] + deploy_conf["anchors"][2]

# ==================== 红线检测配置 ====================
# 红线检测专用通道的图像尺寸：宽 640、高 480。
# 这些参数已针对原生 RGB565 通道 1 做过硬件验证。
LINE_W, LINE_H = 640, 480
# LAB 红色阈值：(L最小, L最大, A最小, A最大, B最小, B最大)。
# 光照或红线材质变化时，通常需要重新标定这组参数。
RED_LAB = (11, 74, 15, 55, -7, 39)
# 每个 ROI 的格式都是 (左上角x, 左上角y, 宽度, 高度)。
# 图像坐标原点位于左上角，列表按“离车最近 -> 最远”排列。
LINE_ROIS = (
    (0, 355, 640, 125),  # 第0层：画面底部，离小车最近
    (0, 265, 640, 90),   # 第1层
    (0, 195, 640, 70),   # 第2层
    (0, 135, 640, 60),   # 第3层
    (0, 85, 640, 50),    # 第4层
    (0, 40, 640, 45),    # 第5层
    (0, 0, 640, 40),     # 第6层：画面顶部，离小车最远
)
# 每层 find_blobs 所要求的最少像素数/面积；远处红线较小，阈值逐层降低。
LINE_ROI_MIN_PIXELS = (80, 60, 50, 40, 30, 25, 20)
# 直线拟合权重；近处红线对车辆当前转向更重要，因此权重更大。
LINE_ROI_WEIGHTS = (7, 6, 5, 4, 3, 2, 1)
# 以下参数供单红块兼容辅助函数使用，当前主循环主要采用多层 ROI 算法。
LINE_MIN_PX  = 80
LINE_MIN_AR  = 80
LINE_MIN_W   = 5
LINE_MIN_H   = 5
LINE_CONF_PX = 2000
# 输出误差统一到 640 像素宽的坐标系。
LINE_ERROR_W = 640
# 摄像头光轴与车体机械中心的安装偏差补偿，单位：像素。
LINE_CENTER_BIAS = -6
# 输出给G3507的误差绝对值上限。超过该范围只会让转向长期饱和，
# 不再提供有效的方向信息。
LINE_ERROR_ABS_LIMIT = 50
# 相邻两帧误差允许的最大变化量，过滤底部ROI突然变宽造成的边缘外推尖峰。
LINE_ERROR_STEP_LIMIT = 20
# 相邻 ROI 层之间允许的最大中心横向跳变，超过则认为不是同一条线。
LINE_MAX_CENTER_JUMP = 80
# 至少需要3个有效层，才能正常拟合巡线轨迹。
LINE_MIN_VALID_LAYERS = 3
# 轨迹开始后，最多允许连续缺失1层。
LINE_MAX_MISSING_LAYERS = 1
# 红色块宽度达到画面宽度的45%时，将其视为路口候选。
JUNCTION_WIDTH = 0.45
# 路口前方至少出现多少层窄红线，才算“可以继续直行”。
JUNCTION_CONTINUATION_LAYERS = 1
# 判断前方延续线时，候选中心与预测中心允许的最大偏差。
JUNCTION_FORWARD_MAX_JUMP = 160
# 入射线交点左右至少延伸80像素，才认为相应方向存在支路。
JUNCTION_BRANCH_EXTENT = 80
# 路口连续出现3帧后才确认，避免单帧误识别。
JUNCTION_DBNC = 3
# 路口连续消失3帧后才解除锁定。
JUNCTION_CLEAR_DBNC = 3
# 路口仍可见时每50 ms上报一次坐标，减小车辆行进造成的触发位置抖动。
JUNCTION_TX_PERIOD = 50

# ==================== UART1 初始化 ====================
# 板载接口：3号脚 TX(GPIO9)，4号脚 RX(GPIO10)；通信格式为 115200 8N1。
fm = FPIOA(); fm.set_function(9, fm.UART1_TXD); fm.set_function(10, fm.UART1_RXD)
uart = UART(UART.UART1, 115200, bits=8, parity=0, stop=1)

# ==================== 摄像头 PipeLine 与 KPU 初始化 ====================
pl = PipeLine(rgb888p_size=RGB888P_SIZE, display_mode=display_mode)
# 额外创建 640x480 的通道1，专门给红线颜色识别使用。
pl.create(ch1_frame_size=[LINE_W, LINE_H])
disp_size = pl.get_display_size()   # LCD 实际显示尺寸：(800, 480)

# 创建目标检测应用，并配置从 1280x720 图像到模型输入尺寸的预处理。
det_app = DetectionApp("video", kmodel_path, labels, model_size,
                       anchors, model_type, conf_thr, nms_thr,
                       RGB888P_SIZE, disp_size, debug_mode=0)
det_app.config_preprocess()

# ==================== 跨帧状态 ====================
target_digit = None       # G3507指定的目标病房数字，尚未指定时为 None
uart_rx_buffer = ""       # UART分包/粘包解析缓存
junction_candidate = None # 当前尚未通过连续帧确认的路口类型
junction_cnt = 0          # 同一路口候选连续出现的帧数
active_junction = None    # 已确认并锁存的路口类型
junction_clear_cnt = 0    # 已确认路口连续消失的帧数
last_junction_tx_ms = 0   # 上一次发送路口消息的时间
active_junction_y = 0     # 当前路口横向红线的纵坐标
prev_line_center = LINE_W // 2  # 上一帧预测的车前红线中心
prev_line_error = 0        # 上一帧已过滤的误差，用于抑制视觉单帧尖峰
last_tx_ms = 0            # 上一次发送 LINE/LOST 的时间
TX_PERIOD  = 40           # LINE 最快发送周期：40 ms（约25 Hz）
NUM_TX_PERIOD = 100       # NUM 最快发送周期：100 ms
ROOM_TX_PERIOD = 100      # ROOM 最快发送周期：100 ms
last_num_tx_ms = 0        # 上一次发送 NUM 的时间
last_room_tx_ms = 0       # 上一次发送 ROOM 的时间

fc = 0                    # 主循环帧计数器
lost_frames = 0           # 连续丢线帧数
lost_printed = False      # 避免重复打印“丢线”日志
line_err_shown = False    # 红线异常只打印一次，防止刷屏
fps_start_ms = time.ticks_ms()  # FPS统计起始时间
fps_frames = 0            # 当前统计窗口内的帧数
fps_text = ""             # LCD上显示的FPS文字

# 按 $名称,参数...# 格式组帧，通过 UART 发送并输出调试日志。
def tx(name, *args):
    frame = "$" + ",".join([name] + [str(a) for a in args]) + "#"
    uart.write(frame)
    print("[TX] " + frame)

# 读取并解析 G3507 发来的 PING 或 SET_TARGET 命令。
def poll_target_command():
    global uart_rx_buffer, target_digit
    try:
        if not uart.any():
            return
        data = uart.read()
        if not data:
            return
        try:
            text = data.decode("utf-8")
        except Exception:
            text = str(data, "utf-8")
    except Exception as exc:
        print("[UART RX] error:", exc)
        return

    # UART一次读取不一定正好得到完整帧，因此先追加到缓存。
    uart_rx_buffer += text
    if len(uart_rx_buffer) > 128:
        # 缓存异常增长时，只保留最后一个“$”之后的潜在有效帧。
        last_start = uart_rx_buffer.rfind("$")
        uart_rx_buffer = (
            uart_rx_buffer[last_start:] if last_start >= 0 else "")

    while True:
        start = uart_rx_buffer.find("$")
        if start < 0:
            uart_rx_buffer = ""
            return
        end = uart_rx_buffer.find("#", start + 1)
        if end < 0:
            uart_rx_buffer = uart_rx_buffer[start:]
            return

        # 一次循环只取出一个完整的 $...# 帧，缓存中剩余内容继续解析。
        payload = uart_rx_buffer[start + 1:end]
        uart_rx_buffer = uart_rx_buffer[end + 1:]
        parts = payload.split(",")

        # 冷启动握手：G3507持续发送 $PING#，K230用 $READY# 明确应答。
        # 这样任意一块板单独重启后，都能重新确认通信已经恢复。
        if len(parts) == 1 and parts[0] == "PING":
            tx("READY")
            print("[UART RX] ping -> ready")
            continue

        # SET_TARGET 只接受一个参数，且目标数字必须位于1～8。
        if len(parts) != 2 or parts[0] != "SET_TARGET":
            continue
        try:
            digit = int(parts[1])
        except Exception:
            continue
        if 1 <= digit <= 8:
            target_digit = digit
            tx("TARGET", target_digit, 0)
            print("[UART RX] target=%d" % target_digit)

# 把模型原始输出转换为便于业务使用的数字检测结果列表。
def build_digit_detections(boxes, scores, indices):
    detections = []
    if not boxes or scores is None or indices is None:
        return detections

    # 三组输出理论上等长；取最小长度可避免异常输出导致越界。
    count = min(len(boxes), len(scores), len(indices))
    for i in range(count):
        try:
            box = boxes[i]
            digit = DIGIT_MAP.get(int(indices[i]))
            if digit is None or len(box) < 4:
                continue
            x1 = int(box[0])
            x2 = int(box[2])
            confidence = min(100, max(0, int(float(scores[i]) * 100)))
        except Exception:
            continue
        detections.append({
            "digit": digit,
            "confidence": confidence,
            "center_x": (x1 + x2) // 2,
            "direction": 0,  # -1=左，0=中间/未知，1=右
        })

    # 先按目标中心从左到右排序，再给每个数字划分方向。
    detections.sort(key=lambda item: item["center_x"])
    if len(detections) == 1:
        # T字路口边缘可能只剩一个病房牌进入画面。
        # 使用绝对屏幕区域判断左右：左40%为左、右40%为右；
        # 中间20%作为死区，保证居中的起点数字方向仍为0。
        center_x = detections[0]["center_x"]
        if center_x < int(RGB888P_SIZE[0] * 0.40):
            detections[0]["direction"] = -1
        elif center_x > int(RGB888P_SIZE[0] * 0.60):
            detections[0]["direction"] = 1
    else:
        # 多个数字时，按数量把左半部分标为左、右半部分标为右。
        # 若数量为奇数，正中间的一个会保留 direction=0。
        half = len(detections) // 2
        for i in range(len(detections)):
            if i < half:
                detections[i]["direction"] = -1
            elif i >= len(detections) - half:
                detections[i]["direction"] = 1
    return detections

# 选择与目标数字一致、且方向已经明确的最高置信度病房牌。
def select_target_room(detections, wanted_digit):
    if wanted_digit is None or not detections:
        return None
    matches = []
    for item in detections:
        if (item["digit"] == wanted_digit
                and item["direction"] in (-1, 1)):
            matches.append(item)
    if not matches:
        return None
    return max(matches, key=lambda item: item["confidence"])

# 将一个坐标值按比例从源尺寸映射到目标尺寸。
def map_value(value, source_size, target_size):
    if source_size <= 0:
        raise ValueError("source_size must be positive")
    return int(value * target_size // source_size)

# 将点坐标从摄像头图像坐标系映射到LCD坐标系。
def map_point(point, source_width, source_height,
              target_width, target_height):
    return (
        map_value(point[0], source_width, target_width),
        map_value(point[1], source_height, target_height),
    )

# 将 (x, y, 宽, 高) 矩形从源坐标系映射到目标坐标系。
def map_rect(rect, source_width, source_height,
             target_width, target_height):
    x, y, width, height = rect
    return (
        map_value(x, source_width, target_width),
        map_value(y, source_height, target_height),
        map_value(width, source_width, target_width),
        map_value(height, source_height, target_height),
    )

# 旧版单层算法辅助函数：选择尺寸合格且像素数最多的色块。
def select_main_blob(blobs, min_width, min_height):
    best = None
    best_pixels = -1
    if not blobs:
        return None
    for blob in blobs:
        try:
            if blob.w() < min_width or blob.h() < min_height:
                continue
            pixels = blob.pixels()
        except Exception:
            continue
        if pixels > best_pixels:
            best = blob
            best_pixels = pixels
    return best

# 构造统一的“巡线无效/丢线”结果，避免调用方访问缺失字段。
def _lost_result():
    return {
        "valid": False,
        "cx": 0,
        "cy": 0,
        "error": 0,
        "angle": 0,
        "confidence": 0,
        "cross": False,
        "rect": None,
        "pixels": 0,
        "area": 0,
        "points": [],
        "selected_rects": [],
        "valid_layers": 0,
        "junction": None,
    }

# 旧版单色块结果构造函数。
# 当前主循环使用 build_multi_line_result()；保留本函数供兼容和测试使用。
def build_line_result(blob, source_width, error_width,
                      confidence_pixels, cross_width_ratio):
    if blob is None:
        return _lost_result()
    if source_width <= 0 or error_width <= 0 or confidence_pixels <= 0:
        return _lost_result()
    try:
        cx = blob.cx()
        cy = blob.cy()
        width = blob.w()
        height = blob.h()
        pixels = blob.pixels()
        rect = blob.rect()
    except Exception:
        return _lost_result()

    # 将色块中心统一换算到 error_width 坐标系，再计算相对中心的误差。
    normalized_cx = map_value(cx, source_width, error_width)
    confidence = min(100, max(0, pixels * 100 // confidence_pixels))
    return {
        "valid": True,
        "cx": cx,
        "cy": cy,
        "error": normalized_cx - error_width // 2,
        "angle": 0,
        "confidence": confidence,
        "cross": width >= int(source_width * cross_width_ratio),
        "rect": rect,
        "pixels": pixels,
        "area": width * height,
        "points": [(cx, cy)],
        "selected_rects": [rect],
        "valid_layers": 1,
        "junction": None,
    }

# 把 CanMV blob 对象转换成普通字典，便于后续算法和测试处理。
def _blob_info(blob):
    try:
        return {
            "blob": blob,
            "cx": int(blob.cx()),
            "cy": int(blob.cy()),
            "w": int(blob.w()),
            "h": int(blob.h()),
            "pixels": int(blob.pixels()),
            "rect": blob.rect(),
        }
    except Exception:
        return None

# 从近到远在各 ROI 中选择属于同一条红线的色块。
# previous_center 是上一帧车前红线中心，用于维持时间连续性；
# 返回列表与 layers 等长，没有合适色块的层使用 None 占位。
def _select_layer_track(layers, previous_center):
    selected = []
    expected_x = previous_center  # 当前层期望出现的横坐标
    last_x = None                 # 上一个窄红线色块的中心
    missing = 0                   # 轨迹开始后的连续缺失层数
    started = False               # 是否已经找到第一段有效窄红线
    wide_limit = int(LINE_W * JUNCTION_WIDTH)

    for layer in layers:
        candidates = layer["candidates"]
        chosen = None
        if candidates:
            # 在当前层中选择最接近期望横坐标的候选色块。
            nearest = min(
                candidates, key=lambda item: abs(item["cx"] - expected_x))
            if (not started
                    or abs(nearest["cx"] - expected_x)
                    <= LINE_MAX_CENTER_JUMP):
                chosen = nearest

        if chosen is None:
            selected.append(None)
            if started:
                # 轨迹中间可以短暂缺一层；连续缺失过多则停止向远处搜索。
                missing += 1
                if missing > LINE_MAX_MISSING_LAYERS:
                    break
            continue

        # 路口色块可能非常不对称，特别是最底层的左/右T字支路。
        # 此时宽色块的几何中心并不等于入射直线中心：
        # 保留它用于路口分类，但不让它改变上层窄红线的预测方向。
        if chosen["w"] >= wide_limit:
            selected.append(chosen)
            continue

        started = True
        missing = 0
        selected.append(chosen)
        if last_x is None:
            expected_x = chosen["cx"]
        else:
            # 使用最近两层中心的变化量，线性外推下一层中心位置。
            expected_x = chosen["cx"] + (chosen["cx"] - last_x)
        expected_x = min(LINE_W - 1, max(0, expected_x))
        last_x = chosen["cx"]

    while len(selected) < len(layers):
        selected.append(None)
    return selected

# 对选中的多层红线中心做加权直线拟合。
# 拟合形式为 x = slope*y + intercept，返回画面底边的预测横坐标
# bottom_x 和红线角度 angle。近处 ROI 使用更大的拟合权重。
def _fit_track(selected, min_valid_layers=LINE_MIN_VALID_LAYERS):
    fit_items = []
    wide_limit = int(LINE_W * JUNCTION_WIDTH)
    for i in range(len(selected)):
        item = selected[i]
        if item is not None and item["w"] < wide_limit:
            fit_items.append((item, LINE_ROI_WEIGHTS[i]))

    # 路口处的宽红块不适合直接代表巡线中心。
    # 只有窄红线点不足时，才以最低权重1加入宽红块，尝试维持可用结果。
    if len(fit_items) < min_valid_layers:
        for i in range(len(selected)):
            item = selected[i]
            if item is not None and item["w"] >= wide_limit:
                fit_items.append((item, 1))

    if len(fit_items) < min_valid_layers:
        return None

    # 以下变量是加权最小二乘直线拟合所需的累加量。
    sw = sx = sy = syy = syx = 0.0
    for item, weight in fit_items:
        x = float(item["cx"])
        y = float(item["cy"])
        w = float(weight)
        sw += w
        sx += w * x
        sy += w * y
        syy += w * y * y
        syx += w * y * x

    denom = sw * syy - sy * sy
    if abs(denom) < 0.001:
        # y坐标几乎相同，无法可靠计算斜率时按竖直巡线处理。
        slope = 0.0
        intercept = sx / max(sw, 1.0)
    else:
        slope = (sw * syx - sy * sx) / denom
        intercept = (sx - slope * sy) / sw

    # 把拟合线外推到画面最底部，得到最接近车体的红线中心。
    bottom_x = int(slope * (LINE_H - 1) + intercept)
    bottom_x = min(LINE_W - 1, max(0, bottom_x))
    angle = int(math.atan(slope) * 180.0 / math.pi)
    return bottom_x, angle

# 拟合进入路口的窄红线，并预测它与横向支路相交时的横坐标。
def _predict_entry_line(selected, junction_index, junction_y, wide_limit):
    points = []
    for i in range(junction_index):
        item = selected[i]
        if item is not None and item["w"] < wide_limit:
            points.append(item)

    # 侧向T字路口可能已经占满最靠近车体的ROI，导致路口下方没有入射线点。
    # 此时改用路口上方仍可见的窄红线预测交点，而不是直接判定丢线。
    if not points:
        for i in range(junction_index + 1, len(selected)):
            item = selected[i]
            if item is not None and item["w"] < wide_limit:
                points.append(item)
                if len(points) >= 3:
                    break
    if not points:
        return None

    # 最接近路口的3个点足以估算入射线，即使摄像头存在一定倾斜。
    points = points[-3:]
    if len(points) == 1:
        return float(points[0]["cx"]), 0.0

    count = float(len(points))
    sx = sy = syy = syx = 0.0
    for item in points:
        x = float(item["cx"])
        y = float(item["cy"])
        sx += x
        sy += y
        syy += y * y
        syx += y * x
    denom = count * syy - sy * sy
    if abs(denom) < 0.001:
        slope = 0.0
        intercept = sx / count
    else:
        slope = (count * syx - sy * sx) / denom
        intercept = (sx - slope * sy) / count
    return slope * float(junction_y) + intercept, slope

# 统计路口上方有多少层窄红线与预测的前进方向一致。
def _forward_continuation_count(
        layers, junction_index, junction_y, entry_x, slope, wide_limit):
    count = 0
    for i in range(junction_index + 1, len(layers)):
        best_delta = None
        for item in layers[i]["candidates"]:
            if item["w"] >= wide_limit:
                continue
            # 沿入射线斜率，预测当前候选所在高度应该出现的横坐标。
            expected_x = (
                entry_x + slope * float(item["cy"] - junction_y))
            delta = abs(float(item["cx"]) - expected_x)
            if best_delta is None or delta < best_delta:
                best_delta = delta
        if (best_delta is not None
                and best_delta <= JUNCTION_FORWARD_MAX_JUMP):
            count += 1
    return count

# 根据宽红块的左右延伸和前方延续情况识别路口类型。
# 返回 {"kind": 类型, "y": 路口纵坐标}；无法确认时返回 None。
def _classify_junction(layers, selected):
    wide_limit = int(LINE_W * JUNCTION_WIDTH)

    # 检查每层的全部宽色块，而不只检查巡线轨迹已选中的色块。
    # 单侧支路会把宽色块中心拉向一边，它可能被正常轨迹选择器主动拒绝，
    # 但仍然应该参与路口识别。
    for junction_index in range(len(layers)):
        for junction in layers[junction_index]["candidates"]:
            if junction["w"] < wide_limit:
                continue

            prediction = _predict_entry_line(
                selected, junction_index, junction["cy"], wide_limit)
            if prediction is None:
                continue
            entry_x, slope = prediction

            # 计算宽红块相对入射线交点向左、向右各延伸了多少像素。
            left_extent = entry_x - float(junction["rect"][0])
            right_extent = (
                float(junction["rect"][0] + junction["rect"][2])
                - entry_x)
            left_ok = left_extent >= JUNCTION_BRANCH_EXTENT
            right_ok = right_extent >= JUNCTION_BRANCH_EXTENT
            continues = _forward_continuation_count(
                layers,
                junction_index,
                junction["cy"],
                entry_x,
                slope,
                wide_limit,
            )
            forward_ok = continues >= JUNCTION_CONTINUATION_LAYERS

            # 前方+左右都有红线：十字路口。
            if forward_ok and left_ok and right_ok:
                return {"kind": "CROSS", "y": junction["cy"]}
            # 左右都有支路但前方截止：正面T字路口。
            if not forward_ok and left_ok and right_ok:
                return {"kind": "TJUNC_FRONT", "y": junction["cy"]}
            # 前方可通且只有左侧支路：左T字路口。
            if forward_ok and left_ok and not right_ok:
                return {"kind": "TJUNC_LEFT", "y": junction["cy"]}
            # 前方可通且只有右侧支路：右T字路口。
            if forward_ok and right_ok and not left_ok:
                return {"kind": "TJUNC_RIGHT", "y": junction["cy"]}
    return None

# 汇总轨迹选择、路口分类和直线拟合，生成一帧完整巡线结果。
def build_multi_line_result(layers, previous_center):
    selected = _select_layer_track(layers, previous_center)
    junction = _classify_junction(layers, selected)
    fit = _fit_track(selected)
    if fit is None and junction is not None:
        # 路口宽支路可能覆盖多个近处ROI。
        # 确认存在路口时，允许仅用2个窄红线点维持一个可用的前进方向。
        fit = _fit_track(selected, min_valid_layers=2)
    if fit is None:
        result = _lost_result()
        result["junction"] = junction
        return result, selected

    bottom_x, angle = fit
    valid_items = [item for item in selected if item is not None]
    main = valid_items[0]
    pixels = sum(item["pixels"] for item in valid_items)
    area = sum(item["w"] * item["h"] for item in valid_items)
    # 当前置信度按“有效层数 / 总层数”计算并换算成百分比。
    confidence = min(
        100, len(valid_items) * 100 // len(LINE_ROIS))
    return {
        "valid": True,
        "cx": bottom_x,
        "cy": LINE_H - 1,
        # 负值表示红线偏左，正值表示红线偏右；最后减去机械中心偏差。
        "error": bottom_x - LINE_ERROR_W // 2 - LINE_CENTER_BIAS,
        "angle": angle,
        "confidence": confidence,
        "cross": False,
        "rect": main["rect"],
        "pixels": pixels,
        "area": area,
        "points": [
            (item["cx"], item["cy"])
            for item in valid_items
        ],
        "selected_rects": [
            item["rect"] for item in valid_items
        ],
        "valid_layers": len(valid_items),
        "junction": junction,
    }, selected

# 对视觉拟合误差做绝对限幅和单帧变化限幅。
# 先限制物理范围，再限制相邻帧跳变，避免一次异常值污染下一帧轨迹选择。
def _filter_line_error(raw_error, previous_error):
    bounded = min(
        LINE_ERROR_ABS_LIMIT,
        max(-LINE_ERROR_ABS_LIMIT, int(raw_error)),
    )
    lower = int(previous_error) - LINE_ERROR_STEP_LIMIT
    upper = int(previous_error) + LINE_ERROR_STEP_LIMIT
    return min(upper, max(lower, bounded))

# 把 ROI、候选色块、选中轨迹、误差、路口和 FPS 画到 LCD OSD。
def draw_line_overlay(osd, result, candidates, roi,
                      source_size, target_size,
                      lost_frames=0, fps_text=""):
    source_width, source_height = source_size
    target_width, target_height = target_size

    # 同时兼容单个ROI元组和多个ROI组成的元组。
    rois = roi
    if rois and isinstance(rois[0], int):
        rois = (rois,)
    for one_roi in rois:
        roi_rect = map_rect(
            one_roi, source_width, source_height,
            target_width, target_height)
        osd.draw_rectangle(
            roi_rect[0], roi_rect[1], roi_rect[2], roi_rect[3],
            color=(255, 0, 80, 255), thickness=1)

    # 灰色竖线表示LCD画面的几何中心。
    center_x = target_width // 2
    osd.draw_line(
        center_x, 0, center_x, target_height - 1,
        color=(255, 128, 128, 128), thickness=1)

    # 黄色细框：find_blobs 找到的全部红色候选。
    if candidates:
        for blob in candidates:
            try:
                candidate_rect = map_rect(
                    blob.rect(), source_width, source_height,
                    target_width, target_height)
                osd.draw_rectangle(
                    candidate_rect[0], candidate_rect[1],
                    candidate_rect[2], candidate_rect[3],
                    color=(255, 255, 200, 100), thickness=1)
            except Exception:
                pass

    # 绿色框：轨迹选择器最终采用的色块。
    selected_rects = result.get("selected_rects", [])
    for rect in selected_rects:
        selected_rect = map_rect(
            rect, source_width, source_height,
            target_width, target_height)
        osd.draw_rectangle(
            selected_rect[0], selected_rect[1],
            selected_rect[2], selected_rect[3],
            color=(255, 0, 255, 0), thickness=2)

    # 红色十字和连线：各有效层的红线中心。
    points = result.get("points", [])
    last_point = None
    for point in points:
        mapped = map_point(
            point, source_width, source_height,
            target_width, target_height)
        osd.draw_cross(
            mapped[0], mapped[1],
            color=(255, 255, 0, 0), size=5, thickness=2)
        if last_point is not None:
            osd.draw_line(
                last_point[0], last_point[1], mapped[0], mapped[1],
                color=(255, 255, 0, 0), thickness=2)
        last_point = mapped

    if result["valid"]:
        # 黄色大十字：拟合直线在画面底部预测出的车前红线中心。
        main_rect = map_rect(
            result["rect"], source_width, source_height,
            target_width, target_height)
        main_point = map_point(
            (result["cx"], result["cy"]), source_width, source_height,
            target_width, target_height)
        osd.draw_rectangle(
            main_rect[0], main_rect[1], main_rect[2], main_rect[3],
            color=(255, 255, 0, 0), thickness=2)
        osd.draw_cross(
            main_point[0], main_point[1],
            color=(255, 255, 0, 0), size=8, thickness=2)
        osd.draw_line(
            center_x, target_height // 2, main_point[0], main_point[1],
            color=(255, 255, 255, 0), thickness=1)
        osd.draw_string_advanced(
            10, 10, 18, "FOUND", color=(255, 0, 255, 0))
        osd.draw_string_advanced(
            10, 32, 14, "cx=%d" % result["cx"],
            color=(255, 255, 255, 255))
        osd.draw_string_advanced(
            10, 50, 28, "ERR:%+d" % result["error"],
            color=(255, 255, 255, 0))
        osd.draw_string_advanced(
            10, 82, 13,
            "angle=%d layers=%d" % (
                result["angle"], result.get("valid_layers", 0)),
            color=(255, 210, 210, 210))
        osd.draw_string_advanced(
            10, 100, 13, "conf=%d%%" % result["confidence"],
            color=(255, 210, 210, 210))
    else:
        osd.draw_string_advanced(
            10, 10, 20, "LINE LOST", color=(255, 255, 0, 0))
        osd.draw_string_advanced(
            10, 35, 14, "lost_frames=%d" % lost_frames,
            color=(255, 255, 200, 200))

    junction_info = result.get("junction")
    if junction_info:
        junction_text = junction_info["kind"]
        junction_labels = {
            "CROSS": "CROSS",
            "TJUNC_LEFT": "TJUNC LEFT",
            "TJUNC_FRONT": "TJUNC FRONT",
            "TJUNC_RIGHT": "TJUNC RIGHT",
        }
        osd.draw_string_advanced(
            10, 120, 16,
            junction_labels.get(junction_text, junction_text),
            color=(255, 255, 255, 0))
    else:
        osd.draw_string_advanced(
            10, 120, 12, "ROI layers=%d" % len(rois),
            color=(255, 180, 180, 180))
    if fps_text:
        osd.draw_string_advanced(
            target_width - 90, 10, 14, fps_text,
            color=(255, 255, 255, 0))

# 在一帧 640x480 RGB565 图像中执行七层红线与路口检测。
def red_line(frame):
    global line_err_shown, prev_line_center, prev_line_error
    layers = []
    all_blobs = []

    # 每个ROI独立找红色色块，再把结果交给跨层轨迹算法。
    for i in range(len(LINE_ROIS)):
        roi = LINE_ROIS[i]
        min_pixels = LINE_ROI_MIN_PIXELS[i]
        try:
            blobs = frame.find_blobs(
                [RED_LAB],
                roi=roi,
                pixels_threshold=min_pixels,
                area_threshold=min_pixels,
                # 合并相邻的红色区域，降低红线断裂造成的碎片数量。
                merge=True,
            )
        except Exception as exc:
            if not line_err_shown:
                print("[REDLINE] find_blobs error:", exc)
                line_err_shown = True
            blobs = []

        candidates = []
        for blob in blobs:
            info = _blob_info(blob)
            if info is None:
                continue
            # 过滤极小噪点；每层更严格的像素阈值已在 find_blobs 中设置。
            if info["w"] < 3 or info["h"] < 3:
                continue
            candidates.append(info)
            all_blobs.append(blob)
        layers.append({
            "roi": roi,
            "candidates": candidates,
        })

    result, selected = build_multi_line_result(
        layers, prev_line_center)
    if result["valid"]:
        # 使用过滤后的误差更新输出和下一帧预测中心，避免原始310级尖峰
        # 把轨迹选择器连续拉向画面边缘。
        filtered_error = _filter_line_error(
            result["error"], prev_line_error)
        result["error"] = filtered_error
        result["cx"] = min(
            LINE_W - 1,
            max(
                0,
                LINE_ERROR_W // 2
                + LINE_CENTER_BIAS
                + filtered_error,
            ),
        )
        prev_line_error = filtered_error
        prev_line_center = result["cx"]
    return result, all_blobs


print("[INFO] AI digit + red-line running...")

# ==================== 主循环 ====================
while True:
    # 通道0：供KPU数字检测使用的 1280x720 RGB888P 图像。
    img = pl.get_frame()
    # 与LCD同尺寸的ARGB8888叠加层，所有调试图形都画在这里。
    osd = pl.osd_img

    # ---------- 1. AI数字检测 ----------
    res = det_app.run(img)

    # ---------- 2. 红线检测 ----------
    # 从PipeLine的原生RGB565通道1抓图，不把AI图像复制/转换成RGB565。
    try:
        line_frame = pl.sensor.snapshot(chn=CAM_CHN_ID_1)
        line_result, line_blobs = red_line(line_frame)
    except Exception as exc:
        if not line_err_shown:
            print("[REDLINE] channel1 snapshot error:", exc)
            line_err_shown = True
        line_result = _lost_result()
        line_blobs = []

    # ---------- 3. LCD叠加显示 ----------
    # DetectionApp.draw_result() 内部会清空OSD，所以数字结果必须先画，
    # 然后再画红线调试层，否则红线图形会被数字绘制过程擦掉。
    det_app.draw_result(osd, res)
    if line_result["valid"]:
        lost_frames = 0
    else:
        lost_frames += 1

    # 每秒更新一次FPS文本，避免每帧频繁格式化字符串。
    fps_frames += 1
    fps_now = time.ticks_ms()
    fps_elapsed = time.ticks_diff(fps_now, fps_start_ms)
    if fps_elapsed >= 1000:
        fps_text = "FPS:%.1f" % (fps_frames * 1000.0 / max(fps_elapsed, 1))
        fps_start_ms = fps_now
        fps_frames = 0

    draw_line_overlay(
        osd,
        line_result,
        line_blobs,
        roi=LINE_ROIS,
        source_size=(LINE_W, LINE_H),
        target_size=(disp_size[0], disp_size[1]),
        lost_frames=lost_frames,
        fps_text=fps_text,
    )
    # 将本帧OSD提交到LCD。
    pl.show_image()

    # ---------- 4. 巡线/路口 UART发送 ----------
    now = time.ticks_ms()
    elapsed = time.ticks_diff(now, last_tx_ms)
    ok = line_result["valid"]
    err = line_result["error"]
    ang = line_result["angle"]
    cnf = line_result["confidence"]

    # raw_junction 是当前单帧识别结果，active_junction 是去抖后的锁存结果。
    junction_info = line_result.get("junction")
    raw_junction = (
        junction_info["kind"] if junction_info is not None else None)
    raw_junction_y = (
        junction_info["y"] if junction_info is not None else 0)
    if raw_junction:
        # 当前帧看到了路口：累计相同类型的连续出现帧数。
        junction_clear_cnt = 0
        if raw_junction == junction_candidate:
            junction_cnt += 1
        else:
            junction_candidate = raw_junction
            junction_cnt = 1
    else:
        # 当前帧没看到路口：清空候选，并累计已锁存路口的消失帧数。
        junction_candidate = None
        junction_cnt = 0
        if active_junction is not None:
            junction_clear_cnt += 1
            if junction_clear_cnt >= JUNCTION_CLEAR_DBNC:
                active_junction = None
                junction_clear_cnt = 0

    if junction_cnt >= JUNCTION_DBNC:
        # 同一候选连续达到3帧后，才升级为有效路口。
        if active_junction != junction_candidate:
            active_junction = junction_candidate
            # 把上次发送时间回拨一个周期，使新路口可以立即发送第一帧。
            last_junction_tx_ms = time.ticks_add(
                now, -JUNCTION_TX_PERIOD)
        active_junction_y = raw_junction_y

    if ok:
        # 正常巡线时，以最快40 ms一次的频率发送误差、角度和置信度。
        if elapsed >= TX_PERIOD:
            last_tx_ms = now
            tx("LINE", err, ang, cnf)
        if fc % 10 == 0:
            print("[LINE] err=%d ang=%d c=%d" % (err, ang, cnf))
        lost_printed = False
    elif lost_frames >= 3:
        # 连续丢线3帧后才上报LOST，过滤偶发的一帧识别失败。
        if not lost_printed:
            print("[LINE] LOST")
            lost_printed = True
        # 丢线持续存在时约每400 ms重复发送，便于下位机做安全保护。
        if elapsed >= TX_PERIOD * 10:
            last_tx_ms = now
            tx("LOST")

    # 已确认且当前帧仍看见同一路口时，每200 ms重复发送一次。
    junction_elapsed = time.ticks_diff(now, last_junction_tx_ms)
    if (active_junction is not None
            and raw_junction == active_junction
            and junction_elapsed >= JUNCTION_TX_PERIOD):
        last_junction_tx_ms = now
        if active_junction == "CROSS":
            # 参数1表示十字路口有效，第三个字段是路口纵坐标。
            tx("CROSS", 1, active_junction_y)
            print("[LINE] CROSS")
        elif active_junction == "TJUNC_LEFT":
            # T字类型：-1=左侧支路，0=前方截止，1=右侧支路。
            tx("TJUNC", -1, active_junction_y)
            print("[LINE] TJUNC LEFT")
        elif active_junction == "TJUNC_FRONT":
            tx("TJUNC", 0, active_junction_y)
            print("[LINE] TJUNC FRONT")
        elif active_junction == "TJUNC_RIGHT":
            tx("TJUNC", 1, active_junction_y)
            print("[LINE] TJUNC RIGHT")

    # ---------- 5. 接收目标数字并发送AI数字结果 ----------
    # 先处理G3507的 $PING# / $SET_TARGET,n# 命令。
    poll_target_command()
    # DetectionApp输出：boxes=检测框，scores=置信度，idx=模型类别编号。
    boxes, scores, indices = res.get("boxes"), res.get("scores"), res.get("idx")
    digit_detections = build_digit_detections(boxes, scores, indices)
    if len(digit_detections) == 1:
        # 画面中恰好只有一个数字时，周期发送NUM，供起点识别目标。
        d = digit_detections[0]["digit"]
        c = digit_detections[0]["confidence"]
        if (d and time.ticks_diff(now, last_num_tx_ms)
                >= NUM_TX_PERIOD):
            tx("NUM", d, c)
            last_num_tx_ms = now

    # 已知目标数字后，只发送该数字所在的左/右方向，忽略其他病房牌。
    room = select_target_room(digit_detections, target_digit)
    if (room is not None
            and time.ticks_diff(now, last_room_tx_ms)
            >= ROOM_TX_PERIOD):
        tx(
            "ROOM",
            room["digit"],
            room["confidence"],
            room["direction"],
        )
        last_room_tx_ms = now

    # ---------- 6. 帧计数与内存回收 ----------
    fc += 1
    # CanMV长期运行时每帧主动回收临时对象，降低内存碎片和溢出风险。
    gc.collect()
