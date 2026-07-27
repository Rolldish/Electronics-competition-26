# -*- coding: utf-8 -*-
"""
red_line_debug.py —— K230 红线巡线独立调试脚本

功能：
  1. 初始化 GC2093 摄像头（640x480 RGB565）
  2. 在画面底部单个 ROI 内进行红色 LAB 阈值分割
  3. 色块去噪 → 主红线选择 → cx / error 计算
  4. 调试画面叠加绘制
  5. 串口终端调试信息输出（仅 print，不发送正式 UART 协议）

运行方式：
  在 CanMV IDE 中打开此文件并运行，或将此文件复制到 /sdcard/ 后导入运行。

依赖（CanMV K230 MicroPython）：
  media.sensor  – Sensor
  media.display – Display
  media.media   – MediaManager

注意：
  - 本脚本不会控制电机
  - 本脚本不会发送 $LINE / $LOST / $CROSS / $READY 等正式 UART 协议
  - 本脚本不依赖 G3507、不依赖 YOLO、不依赖 KPU
  - 本脚本尚未经过 K230 真机验证
"""

import gc
import os
import time

# ============================================================
# 集中配置区 —— 所有现场需要调整的参数均在此处
# ============================================================

# --- 图像分辨率 ---
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

# --- 像素格式 ---
# 必须与 Sensor 支持的格式一致。项目现有代码使用 "RGB565"
PIXFORMAT_NAME = "RGB565"

# --- 显示模式 ---
# 项目现有代码使用 "ST7701"（LCD 屏幕），同时 to_ide=True 将画面回传 IDE
# 可选值： "ST7701" / "VIRT" / "LT9611" / "ST7701_HDMI" 等
# 若使用 HDMI 输出，请根据实际硬件修改此值
DISPLAY_MODE = "ST7701"
DISPLAY_TO_IDE = True

# --- 底部 ROI（感兴趣区域） ---
# 注意：该 ROI 只是摄像头尚未固定时的初始测试值，后续需要根据真机截图调整！
ROI_X = 0
ROI_Y = 260
ROI_W = 640
ROI_H = 220

# --- 红色 LAB 阈值 ---
# 6 值 LAB 阈值格式：(L_MIN, L_MAX, A_MIN, A_MAX, B_MIN, B_MAX)
# 当前值与 drug_car/config.py 的 RED_LAB_THRESHOLD 保持一致
# 注意：该阈值尚未经过真机红线调参验证，可能需要在真机上用 calibrate_red.py 重新标定
RED_LAB_THRESHOLD = (11, 74, 15, 55, -7, 39)

# --- 色块过滤参数 ---
PIXELS_THRESHOLD = 80   # 色块像素数下限，过滤小噪点
AREA_THRESHOLD = 80     # 色块外接矩形面积下限，过滤窄条噪点
MERGE_BLOBS = True      # 是否合并相邻色块（True / False）

# --- 主色块筛选参数 ---
MIN_BLOB_WIDTH = 5      # 最小有效色块宽度（像素）
MIN_BLOB_HEIGHT = 5     # 最小有效色块高度（像素）

# --- 丢线阈值 ---
# 连续丢线帧数达到此值后，状态标记为 LOST（不影响检测逻辑，仅用于信息显示）
LOST_FRAME_THRESHOLD = 5

# --- 调试绘制开关 ---
DRAW_ALL_BLOBS = True           # 是否绘制所有通过基础阈值色块的包围框
DRAW_CENTER_LINE = True         # 是否绘制图像中心竖线（x=320）
DRAW_CENTER_TO_BLOB_LINE = True # 是否绘制从图像中心到红线中心的连线
DRAW_BINARY_PREVIEW = False     # 是否在画面角落绘制二值图预览（需要 image.copy + binary 支持）

# --- 打印控制 ---
PRINT_INTERVAL_MS = 200         # 终端打印最小间隔（毫秒），避免无限高速打印

# --- 帧率显示 ---
SHOW_FPS = True                 # 是否在画面上显示 FPS

# --- 垃圾回收间隔 ---
GC_INTERVAL_FRAMES = 30         # 每隔多少帧执行一次 gc.collect()

# ============================================================
# 辅助函数
# ============================================================

def _ticks_ms():
    """返回当前毫秒时间戳，兼容 K230 MicroPython 和 PC Python。"""
    if hasattr(time, "ticks_ms"):
        return time.ticks_ms()
    return int(time.time() * 1000)


def _ticks_diff(now, before):
    """计算毫秒差值。"""
    if hasattr(time, "ticks_diff"):
        return time.ticks_diff(now, before)
    return now - before


def _sleep_ms(ms):
    """毫秒延时。"""
    if hasattr(time, "sleep_ms"):
        time.sleep_ms(ms)
    else:
        time.sleep(ms / 1000.0)


def _resolve_display_mode(display_module, mode):
    """将字符串 DISPLAY_MODE 解析为 Display 常量（如 'ST7701' → Display.ST7701）。"""
    if isinstance(mode, str) and hasattr(display_module, mode):
        return getattr(display_module, mode)
    return mode


# ============================================================
# 验证函数
# ============================================================

def validate_roi():
    """检查 ROI 是否在图像范围内，必要时裁剪并打印警告。"""
    global ROI_X, ROI_Y, ROI_W, ROI_H
    changed = False

    if ROI_X < 0:
        ROI_W += ROI_X
        ROI_X = 0
        changed = True
    if ROI_Y < 0:
        ROI_H += ROI_Y
        ROI_Y = 0
        changed = True
    if ROI_X + ROI_W > IMAGE_WIDTH:
        ROI_W = IMAGE_WIDTH - ROI_X
        changed = True
    if ROI_Y + ROI_H > IMAGE_HEIGHT:
        ROI_H = IMAGE_HEIGHT - ROI_Y
        changed = True
    if ROI_W <= 0 or ROI_H <= 0:
        raise ValueError("ROI 无效：ROI 区域完全在图像范围之外")

    if changed:
        print("[WARN] ROI 已裁剪至图像边界内: x=%d y=%d w=%d h=%d" %
              (ROI_X, ROI_Y, ROI_W, ROI_H))


# ============================================================
# 主色块选择
# ============================================================

def select_main_blob(blobs):
    """从 find_blobs 返回的色块列表中选择主红线区域。

    默认策略：优先选择 pixels() 最大的色块。
    只返回通过最小宽高过滤的色块。

    Args:
        blobs: find_blobs 返回的色块列表（每个 blob 对象支持 .pixels(), .cx(), .cy(),
               .w(), .h(), .rect() 等方法）

    Returns:
        主色块对象，或 None（无有效色块）
    """
    if not blobs:
        return None

    candidates = []
    for blob in blobs:
        try:
            w = blob.w()
            h = blob.h()
        except Exception:
            continue
        if w < MIN_BLOB_WIDTH or h < MIN_BLOB_HEIGHT:
            continue
        try:
            px = blob.pixels()
        except Exception:
            px = 0
        candidates.append((blob, px))

    if not candidates:
        return None

    # 按像素数降序排列，选择最大的
    candidates.sort(key=lambda item: -item[1])
    return candidates[0][0]


# ============================================================
# 调试信息绘制
# ============================================================

def draw_debug_overlay(image, main_blob, lost_frames, blobs=None,
                       blob_count=0, fps_text=""):
    """在图像上绘制调试信息叠加层。

    - 正常检测时：ROI框、中心线、色块框、主色块中心、cx、error
    - 丢线时：ROI框、中心线、LINE LOST、不绘制伪中心

    Args:
        image: 摄像头采集的原始图像（会直接在上面绘制）
        main_blob: 主色块对象或 None
        lost_frames: 连续丢线帧数
        blobs: 所有通过阈值的色块列表（可选，用于绘制所有候选框）
        blob_count: 通过基础阈值的色块总数
        fps_text: FPS 字符串
    """
    center_x = IMAGE_WIDTH // 2
    center_y = IMAGE_HEIGHT // 2

    # 1. ROI 矩形（绿色）
    image.draw_rectangle(ROI_X, ROI_Y, ROI_W, ROI_H,
                         color=(0, 255, 0), thickness=2)

    # 2. 图像中心竖线 x=320（白色半透明 → 灰色）
    if DRAW_CENTER_LINE:
        image.draw_line(center_x, 0, center_x, IMAGE_HEIGHT - 1,
                        color=(128, 128, 128), thickness=1)

    # 3. 画面方向标记
    image.draw_string_advanced(10, IMAGE_HEIGHT - 20, 14,
                               "LEFT", color=(200, 200, 200))
    image.draw_string_advanced(IMAGE_WIDTH - 60, IMAGE_HEIGHT - 20, 14,
                               "RIGHT", color=(200, 200, 200))
    image.draw_string_advanced(center_x - 20, 10, 14,
                               "TOP", color=(200, 200, 200))

    if main_blob is not None:
        # --- 检测到红线 ---

        # 4. 所有通过基础阈值的色块框（淡蓝色），可选
        if DRAW_ALL_BLOBS and blobs:
            for blob_item in blobs:
                try:
                    rx, ry, rw, rh = blob_item.rect()
                    image.draw_rectangle(rx, ry, rw, rh,
                                         color=(255, 200, 100), thickness=1)
                except Exception:
                    pass

        # 5. 主色块包围框（红色）
        try:
            rx, ry, rw, rh = main_blob.rect()
            image.draw_rectangle(rx, ry, rw, rh,
                                 color=(255, 0, 0), thickness=2)
        except Exception:
            pass

        # 6. 主色块中心点（红色十字）
        try:
            cx_blob = main_blob.cx()
            cy_blob = main_blob.cy()
            image.draw_cross(cx_blob, cy_blob,
                             color=(255, 0, 0), size=8, thickness=2)
        except Exception:
            cx_blob = 0
            cy_blob = 0

        # 7. 从图像中心到红线中心的连线（黄色），可选
        if DRAW_CENTER_TO_BLOB_LINE:
            try:
                image.draw_line(center_x, center_y,
                                cx_blob, cy_blob,
                                color=(255, 255, 0), thickness=1)
            except Exception:
                pass

        # 8. 数值信息叠加
        error_val = cx_blob - center_x
        try:
            pixels_val = main_blob.pixels()
            area_val = main_blob.w() * main_blob.h()
        except Exception:
            pixels_val = 0
            area_val = 0

        # 左上角信息面板
        y_offset = 10
        image.draw_string_advanced(10, y_offset, 16,
                                   "FOUND", color=(0, 255, 0))
        y_offset += 20
        image.draw_string_advanced(10, y_offset, 14,
                                   "cx=%d" % cx_blob, color=(255, 255, 255))
        y_offset += 18
        image.draw_string_advanced(10, y_offset, 14,
                                   "error=%d" % error_val, color=(255, 255, 255))
        y_offset += 18
        image.draw_string_advanced(10, y_offset, 14,
                                   "pixels=%d area=%d" % (pixels_val, area_val),
                                   color=(200, 200, 200))
        y_offset += 18
        image.draw_string_advanced(10, y_offset, 13,
                                   "roi=(%d,%d,%d,%d)" % (ROI_X, ROI_Y, ROI_W, ROI_H),
                                   color=(180, 180, 180))
        y_offset += 18
        image.draw_string_advanced(10, y_offset, 12,
                                   "thr=(%d,%d,%d,%d,%d,%d)" % RED_LAB_THRESHOLD,
                                   color=(160, 160, 160))

        # FPS
        if SHOW_FPS and fps_text:
            image.draw_string_advanced(IMAGE_WIDTH - 80, 10, 14,
                                       fps_text, color=(255, 255, 0))

    else:
        # --- 丢线 ---
        y_offset = 10
        image.draw_string_advanced(10, y_offset, 20,
                                   "LINE LOST", color=(255, 0, 0))
        y_offset += 25
        image.draw_string_advanced(10, y_offset, 14,
                                   "lost_frames=%d" % lost_frames,
                                   color=(255, 200, 200))
        y_offset += 18
        image.draw_string_advanced(10, y_offset, 13,
                                   "roi=(%d,%d,%d,%d)" % (ROI_X, ROI_Y, ROI_W, ROI_H),
                                   color=(180, 180, 180))
        y_offset += 18
        image.draw_string_advanced(10, y_offset, 12,
                                   "thr=(%d,%d,%d,%d,%d,%d)" % RED_LAB_THRESHOLD,
                                   color=(160, 160, 160))

        if SHOW_FPS and fps_text:
            image.draw_string_advanced(IMAGE_WIDTH - 80, 10, 14,
                                       fps_text, color=(255, 255, 0))


# ============================================================
# 调试信息打印
# ============================================================

def print_debug_status(cx, error_val, pixels_val, area_val, lost_frames,
                       blob_count, fps_text=""):
    """以受控频率向串口终端输出一行调试信息。

    只使用 print()，不发送任何正式 UART 协议帧。
    """
    if cx is not None:
        print("FOUND cx=%d error=%d pixels=%d area=%d roi=(%d,%d,%d,%d) "
              "blobs=%d %s" %
              (cx, error_val, pixels_val, area_val,
               ROI_X, ROI_Y, ROI_W, ROI_H, blob_count, fps_text))
    else:
        print("LOST frames=%d roi=(%d,%d,%d,%d) blobs=%d %s" %
              (lost_frames, ROI_X, ROI_Y, ROI_W, ROI_H, blob_count, fps_text))


# ============================================================
# 资源清理
# ============================================================

def cleanup(sensor, display_module, media_module):
    """按项目已验证的清理顺序释放硬件资源。

    清理顺序（来自 app.py VisionApplication.close）：
      1. sensor.stop()
      2. Display.deinit()
      3. os.exitpoint(os.EXITPOINT_ENABLE_SLEEP) + time.sleep_ms(100)
      4. MediaManager.deinit()
    """
    # 1. 停止 Sensor
    try:
        sensor.stop()
        print("[cleanup] Sensor stopped")
    except Exception as e:
        print("[cleanup] Sensor stop failed: %s" % e)

    # 2. 释放 Display
    try:
        display_module.deinit()
        print("[cleanup] Display released")
    except Exception as e:
        print("[cleanup] Display deinit failed: %s" % e)

    # 3. 退出点 + 延时
    try:
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    except (ImportError, AttributeError):
        pass
    _sleep_ms(100)

    # 4. 释放 MediaManager
    try:
        media_module.deinit()
        print("[cleanup] MediaManager released")
    except Exception as e:
        print("[cleanup] MediaManager deinit failed: %s" % e)


# ============================================================
# 主循环
# ============================================================

def main():
    """红线调试主函数。

    初始化摄像头→验证ROI→持续采集→红线检测→绘制→终端输出。
    退出时（IDE 停止按钮或异常）安全释放所有硬件资源。
    """
    print("=" * 50)
    print("  K230 红线巡线调试脚本")
    print("  red_line_debug.py")
    print("=" * 50)
    print("分辨率: %d x %d" % (IMAGE_WIDTH, IMAGE_HEIGHT))
    print("像素格式: %s" % PIXFORMAT_NAME)
    print("显示模式: %s (to_ide=%s)" % (DISPLAY_MODE, DISPLAY_TO_IDE))
    print("ROI: x=%d y=%d w=%d h=%d" % (ROI_X, ROI_Y, ROI_W, ROI_H))
    print("LAB 阈值: (%d,%d,%d,%d,%d,%d)" % RED_LAB_THRESHOLD)
    print("pixels_threshold=%d area_threshold=%d merge=%s" %
          (PIXELS_THRESHOLD, AREA_THRESHOLD, MERGE_BLOBS))
    print("min_blob: w>=%d h>=%d" % (MIN_BLOB_WIDTH, MIN_BLOB_HEIGHT))
    print("丢线阈值: %d 帧" % LOST_FRAME_THRESHOLD)
    print("打印间隔: %d ms" % PRINT_INTERVAL_MS)
    print("=" * 50)

    # 验证 ROI
    validate_roi()
    print("[OK] ROI 验证通过: x=%d y=%d w=%d h=%d" %
          (ROI_X, ROI_Y, ROI_W, ROI_H))

    # 导入 CanMV K230 硬件模块
    try:
        from media.sensor import Sensor
        from media.display import Display
        from media.media import MediaManager
    except ImportError as exc:
        print("[FATAL] 无法导入 CanMV K230 硬件模块: %s" % exc)
        print("        此脚本必须在 CanMV K230 板端运行。")
        return

    sensor = None
    display_mode_resolved = _resolve_display_mode(Display, DISPLAY_MODE)

    try:
        # ---- 初始化 Sensor ----
        print("[init] 初始化 GC2093 Sensor ...")
        sensor = Sensor()
        sensor.reset()
        sensor.set_framesize(width=IMAGE_WIDTH, height=IMAGE_HEIGHT)
        # 使用 Sensor.RGB565 或按 PIXFORMAT_NAME 查找对应常量
        pixformat = getattr(Sensor, PIXFORMAT_NAME, Sensor.RGB565)
        sensor.set_pixformat(pixformat)
        print("[init] Sensor 配置完成")

        # ---- 初始化 Display ----
        print("[init] 初始化 Display (mode=%s, to_ide=%s) ..." %
              (DISPLAY_MODE, DISPLAY_TO_IDE))
        Display.init(display_mode_resolved, to_ide=DISPLAY_TO_IDE)
        print("[init] Display 初始化完成")

        # ---- 初始化 MediaManager ----
        print("[init] 初始化 MediaManager ...")
        MediaManager.init()
        print("[init] MediaManager 初始化完成")

        # ---- 启动 Sensor 流 ----
        sensor.run()
        print("[init] Sensor 已启动，开始采集画面")
        print("-" * 50)
        print("提示：将红线置于画面底部 ROI 区域内观察检测结果。")
        print("      画面中绿色框 = ROI，灰色竖线 = 图像中心 x=320。")
        print("      红色框 = 主红线色块，红色十字 = 色块中心。")
        print("      LINE LOST = 当前帧未检测到有效红色色块。")
        print("-" * 50)

        # ---- 运行时变量 ----
        frame_count = 0
        lost_frames = 0
        last_print_ms = 0
        fps_start_ms = _ticks_ms()
        fps_frame_count = 0
        fps_text = ""

        # ---- 主循环 ----
        while True:
            # CanMV 协作调度点
            try:
                os.exitpoint()
            except (ImportError, AttributeError):
                pass

            now_ms = _ticks_ms()

            # 采集一帧图像
            image = sensor.snapshot()
            frame_count += 1

            # ---- 在 ROI 内查找红色色块 ----
            try:
                blobs = image.find_blobs(
                    [RED_LAB_THRESHOLD],
                    roi=(ROI_X, ROI_Y, ROI_W, ROI_H),
                    pixels_threshold=PIXELS_THRESHOLD,
                    area_threshold=AREA_THRESHOLD,
                    merge=MERGE_BLOBS,
                )
            except Exception as exc:
                print("[WARN] find_blobs 异常: %s" % exc)
                blobs = []

            blob_count = len(blobs) if blobs else 0

            # ---- 选择主红线色块 ----
            main_blob = select_main_blob(blobs)

            if main_blob is not None:
                # 检测到有效红线
                lost_frames = 0
                try:
                    cx = main_blob.cx()
                    pixels_val = main_blob.pixels()
                    area_val = main_blob.w() * main_blob.h()
                except Exception:
                    cx = None
                    pixels_val = 0
                    area_val = 0

                if cx is not None:
                    error_val = cx - (IMAGE_WIDTH // 2)
                else:
                    error_val = None
            else:
                # 当前帧未检测到有效红线
                cx = None
                error_val = None
                pixels_val = 0
                area_val = 0
                lost_frames += 1

            # ---- FPS 计算 ----
            if SHOW_FPS:
                fps_frame_count += 1
                fps_elapsed = _ticks_diff(now_ms, fps_start_ms)
                if fps_elapsed >= 1000:
                    fps = fps_frame_count * 1000.0 / max(fps_elapsed, 1)
                    fps_text = "FPS:%.1f" % fps
                    fps_frame_count = 0
                    fps_start_ms = now_ms

            # ---- 调试画面绘制 ----
            draw_debug_overlay(image, main_blob, lost_frames, blobs=blobs,
                               blob_count=blob_count, fps_text=fps_text)

            # ---- 显示画面 ----
            try:
                Display.show_image(image)
            except Exception as exc:
                print("[WARN] Display.show_image 异常: %s" % exc)

            # ---- 终端打印（受控频率） ----
            elapsed_since_print = _ticks_diff(now_ms, last_print_ms)
            if elapsed_since_print >= PRINT_INTERVAL_MS:
                print_debug_status(cx, error_val, pixels_val, area_val,
                                   lost_frames, blob_count, fps_text)
                last_print_ms = now_ms

            # ---- 垃圾回收 ----
            if frame_count % GC_INTERVAL_FRAMES == 0:
                try:
                    gc.collect()
                except Exception:
                    pass

    except KeyboardInterrupt:
        print("\n[exit] 用户中断")
    except Exception as exc:
        print("[FATAL] 运行时异常: %s" % exc)
        # 尝试打印更多异常上下文
        try:
            import sys
            sys.print_exception(exc)
        except Exception:
            pass
    finally:
        print("[cleanup] 开始释放硬件资源 ...")
        cleanup(sensor, Display, MediaManager)
        print("[cleanup] 资源释放完成。脚本退出。")


# ============================================================
# 入口
# ============================================================

if __name__ == "__main__":
    main()
