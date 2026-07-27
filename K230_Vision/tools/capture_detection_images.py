"""Raw full-frame image collector for object-detection datasets on CanMV K230.

Copy this file to the board and run it from CanMV IDE.  It saves complete
camera frames to /sdcard/detection_dataset/raw_images/<batch>/ and appends a
simple metadata text file in the same batch directory.
"""

import os
import time


# ---------------- User settings ----------------
BATCH_NAME = "R05_dual_digit"
SCENE = "dual"
LIGHTING = "normal"
MOTION = "static"
NOTE = "pair_set=B"

CAPTURE_MODE = "key"   # single, countdown, interval, key
COUNTDOWN_SECONDS = 3
SAVE_INTERVAL_MS = 1200
MAX_SAVE_COUNT = 84
START_INDEX = 1
WARMUP_FRAMES = 30
MIN_FRAME_MEAN = 8

# YAHBOOM K230 board user key reference (do not confuse with RESET/BOOT):
# The board's bundled ``04.key.py`` example uses ``ybUtils.YbKey``.
# The bundled YbKey implementation maps physical KEY1 to pin 61 with
# ``fpioa.set_function(61, FPIOA.GPIO0 + 61, ie=1, oe=0)`` and creates
# ``Pin(61, Pin.IN, pull=Pin.PULL_UP, drive=7)``.  It is active-low:
# value()==0 means pressed, value()==1 means released.  Do not change these
# values without re-checking the board documentation and example.
KEY_GPIO_PIN = 61
KEY_GPIO_NUM = 61
KEY_ACTIVE_LEVEL = 0
KEY_DEBOUNCE_MS = 200

FRAME_WIDTH = 640
FRAME_HEIGHT = 480
DISPLAY_WIDTH = 640
DISPLAY_HEIGHT = 480
JPEG_QUALITY = 90

DATASET_ROOT = "/sdcard/detection_dataset/raw_images"


def join_path(*parts):
    cleaned = []
    for part in parts:
        if part is None:
            continue
        text = str(part).replace("\\", "/")
        if not text:
            continue
        if not cleaned:
            cleaned.append(text.rstrip("/"))
        else:
            cleaned.append(text.strip("/"))
    path = "/".join(cleaned)
    if path == "":
        return "/"
    if parts and str(parts[0]).startswith("/") and not path.startswith("/"):
        return "/" + path
    return path


def safe_token(text):
    source = str(text)
    output = ""
    last_was_sep = False
    for char in source:
        is_alnum = (
            ("a" <= char <= "z")
            or ("A" <= char <= "Z")
            or ("0" <= char <= "9")
        )
        if is_alnum:
            output += char.lower()
            last_was_sep = False
        elif char in ("-", "_"):
            if output and not last_was_sep:
                output += char
                last_was_sep = True
        elif char in (" ", ".", "/"):
            if output and not last_was_sep:
                output += "_"
                last_was_sep = True
    output = output.strip("_-")
    if not output:
        return "sample"
    return output


def batch_id(batch_name):
    token = safe_token(batch_name)
    return token.replace("_", "")


def build_filename(batch_name, scene, motion, index):
    return "%s_%s_%s_%06d.jpg" % (
        batch_id(batch_name),
        safe_token(scene),
        safe_token(motion),
        index,
    )


def file_exists(path):
    try:
        os.stat(path)
        return True
    except OSError:
        return False


def next_available_image_path(directory, batch_name, scene, motion,
                              start_index, exists_fn=file_exists):
    index = start_index
    while index < 1000000:
        filename = build_filename(batch_name, scene, motion, index)
        path = join_path(directory, filename)
        if not exists_fn(path):
            return index, path
        index += 1
    raise RuntimeError("no available image filename")


def metadata_line(filename, batch, scene, lighting, motion, note):
    clean_note = str(note).replace(",", " ").replace("\r", " ").replace("\n", " ")
    while "  " in clean_note:
        clean_note = clean_note.replace("  ", " ")
    return "%s,%s,%s,%s,%s,%s\n" % (
        filename,
        safe_token(batch),
        safe_token(scene),
        safe_token(lighting),
        safe_token(motion),
        clean_note.strip(),
    )


def ensure_dir(path):
    normalized = str(path).replace("\\", "/").rstrip("/")
    if not normalized or normalized == "/":
        return
    try:
        os.stat(normalized)
        return
    except OSError:
        pass

    slash = normalized.rfind("/")
    if slash > 0:
        ensure_dir(normalized[:slash])
    try:
        os.mkdir(normalized)
    except OSError:
        try:
            os.stat(normalized)
        except OSError as exc:
            raise exc


def ticks_ms():
    if hasattr(time, "ticks_ms"):
        return time.ticks_ms()
    return int(time.time() * 1000)


def ticks_diff(now, before):
    if hasattr(time, "ticks_diff"):
        return time.ticks_diff(now, before)
    return now - before


def sleep_ms(ms):
    if hasattr(time, "sleep_ms"):
        time.sleep_ms(ms)
    else:
        time.sleep(ms / 1000.0)


class CaptureKey:
    """Debounced active-level edge detector for the physical user KEY1."""

    def __init__(self, pin, active_level=KEY_ACTIVE_LEVEL,
                 debounce_ms=KEY_DEBOUNCE_MS, clock_fn=ticks_ms):
        self.pin = pin
        self.active_level = active_level
        self.debounce_ms = debounce_ms
        self.clock_fn = clock_fn
        initial = self._read_pressed()
        self._stable_pressed = initial
        self._raw_pressed = initial
        self._raw_changed_at = self.clock_fn()

    def _read_pressed(self):
        return self.pin.value() == self.active_level

    def is_capture_key_pressed_edge(self, now_ms=None):
        """Return True only once for a debounced press transition."""
        if now_ms is None:
            now_ms = self.clock_fn()
        raw_pressed = self._read_pressed()
        if raw_pressed != self._raw_pressed:
            self._raw_pressed = raw_pressed
            self._raw_changed_at = now_ms
            return False
        if (raw_pressed != self._stable_pressed and
                ticks_diff(now_ms, self._raw_changed_at) >= self.debounce_ms):
            was_pressed = self._stable_pressed
            self._stable_pressed = raw_pressed
            return (not was_pressed) and raw_pressed
        return False

    def is_pressed(self):
        return self._read_pressed()


def init_capture_key():
    """Map KEY1 to GPIO61 using the board's documented FPIOA wiring."""
    from machine import FPIOA, Pin

    fpioa = FPIOA()
    fpioa.set_function(KEY_GPIO_PIN, FPIOA.GPIO0 + KEY_GPIO_NUM, ie=1, oe=0)
    pin = Pin(KEY_GPIO_PIN, Pin.IN, pull=Pin.PULL_UP, drive=7)
    return CaptureKey(pin)


def is_capture_key_pressed_edge(key_controller):
    return key_controller.is_capture_key_pressed_edge()


def wait_capture_key_release(key_controller):
    """Compatibility helper; polling loop normally waits without blocking."""
    return not key_controller.is_pressed()


def remaining_space_text(path):
    if not hasattr(os, "statvfs"):
        return "free:unknown"
    try:
        stat = os.statvfs(path)
        free_bytes = stat[0] * stat[3]
        return "free:%dMB" % (free_bytes // (1024 * 1024))
    except Exception:
        return "free:unknown"


class MetadataConflict(Exception):
    """Raised when metadata for an already-saved filename has different content."""
    pass


def check_metadata_conflict(path, line, exists_fn=file_exists, open_fn=open):
    """Raise MetadataConflict if the same filename exists with different content.
    Returns True if exact duplicate (safe to skip).
    Returns False if filename does not exist yet (safe to write)."""
    if not exists_fn(path):
        return False
    try:
        new_filename = line.split(",")[0].strip()
        with open_fn(path, "r") as handle:
            for existing_line in handle:
                if not existing_line.strip():
                    continue
                existing_filename = existing_line.split(",")[0].strip()
                if existing_filename != new_filename:
                    continue
                if existing_line.rstrip("\n") == line.rstrip("\n"):
                    return True  # exact duplicate
                msg = (
                    "[metadata] CONFLICT filename=%s\n"
                    "  existing: %s"
                    "  new:      %s" % (new_filename, existing_line.rstrip("\n"), line.rstrip("\n"))
                )
                raise MetadataConflict(msg)
    except MetadataConflict:
        raise
    except Exception:
        pass
    return False


def append_metadata(path, line, exists_fn=file_exists, open_fn=open):
    if not exists_fn(path):
        with open_fn(path, "w") as handle:
            handle.write("filename,batch,scene,lighting,motion,note\n")
    if not line:
        return
    if check_metadata_conflict(path, line, exists_fn=exists_fn, open_fn=open_fn):
        print("[metadata] exact duplicate skipped: %s" % line.split(",")[0].strip())
        return
    with open_fn(path, "a") as handle:
        handle.write(line)


def save_image(image_obj, path):
    try:
        image_obj.save(path, quality=JPEG_QUALITY)
    except TypeError:
        image_obj.save(path)


def draw_overlay(img, saved_count, next_index, batch_dir, frame_count=0,
                 mean_value=None):
    try:
        img.draw_string_advanced(
            10,
            10,
            20,
            "save:%d next:%06d" % (saved_count, next_index),
            color=(255, 0, 0),
        )
        img.draw_string_advanced(
            10,
            36,
            18,
            "%s %s %s" % (safe_token(SCENE), safe_token(MOTION), safe_token(LIGHTING)),
            color=(255, 255, 0),
        )
        img.draw_string_advanced(
            10,
            60,
            16,
            remaining_space_text(batch_dir),
            color=(0, 255, 0),
        )
        img.draw_string_advanced(
            10,
            82,
            16,
            "frame:%d mean:%s" % (frame_count, mean_value),
            color=(0, 255, 255),
        )
    except Exception:
        pass


def frame_mean(image_obj):
    try:
        stats = image_obj.get_statistics()
        if hasattr(stats, "l_mean"):
            return int(stats.l_mean())
        if hasattr(stats, "mean"):
            return int(stats.mean())
    except Exception:
        pass
    return None


def frame_is_too_dark(image_obj):
    mean = frame_mean(image_obj)
    if mean is None:
        return False
    return mean < MIN_FRAME_MEAN


def should_capture(mode, saved_count, start_time, last_save_time,
                   key_pressed=False):
    if saved_count >= MAX_SAVE_COUNT:
        return False
    if mode == "single":
        return saved_count == 0
    if mode == "countdown":
        return saved_count == 0 and ticks_diff(ticks_ms(), start_time) >= COUNTDOWN_SECONDS * 1000
    if mode == "interval":
        return ticks_diff(ticks_ms(), last_save_time) >= SAVE_INTERVAL_MS
    if mode == "key":
        return key_pressed
    return False


def run():
    from media.sensor import Sensor
    from media.display import Display
    from media.media import MediaManager

    sensor = None
    batch_dir = join_path(DATASET_ROOT, safe_token(BATCH_NAME))
    metadata_path = join_path(batch_dir, "metadata.txt")
    saved_count = 0
    frame_count = 0
    current_index = START_INDEX
    start_time = ticks_ms()
    last_save_time = start_time - SAVE_INTERVAL_MS
    key_controller = None

    try:
        ensure_dir(batch_dir)
        append_metadata(metadata_path, "")

        sensor = Sensor()
        sensor.reset()
        sensor.set_framesize(width=FRAME_WIDTH, height=FRAME_HEIGHT)
        sensor.set_pixformat(Sensor.RGB565)

        Display.init(Display.ST7701, width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT, to_ide=True)
        MediaManager.init()
        sensor.run()

        print("[capture] batch_dir=%s" % batch_dir)
        print("[capture] mode=%s scene=%s lighting=%s motion=%s" % (
            CAPTURE_MODE, SCENE, LIGHTING, MOTION
        ))
        print("[capture] max=%d interval_ms=%d warmup=%d" % (
            MAX_SAVE_COUNT, SAVE_INTERVAL_MS, WARMUP_FRAMES
        ))
        if CAPTURE_MODE == "key":
            key_controller = init_capture_key()
            print("[capture] key=KEY1 gpio_pin=%d gpio_num=%d active_level=%d debounce_ms=%d" % (
                KEY_GPIO_PIN, KEY_GPIO_NUM, KEY_ACTIVE_LEVEL, KEY_DEBOUNCE_MS
            ))

        while saved_count < MAX_SAVE_COUNT:
            try:
                os.exitpoint()
            except Exception:
                pass

            raw_img = sensor.snapshot()
            frame_count += 1
            current_index, path = next_available_image_path(
                batch_dir,
                BATCH_NAME,
                SCENE,
                MOTION,
                current_index,
            )

            warmup_done = frame_count > WARMUP_FRAMES
            mean_value = frame_mean(raw_img)
            dark_frame = (
                mean_value is not None and mean_value < MIN_FRAME_MEAN
            )

            key_pressed = False
            if CAPTURE_MODE == "key" and key_controller is not None:
                key_pressed = is_capture_key_pressed_edge(key_controller)
            if (warmup_done and not dark_frame
                    and should_capture(CAPTURE_MODE, saved_count, start_time, last_save_time,
                                       key_pressed=key_pressed)):
                filename = path[path.rfind("/") + 1:]
                meta_line = metadata_line(
                    filename, BATCH_NAME, SCENE, LIGHTING, MOTION, NOTE
                )
                # Pre-check metadata conflict before saving image
                try:
                    if check_metadata_conflict(metadata_path, meta_line):
                        print("[metadata] duplicate skipped: %s" % filename)
                        saved_count += 1
                        current_index += 1
                        last_save_time = ticks_ms()
                        continue
                except MetadataConflict as exc:
                    print("[metadata] CONFLICT STOPPED: %s" % exc)
                    sleep_ms(1000)
                    continue
                try:
                    save_image(raw_img, path)
                    append_metadata(metadata_path, meta_line)
                    saved_count += 1
                    current_index += 1
                    last_save_time = ticks_ms()
                    if CAPTURE_MODE == "key":
                        print("[KEY CAPTURE] saved: %s" % path)
                        print("count: %d/%d" % (saved_count, MAX_SAVE_COUNT))
                    else:
                        print("[capture] saved %s" % path)
                except Exception as exc:
                    print("[capture] save failed path=%s error=%s" % (path, exc))
                    sleep_ms(1000)
            elif warmup_done and dark_frame and saved_count == 0:
                print("[capture] skip dark frame mean=%s" % mean_value)

            preview_img = raw_img
            try:
                preview_img = raw_img.copy()
            except Exception:
                pass
            draw_overlay(preview_img, saved_count, current_index, batch_dir,
                         frame_count, mean_value)
            Display.show_image(preview_img, 0, 0, Display.LAYER_OSD2)

            if CAPTURE_MODE in ("single", "countdown") and saved_count >= 1:
                break
            sleep_ms(30)

        print("[capture] finished saved=%d dir=%s" % (saved_count, batch_dir))

    finally:
        try:
            if sensor is not None:
                sensor.stop()
        except Exception:
            pass
        try:
            Display.deinit()
        except Exception:
            pass
        try:
            os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        except Exception:
            pass
        sleep_ms(100)
        try:
            MediaManager.deinit()
        except Exception:
            pass


if __name__ == "__main__":
    run()
