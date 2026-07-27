"""Red-line LAB threshold calibration utility for CanMV K230.

Run this script on the K230 board via the IDE to visualise the current
red threshold and receive LAB-statistic suggestions.

Usage:
    1. Edit RED_LAB_THRESHOLD in drug_car/config.py.
    2. Run this script from the IDE.
    3. Observe the masked overlay and terminal suggestions.
    4. Iterate until the red line is cleanly segmented.

This utility does NOT write config.py automatically.
"""

# Board-compatible import: on /sdcard the package is drug_car,
# on the host PC it is deploy.drug_car.
try:
    from drug_car.config import (
        DEBUG_DRAW,
        DISPLAY_MODE,
        DISPLAY_TO_IDE,
        FRAME_HEIGHT,
        FRAME_WIDTH,
        GC_INTERVAL_FRAMES,
        LINE_ROIS,
        PIXFORMAT,
        RED_LAB_THRESHOLD,
    )
except ImportError:
    from deploy.drug_car.config import (  # type: ignore[no-redef]
        DEBUG_DRAW,
        DISPLAY_MODE,
        DISPLAY_TO_IDE,
        FRAME_HEIGHT,
        FRAME_WIDTH,
        GC_INTERVAL_FRAMES,
        LINE_ROIS,
        PIXFORMAT,
        RED_LAB_THRESHOLD,
    )


def suggest_lab_threshold(statistics, margin=10):
    """Suggest a LAB threshold tuple with margin around observed
    min/max values.  Clamped to valid LAB ranges."""
    l_lo = max(0, statistics["l_min"] - margin)
    l_hi = min(100, statistics["l_max"] + margin)
    a_lo = max(-128, statistics["a_min"] - margin)
    a_hi = min(127, statistics["a_max"] + margin)
    b_lo = max(-128, statistics["b_min"] - margin)
    b_hi = min(127, statistics["b_max"] + margin)
    return (l_lo, l_hi, a_lo, a_hi, b_lo, b_hi)


def _resolve_display_mode(display_module, mode):
    """Resolve a string DISPLAY_MODE like 'ST7701' to the Display
    constant Display.ST7701.  Pass through non-string values."""
    if isinstance(mode, str) and hasattr(display_module, mode):
        return getattr(display_module, mode)
    return mode


def _cleanup(sensor, display, media):
    """Idempotent cleanup in the approved design sequence."""
    for name, action in (
        ("Sensor", lambda: sensor.stop()),
    ):
        try:
            action()
            print("%s stopped" % name)
        except Exception:
            print("%s release failed" % name)

    try:
        import os
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    except (ImportError, AttributeError):
        pass
    try:
        import time
        time.sleep_ms(100)
    except (ImportError, AttributeError):
        pass

    for name, action in (
        ("Display", lambda: display.deinit()),
        ("Media", lambda: media.deinit()),
    ):
        try:
            action()
            print("%s released" % name)
        except Exception:
            print("%s release failed" % name)


def main():
    try:
        from media.sensor import Sensor
        from media.display import Display
        from media.media import MediaManager
    except ImportError:
        print("This script requires CanMV K230 hardware.")
        return

    sensor = Sensor()
    display = Display
    media = MediaManager

    try:
        sensor.reset()
        sensor.set_framesize(width=FRAME_WIDTH, height=FRAME_HEIGHT)
        sensor.set_pixformat(getattr(Sensor, PIXFORMAT, 0))
        display.init(_resolve_display_mode(Display, DISPLAY_MODE),
                     to_ide=DISPLAY_TO_IDE)
        media.init()
        sensor.run()

        frame_count = 0

        while True:
            try:
                import os
                os.exitpoint()
            except (ImportError, AttributeError):
                pass

            image = sensor.snapshot()
            frame_count += 1

            for rx, ry, rw, rh, _weight in LINE_ROIS:
                image.draw_rectangle(rx, ry, rw, rh, color=(0, 255, 0),
                                     thickness=1)

            try:
                stats = image.get_statistics(
                    roi=LINE_ROIS[0][:4],
                    thresholds=[RED_LAB_THRESHOLD],
                )
            except Exception:
                stats = None

            if stats is not None and hasattr(stats, "l_mean"):
                try:
                    suggestion = suggest_lab_threshold({
                        "l_min": int(stats.l_min()),
                        "l_max": int(stats.l_max()),
                        "a_min": int(stats.a_min()),
                        "a_max": int(stats.a_max()),
                        "b_min": int(stats.b_min()),
                        "b_max": int(stats.b_max()),
                    })
                    print("Suggested RED_LAB_THRESHOLD =", suggestion)
                except Exception:
                    pass

            try:
                blobs = image.find_blobs(
                    [RED_LAB_THRESHOLD],
                    roi=LINE_ROIS[0][:4],
                    pixels_threshold=80,
                    area_threshold=80,
                    merge=True,
                )
                for blob in blobs:
                    image.draw_rectangle(
                        blob.rect(), color=(255, 0, 0), thickness=2,
                    )
            except Exception:
                pass

            text = "RED CALIB | frames: %d" % (frame_count,)
            image.draw_string_advanced(10, 10, 14, text, color=(255, 255, 0))

            display.show_image(image)

            if frame_count % GC_INTERVAL_FRAMES == 0:
                try:
                    import gc
                    gc.collect()
                except ImportError:
                    pass

    finally:
        _cleanup(sensor, display, media)


if __name__ == "__main__":
    main()
