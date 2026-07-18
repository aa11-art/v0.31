import sensor
import sys
import time


# 修改这里确定搜索区域：(x, y, width, height)
ROI = (68, 70, 210, 170)


def print_error(stage, error):
    print(stage)
    try:
        sys.print_exception(error)
    except Exception:
        print(repr(error))


try:
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.set_hmirror(True)
    sensor.skip_frames(time=200)
    sensor.set_vflip(True)
    sensor.skip_frames(time=1000)
    sensor.set_auto_whitebal(False)
    sensor.set_auto_gain(False, gain_db=0)
    sensor.set_auto_exposure(False, exposure_us=500)
    sensor.skip_frames(time=300)

    if (
        ROI[0] < 0
        or ROI[1] < 0
        or ROI[2] <= 0
        or ROI[3] <= 0
        or (ROI[0] + ROI[2]) > 320
        or (ROI[1] + ROI[3]) > 240
    ):
        raise ValueError("ROI must stay inside QVGA 320x240")

    print("ROI_BOX_TEST_READY", ROI)

    while True:
        img = sensor.snapshot()
        img.draw_rectangle(ROI, color=(255, 255, 255))
        time.sleep_ms(10)
except Exception as error:
    print_error("ROI_BOX_TEST_FAIL", error)

while True:
    time.sleep_ms(1000)
