import gc
import os
import sensor
import sys
import tf
import time


MODEL1_PATH = "/sd/mobilenet_v2-2026-07-10T07-36-10.164Z_in-int8_out-int8_channel_ptq.tflite"
LABELS1_PATH = "/sd/labels.txt"
MODEL2_PATH = "/sd/mobilenet_v2-2026-07-13T09-04-45.448Z_channel_ptq.tflite"
LABELS2_PATH = "/sd/labels2.txt"

ROI = (60, 70, 225, 170)
TEST_FRAME_COUNT = 20


def print_error(stage, error):
    print(stage)
    try:
        sys.print_exception(error)
    except Exception:
        print(repr(error))


def check_file(path):
    try:
        stat = os.stat(path)
        print("FILE_OK", path, "SIZE", stat[6])
        return True
    except Exception as error:
        print_error("FILE_FAIL " + path, error)
        return False


def load_labels(name, path):
    try:
        with open(path, "r") as label_file:
            labels = [line.rstrip("\r\n") for line in label_file]
    except Exception as error:
        print_error(name + "_LABEL_LOAD_FAIL", error)
        return None

    print(name + "_LABEL_COUNT", len(labels))
    for index, label_name in enumerate(labels):
        print(name + "_LABEL", index, repr(label_name))

    if len(labels) == 0:
        print(name + "_LABEL_EMPTY")
        return None

    return labels


def run_model(name, model_path, label_path):
    model = None
    passed = True
    completed_frames = 0
    max_infer_ms = 0

    labels = load_labels(name, label_path)
    if labels is None:
        return False, 0

    try:
        print(name + "_LOAD_START", model_path)
        model = tf.load(model_path, load_to_fb=True)
        print(name + "_LOAD_OK")
    except Exception as error:
        print_error(name + "_LOAD_FAIL", error)
        return False, 0

    try:
        for frame_id in range(1, TEST_FRAME_COUNT + 1):
            print(name, "FRAME", frame_id, "INFER_START")

            try:
                img = sensor.snapshot()
                start_ms = time.ticks_ms()
                result_count = 0
                frame_valid = True

                for result_count, obj in enumerate(
                    tf.classify(
                        model,
                        img,
                        roi=ROI,
                        min_scale=1.0,
                        scale_mul=0.5,
                        x_overlap=0.0,
                        y_overlap=0.0,
                    ),
                    1,
                ):
                    output = obj.output()
                    output_count = len(output)

                    print(
                        name,
                        "FRAME",
                        frame_id,
                        "RESULT",
                        result_count,
                        "OUTPUT_COUNT",
                        output_count,
                        "LABEL_COUNT",
                        len(labels),
                    )

                    if output_count == 0:
                        print(name, "FRAME", frame_id, "EMPTY_OUTPUT")
                        frame_valid = False
                        break

                    if output_count != len(labels):
                        print(
                            name,
                            "FRAME",
                            frame_id,
                            "OUTPUT_LABEL_COUNT_MISMATCH",
                        )
                        frame_valid = False
                        break

                    best_index = 0
                    best_score = output[0]
                    for output_index in range(1, output_count):
                        if output[output_index] > best_score:
                            best_index = output_index
                            best_score = output[output_index]

                    print(
                        name,
                        "FRAME",
                        frame_id,
                        "BEST_INDEX",
                        best_index,
                        "BEST_LABEL",
                        repr(labels[best_index]),
                        "CONFIDENCE",
                        best_score,
                        "MAPPED_LABEL",
                        best_index + 1,
                    )

                infer_ms = time.ticks_diff(time.ticks_ms(), start_ms)
                if infer_ms > max_infer_ms:
                    max_infer_ms = infer_ms

                print(name, "FRAME", frame_id, "INFER_DONE", infer_ms, "MS")

                if result_count == 0:
                    print(name, "FRAME", frame_id, "NO_CLASSIFY_RESULT")
                    frame_valid = False

                img.draw_rectangle(ROI, color=(255, 255, 255))

                if not frame_valid:
                    passed = False
                    break

                completed_frames += 1
            except Exception as error:
                print_error(name + "_INFER_FAIL_FRAME_" + str(frame_id), error)
                passed = False
                break
    finally:
        model = None
        gc.collect()
        print(name + "_RELEASE_DONE")

    if completed_frames != TEST_FRAME_COUNT:
        passed = False

    print(name + "_COMPLETED_FRAMES", completed_frames)
    print(name + "_MAX_INFER_MS", max_infer_ms)
    return passed, max_infer_ms


print("LABEL_DEBUG_V1")

check_file(MODEL1_PATH)
check_file(LABELS1_PATH)
check_file(MODEL2_PATH)
check_file(LABELS2_PATH)

sensor_ready = True
try:
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.set_hmirror(True)
    sensor.skip_frames(time = 200)
    sensor.set_vflip(True)
    sensor.skip_frames(time=1000)
    print("CAMERA_INIT_OK")
except Exception as error:
    print_error("CAMERA_INIT_FAIL", error)
    sensor_ready = False

model1_pass = False
model2_pass = False
model1_max_ms = 0
model2_max_ms = 0

if sensor_ready:
    model1_pass, model1_max_ms = run_model(
        "MODEL1", MODEL1_PATH, LABELS1_PATH
    )
    gc.collect()
    model2_pass, model2_max_ms = run_model(
        "MODEL2", MODEL2_PATH, LABELS2_PATH
    )

if model1_pass:
    print("MODEL1 PASS")
else:
    print("MODEL1 FAIL")

if model2_pass:
    print("MODEL2 PASS")
else:
    print("MODEL2 FAIL")

print("MODEL1 MAX_INFER_MS", model1_max_ms)
print("MODEL2 MAX_INFER_MS", model2_max_ms)
print("DIAG_DONE")

while True:
    time.sleep_ms(1000)
