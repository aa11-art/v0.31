import gc
import image
import sensor
import sys
import time


TEMPLATE_DIRECTORY = "/sd/templates"
TEMPLATE_COUNT = 10
TEMPLATE_WIDTH = 180
TEMPLATE_HEIGHT = 165
TEMPLATE_SCORE_THRESHOLD = 0.80
ROI = (95, 70, TEMPLATE_WIDTH, TEMPLATE_HEIGHT)

EXPECTED_DIGIT = 1
TEST_FRAME_COUNT = 20

COARSE_WIDTH = 45
COARSE_HEIGHT = 41
X_OFFSETS = (-8, -4, 0, 4, 8)
Y_OFFSETS = (-8, -4, 0, 4)
SCALE_FACTORS = (0.94, 1.00, 1.06)
ROTATION_ANGLES = (-4.0, 0.0, 4.0)

FRAME_WIDTH = 320
FRAME_HEIGHT = 240


def print_error(stage, error):
    print(stage)
    try:
        sys.print_exception(error)
    except Exception:
        print(repr(error))


def template_path(digit):
    return "%s/%d.bmp" % (TEMPLATE_DIRECTORY, digit)


def difference_score(source, reference):
    difference_img = source.copy()
    try:
        difference_img.difference(reference)
        mean_difference = difference_img.get_statistics()[0]
        return 1.0 - (mean_difference / 255.0)
    finally:
        difference_img = None


def scaled_grayscale_copy(source, roi, target_width, target_height):
    result = source.copy(
        roi=roi,
        x_scale=target_width / float(roi[2]),
        y_scale=target_height / float(roi[3]),
    )
    result = result.to_grayscale()
    if result.width() != target_width or result.height() != target_height:
        print(
            "SCALED_SIZE_MISMATCH",
            result.width(),
            result.height(),
            "EXPECTED",
            target_width,
            target_height,
        )
        result = None
        return None
    return result


def candidate_roi(x_offset, y_offset, scale):
    crop_width = int((TEMPLATE_WIDTH * scale) + 0.5)
    crop_height = int((TEMPLATE_HEIGHT * scale) + 0.5)
    center_x = ROI[0] + (ROI[2] / 2.0) + x_offset
    center_y = ROI[1] + (ROI[3] / 2.0) + y_offset
    x = int(center_x - (crop_width / 2.0) + 0.5)
    y = int(center_y - (crop_height / 2.0) + 0.5)

    if x < 0 or y < 0:
        return None
    if (x + crop_width) > FRAME_WIDTH:
        return None
    if (y + crop_height) > FRAME_HEIGHT:
        return None
    return (x, y, crop_width, crop_height)


def load_coarse_templates():
    templates = []
    for digit in range(TEMPLATE_COUNT):
        full_template = None
        coarse_template = None
        path = template_path(digit)
        try:
            full_template = image.Image(path, copy_to_fb=False)
            full_template = full_template.to_grayscale()
            if (
                full_template.width() != TEMPLATE_WIDTH
                or full_template.height() != TEMPLATE_HEIGHT
            ):
                print(
                    "TEMPLATE_SIZE_MISMATCH",
                    path,
                    full_template.width(),
                    full_template.height(),
                )
                return None
            coarse_template = full_template.copy(
                x_scale=COARSE_WIDTH / float(TEMPLATE_WIDTH),
                y_scale=COARSE_HEIGHT / float(TEMPLATE_HEIGHT),
            )
            if (
                coarse_template.width() != COARSE_WIDTH
                or coarse_template.height() != COARSE_HEIGHT
            ):
                print(
                    "COARSE_TEMPLATE_SIZE_MISMATCH",
                    digit,
                    coarse_template.width(),
                    coarse_template.height(),
                )
                return None
            templates.append(coarse_template)
            coarse_template = None
            print("TEMPLATE_READY", digit, path)
        except Exception as error:
            print_error("TEMPLATE_LOAD_FAIL " + path, error)
            return None
        finally:
            full_template = None
            coarse_template = None
            gc.collect()
    return templates


def detect_rotation_support():
    probe = None
    try:
        probe = sensor.snapshot().copy(roi=(ROI[0], ROI[1], 32, 32))
        probe = probe.to_grayscale()
        probe.rotation_corr(z_rotation=1.0)
        print("ROTATION_SEARCH_AVAILABLE")
        return True
    except Exception as error:
        print("ROTATION_SEARCH_UNAVAILABLE")
        print_error("ROTATION_PROBE_FAIL", error)
        return False
    finally:
        probe = None
        gc.collect()


def coarse_search(snapshot, coarse_templates):
    best_scores = [-2.0] * TEMPLATE_COUNT
    best_geometries = [None] * TEMPLATE_COUNT
    valid_candidate_count = 0

    for scale in SCALE_FACTORS:
        for y_offset in Y_OFFSETS:
            for x_offset in X_OFFSETS:
                roi = candidate_roi(x_offset, y_offset, scale)
                if roi is None:
                    continue

                candidate = None
                try:
                    candidate = scaled_grayscale_copy(
                        snapshot,
                        roi,
                        COARSE_WIDTH,
                        COARSE_HEIGHT,
                    )
                    if candidate is None:
                        continue
                    valid_candidate_count += 1

                    for digit in range(TEMPLATE_COUNT):
                        score = difference_score(
                            candidate,
                            coarse_templates[digit],
                        )
                        if score > best_scores[digit]:
                            best_scores[digit] = score
                            best_geometries[digit] = (
                                x_offset,
                                y_offset,
                                scale,
                                roi,
                            )
                finally:
                    candidate = None
        gc.collect()

    print("COARSE_VALID_CANDIDATES", valid_candidate_count)
    return best_scores, best_geometries


def fine_search(snapshot, coarse_scores, best_geometries, rotation_supported):
    digit_results = []
    angles = ROTATION_ANGLES if rotation_supported else (0.0,)

    for digit in range(TEMPLATE_COUNT):
        geometry = best_geometries[digit]
        if geometry is None:
            digit_results.append(None)
            print("DIGIT_FINE_SKIP", digit, "NO_GEOMETRY")
            continue

        template = None
        normalized = None
        best_score = -2.0
        best_angle = 0.0
        try:
            template = image.Image(template_path(digit), copy_to_fb=False)
            template = template.to_grayscale()
            if (
                template.width() != TEMPLATE_WIDTH
                or template.height() != TEMPLATE_HEIGHT
            ):
                print("DIGIT_FINE_SKIP", digit, "BAD_TEMPLATE_SIZE")
                digit_results.append(None)
                continue

            normalized = scaled_grayscale_copy(
                snapshot,
                geometry[3],
                TEMPLATE_WIDTH,
                TEMPLATE_HEIGHT,
            )
            if normalized is None:
                digit_results.append(None)
                continue

            for angle in angles:
                corrected = None
                try:
                    corrected = normalized.copy()
                    if angle != 0.0:
                        corrected.rotation_corr(z_rotation=angle)
                    score = difference_score(corrected, template)
                    if score > best_score:
                        best_score = score
                        best_angle = angle
                except Exception as error:
                    print_error(
                        "ANGLE_SEARCH_FAIL_%d_%s" % (digit, str(angle)),
                        error,
                    )
                finally:
                    corrected = None

            digit_results.append(
                (
                    best_score,
                    geometry[0],
                    geometry[1],
                    geometry[2],
                    best_angle,
                    geometry[3],
                )
            )
            print(
                "DIGIT_RESULT",
                digit,
                "COARSE_SCORE",
                coarse_scores[digit],
                "FINE_SCORE",
                best_score,
                "DX",
                geometry[0],
                "DY",
                geometry[1],
                "SCALE",
                geometry[2],
                "ANGLE",
                best_angle,
            )
        except Exception as error:
            print_error("DIGIT_FINE_FAIL_%d" % digit, error)
            digit_results.append(None)
        finally:
            normalized = None
            template = None
            gc.collect()

    return digit_results


def select_best_result(digit_results):
    best_digit = -1
    best_score = -2.0
    second_score = -2.0
    best_result = None

    for digit in range(TEMPLATE_COUNT):
        result = digit_results[digit]
        if result is None:
            continue
        score = result[0]
        if score > best_score:
            second_score = best_score
            best_score = score
            best_digit = digit
            best_result = result
        elif score > second_score:
            second_score = score

    return best_digit, best_score, second_score, best_result


def run_frame(frame_id, coarse_templates, rotation_supported):
    start_ms = time.ticks_ms()
    snapshot = sensor.snapshot()
    print("FRAME_START", frame_id)

    coarse_scores, best_geometries = coarse_search(
        snapshot,
        coarse_templates,
    )
    digit_results = fine_search(
        snapshot,
        coarse_scores,
        best_geometries,
        rotation_supported,
    )
    best_digit, best_score, second_score, best_result = select_best_result(
        digit_results
    )

    accepted = (
        best_digit >= 0 and best_score >= TEMPLATE_SCORE_THRESHOLD
    )
    correct = accepted and best_digit == EXPECTED_DIGIT
    margin = best_score - second_score
    elapsed_ms = time.ticks_diff(time.ticks_ms(), start_ms)

    snapshot.draw_rectangle(ROI, color=(255, 255, 255))
    if best_result is not None:
        snapshot.draw_rectangle(best_result[5], color=(255, 0, 0))

    if best_result is None:
        print(
            "FRAME_RESULT",
            frame_id,
            "EXPECTED",
            EXPECTED_DIGIT,
            "NO_VALID_RESULT",
            "ELAPSED_MS",
            elapsed_ms,
        )
    else:
        print(
            "FRAME_RESULT",
            frame_id,
            "EXPECTED",
            EXPECTED_DIGIT,
            "BEST_DIGIT",
            best_digit,
            "BEST_SCORE",
            best_score,
            "SECOND_SCORE",
            second_score,
            "MARGIN",
            margin,
            "DX",
            best_result[1],
            "DY",
            best_result[2],
            "SCALE",
            best_result[3],
            "ANGLE",
            best_result[4],
            "ACCEPTED",
            accepted,
            "CORRECT",
            correct,
            "ELAPSED_MS",
            elapsed_ms,
        )

    snapshot = None
    digit_results = None
    best_geometries = None
    gc.collect()
    return accepted, correct, elapsed_ms


print("DIGIT_ALIGNMENT_TEST_V1")
print("EXPECTED_DIGIT", EXPECTED_DIGIT)
print("TEST_FRAME_COUNT", TEST_FRAME_COUNT)
print("TEMPLATE_SCORE_THRESHOLD", TEMPLATE_SCORE_THRESHOLD)
print("ROI", ROI)

if EXPECTED_DIGIT < 0 or EXPECTED_DIGIT >= TEMPLATE_COUNT:
    print("EXPECTED_DIGIT_INVALID", EXPECTED_DIGIT)
    while True:
        time.sleep_ms(1000)

camera_ready = True
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
    print("CAMERA_INIT_OK")
except Exception as error:
    print_error("CAMERA_INIT_FAIL", error)
    camera_ready = False

coarse_templates = None
rotation_supported = False
if camera_ready:
    coarse_templates = load_coarse_templates()
    if coarse_templates is not None:
        rotation_supported = detect_rotation_support()

correct_count = 0
wrong_accepted_count = 0
rejected_count = 0
completed_frames = 0
elapsed_sum_ms = 0
max_elapsed_ms = 0

if camera_ready and coarse_templates is not None:
    for frame_id in range(1, TEST_FRAME_COUNT + 1):
        try:
            accepted, correct, elapsed_ms = run_frame(
                frame_id,
                coarse_templates,
                rotation_supported,
            )
            completed_frames += 1
            elapsed_sum_ms += elapsed_ms
            if elapsed_ms > max_elapsed_ms:
                max_elapsed_ms = elapsed_ms

            if correct:
                correct_count += 1
            elif accepted:
                wrong_accepted_count += 1
            else:
                rejected_count += 1
        except Exception as error:
            print_error("FRAME_FAIL_%d" % frame_id, error)
            completed_frames += 1
            rejected_count += 1
            gc.collect()

accuracy_percent = 0.0
rejection_percent = 0.0
average_elapsed_ms = 0.0
if completed_frames > 0:
    accuracy_percent = 100.0 * correct_count / completed_frames
    rejection_percent = 100.0 * rejected_count / completed_frames
    average_elapsed_ms = elapsed_sum_ms / float(completed_frames)

print("TEST_COMPLETED_FRAMES", completed_frames)
print("CORRECT_ACCEPTED", correct_count)
print("WRONG_ACCEPTED", wrong_accepted_count)
print("REJECTED", rejected_count)
print(
    "RESULT_COUNT_CHECK",
    correct_count + wrong_accepted_count + rejected_count,
    "EXPECTED",
    TEST_FRAME_COUNT,
)
print("ACCURACY_PERCENT", accuracy_percent)
print("REJECTION_PERCENT", rejection_percent)
print("AVERAGE_ELAPSED_MS", average_elapsed_ms)
print("MAX_ELAPSED_MS", max_elapsed_ms)
print("ROTATION_SUPPORTED", rotation_supported)
print("DIGIT_ALIGNMENT_TEST_DONE")

while True:
    time.sleep_ms(1000)
