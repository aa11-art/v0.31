import gc
import image
import sensor
import sys
import time


TEMPLATE_DIRECTORY = "/sd/templates"
TEMPLATE_COUNT = 10
EXPECTED_DIGIT = 6
TEST_FRAME_COUNT = 20
MAX_TEST_FRAMES_PER_RESULT = 5

# 实测搜索区域，只用于寻找箱子正面的白色面板。
SEARCH_ROI = (68, 70, 210, 170)
# 目标大致出现的原始范围，用于面板检测失败时提取中央数字。
APPROX_TARGET_ROI = (95, 70, 180, 165)
APPROX_TARGET_ROI_LOCAL = (
    APPROX_TARGET_ROI[0] - SEARCH_ROI[0],
    APPROX_TARGET_ROI[1] - SEARCH_ROI[1],
    APPROX_TARGET_ROI[2],
    APPROX_TARGET_ROI[3],
)
RECT_THRESHOLD = 5000
MIN_PANEL_WIDTH = 60
MIN_PANEL_HEIGHT = 50
MIN_PANEL_MEAN = 120
MIN_DIGIT_RATIO = 0.01
MAX_DIGIT_RATIO = 0.45

DIGIT_CANVAS_WIDTH = 64
DIGIT_CANVAS_HEIGHT = 96
DIGIT_CANVAS_MARGIN = 4
TEMPLATE_SCORE_THRESHOLD = 0.83
MIN_SCORE_MARGIN = 0.05


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


def automatic_threshold(gray_img):
    try:
        threshold = gray_img.get_histogram().get_threshold().value()
    except Exception:
        threshold = 128
    if threshold < 30:
        threshold = 30
    elif threshold > 220:
        threshold = 220
    return threshold


def choose_digit_blob(binary_img):
    width = binary_img.width()
    height = binary_img.height()
    # 忽略面板左右边缘，避免把墙体或箱体边框当成数字的一部分。
    margin_x = max(8, width // 20)
    margin_y = max(2, height // 40)
    search_roi = (
        margin_x,
        margin_y,
        width - (2 * margin_x),
        height - (2 * margin_y),
    )
    blobs = binary_img.find_blobs(
        [(200, 255)],
        roi=search_roi,
        pixels_threshold=max(20, (width * height) // 1000),
        area_threshold=max(20, (width * height) // 1000),
        merge=False,
    )

    best_blob = None
    best_rank = -1000000
    image_center_x = width // 2
    image_center_y = height // 2

    for blob in blobs:
        blob_rect = blob.rect()
        blob_width = blob_rect[2]
        blob_height = blob_rect[3]
        blob_center_x = blob_rect[0] + (blob_width // 2)
        blob_center_y = blob_rect[1] + (blob_height // 2)

        # 排除墙体边缘或底部色带形成的宽而扁黑块。
        if blob_width > ((width * 8) // 10) and blob_height < (height // 4):
            continue
        if blob_width < 3 or blob_height < 8:
            continue
        # 触碰数字搜索区左右边界的黑块通常来自墙体、箱体边缘或背景。
        if (
            blob_rect[0] <= (margin_x + 1)
            or (blob_rect[0] + blob_width) >= (width - margin_x - 1)
        ):
            continue

        center_distance = (
            abs(blob_center_x - image_center_x)
            + abs(blob_center_y - image_center_y)
        )
        rank = blob.pixels() - (2 * center_distance)
        if rank > best_rank:
            best_rank = rank
            best_blob = blob

    return best_blob


def extract_normalized_digit(gray_source):
    binary_img = None
    digit_crop = None
    canvas = None
    try:
        threshold = automatic_threshold(gray_source)
        binary_img = gray_source.copy()
        # 黑色数字变为白色前景，白色面板变为黑色背景。
        binary_img.binary([(0, threshold)])
        digit_blob = choose_digit_blob(binary_img)
        if digit_blob is None:
            return None, threshold, 0.0, None

        blob_rect = digit_blob.rect()
        digit_ratio = digit_blob.pixels() / float(
            gray_source.width() * gray_source.height()
        )
        digit_crop = binary_img.copy(roi=blob_rect)

        available_width = DIGIT_CANVAS_WIDTH - (2 * DIGIT_CANVAS_MARGIN)
        available_height = DIGIT_CANVAS_HEIGHT - (2 * DIGIT_CANVAS_MARGIN)
        scale_x = available_width / float(digit_crop.width())
        scale_y = available_height / float(digit_crop.height())
        scale = scale_x if scale_x < scale_y else scale_y
        scaled_width = int((digit_crop.width() * scale) + 0.5)
        scaled_height = int((digit_crop.height() * scale) + 0.5)
        draw_x = (DIGIT_CANVAS_WIDTH - scaled_width) // 2
        draw_y = (DIGIT_CANVAS_HEIGHT - scaled_height) // 2

        canvas = image.Image(
            DIGIT_CANVAS_WIDTH,
            DIGIT_CANVAS_HEIGHT,
            sensor.GRAYSCALE,
        )
        canvas.clear()
        canvas.draw_image(
            digit_crop,
            draw_x,
            draw_y,
            x_scale=scale,
            y_scale=scale,
        )
        return canvas, threshold, digit_ratio, blob_rect
    finally:
        binary_img = None
        digit_crop = None


def load_normalized_templates():
    templates = []
    for digit in range(TEMPLATE_COUNT):
        template = None
        normalized = None
        path = template_path(digit)
        try:
            template = image.Image(path, copy_to_fb=False)
            template = template.to_grayscale()
            normalized, threshold, digit_ratio, blob_rect = (
                extract_normalized_digit(template)
            )
            if normalized is None:
                print("TEMPLATE_DIGIT_NOT_FOUND", digit, path)
                return None
            templates.append(normalized)
            normalized = None
            print(
                "TEMPLATE_READY",
                digit,
                "THRESHOLD",
                threshold,
                "DIGIT_RATIO",
                digit_ratio,
                "BLOB",
                blob_rect,
            )
        except Exception as error:
            print_error("TEMPLATE_LOAD_FAIL " + path, error)
            return None
        finally:
            template = None
            normalized = None
            gc.collect()
    return templates


def rectify_panel(search_gray, rect):
    panel = None
    try:
        rect_box = rect.rect()
        corners = rect.corners()
        panel = search_gray.copy(roi=rect_box)
        local_corners = []
        for corner in corners:
            local_corners.append(
                (corner[0] - rect_box[0], corner[1] - rect_box[1])
            )
        panel.rotation_corr(corners=local_corners)
        return panel
    except Exception as error:
        panel = None
        print_error("PANEL_RECTIFY_FAIL", error)
        return None


def find_white_panel_fallback(search_gray):
    approximate = None
    white_mask = None
    panel = None
    normalized = None
    try:
        approximate = search_gray.copy(roi=APPROX_TARGET_ROI_LOCAL)
        white_threshold = automatic_threshold(approximate)
        if white_threshold < 150:
            white_threshold = 150

        white_mask = approximate.copy()
        white_mask.binary([(white_threshold, 255)])
        white_blobs = white_mask.find_blobs(
            [(200, 255)],
            pixels_threshold=300,
            area_threshold=600,
            merge=True,
            margin=10,
        )

        best_white_blob = None
        best_white_pixels = 0
        for blob in white_blobs:
            blob_rect = blob.rect()
            if (
                blob_rect[2] < MIN_PANEL_WIDTH
                or blob_rect[3] < MIN_PANEL_HEIGHT
            ):
                continue
            if blob.pixels() > best_white_pixels:
                best_white_pixels = blob.pixels()
                best_white_blob = blob

        candidate_rois = []
        if best_white_blob is not None:
            candidate_rois.append(("WHITE_BLOB", best_white_blob.rect()))
        candidate_rois.append(
            (
                "APPROX_ROI",
                (0, 0, approximate.width(), approximate.height()),
            )
        )

        for mode, candidate in candidate_rois:
            panel = approximate.copy(roi=candidate)
            panel_mean = panel.get_statistics()[0]
            if panel_mean < MIN_PANEL_MEAN:
                print(
                    "PANEL_FALLBACK_REJECT_MEAN",
                    mode,
                    candidate,
                    panel_mean,
                )
                panel = None
                continue

            normalized, threshold, digit_ratio, blob_rect = (
                extract_normalized_digit(panel)
            )
            if normalized is None:
                print("PANEL_FALLBACK_REJECT_NO_DIGIT", mode, candidate)
                panel = None
                continue
            if (
                digit_ratio < MIN_DIGIT_RATIO
                or digit_ratio > MAX_DIGIT_RATIO
            ):
                print(
                    "PANEL_FALLBACK_REJECT_DIGIT_RATIO",
                    mode,
                    candidate,
                    digit_ratio,
                )
                normalized = None
                panel = None
                continue

            global_x = (
                SEARCH_ROI[0]
                + APPROX_TARGET_ROI_LOCAL[0]
                + candidate[0]
            )
            global_y = (
                SEARCH_ROI[1]
                + APPROX_TARGET_ROI_LOCAL[1]
                + candidate[1]
            )
            global_rect = (
                global_x,
                global_y,
                candidate[2],
                candidate[3],
            )
            global_corners = (
                (global_x, global_y),
                (global_x + candidate[2] - 1, global_y),
                (
                    global_x + candidate[2] - 1,
                    global_y + candidate[3] - 1,
                ),
                (global_x, global_y + candidate[3] - 1),
            )
            print(
                "PANEL_FALLBACK_ACCEPT",
                mode,
                "RECT",
                global_rect,
                "MEAN",
                panel_mean,
                "WHITE_THRESHOLD",
                white_threshold,
                "DIGIT_THRESHOLD",
                threshold,
                "DIGIT_RATIO",
                digit_ratio,
                "DIGIT_BLOB",
                blob_rect,
            )
            info = (
                global_rect,
                global_corners,
                panel_mean,
                digit_ratio,
                blob_rect,
                mode,
            )
            return normalized, info

        return None, None
    except Exception as error:
        print_error("PANEL_FALLBACK_FAIL", error)
        return None, None
    finally:
        approximate = None
        white_mask = None
        panel = None
        gc.collect()


def find_best_panel(snapshot):
    search_gray = None
    best_canvas = None
    best_info = None
    best_rank = -1000000
    try:
        search_gray = snapshot.copy(roi=SEARCH_ROI)
        search_gray = search_gray.to_grayscale()
        rects = search_gray.find_rects(threshold=RECT_THRESHOLD)
        print("RECT_CANDIDATE_COUNT", len(rects))

        search_center_x = SEARCH_ROI[2] // 2
        search_center_y = SEARCH_ROI[3] // 2

        for rect_index, rect in enumerate(rects):
            rect_box = rect.rect()
            if (
                rect_box[2] < MIN_PANEL_WIDTH
                or rect_box[3] < MIN_PANEL_HEIGHT
            ):
                print("PANEL_REJECT_SIZE", rect_index, rect_box)
                continue

            panel = None
            normalized = None
            try:
                panel = rectify_panel(search_gray, rect)
                if panel is None:
                    continue
                panel_mean = panel.get_statistics()[0]
                if panel_mean < MIN_PANEL_MEAN:
                    print(
                        "PANEL_REJECT_MEAN",
                        rect_index,
                        rect_box,
                        panel_mean,
                    )
                    continue

                normalized, threshold, digit_ratio, blob_rect = (
                    extract_normalized_digit(panel)
                )
                if normalized is None:
                    print("PANEL_REJECT_NO_DIGIT", rect_index, rect_box)
                    continue
                if (
                    digit_ratio < MIN_DIGIT_RATIO
                    or digit_ratio > MAX_DIGIT_RATIO
                ):
                    print(
                        "PANEL_REJECT_DIGIT_RATIO",
                        rect_index,
                        rect_box,
                        digit_ratio,
                    )
                    continue

                center_x = rect_box[0] + (rect_box[2] // 2)
                center_y = rect_box[1] + (rect_box[3] // 2)
                center_distance = (
                    abs(center_x - search_center_x)
                    + abs(center_y - search_center_y)
                )
                area = rect_box[2] * rect_box[3]
                rank = area + (rect.magnitude() // 10) - (20 * center_distance)

                print(
                    "PANEL_CANDIDATE",
                    rect_index,
                    "RECT",
                    rect_box,
                    "MEAN",
                    panel_mean,
                    "THRESHOLD",
                    threshold,
                    "DIGIT_RATIO",
                    digit_ratio,
                    "RANK",
                    rank,
                )

                if rank > best_rank:
                    best_rank = rank
                    best_canvas = normalized
                    normalized = None
                    global_corners = []
                    for corner in rect.corners():
                        global_corners.append(
                            (
                                corner[0] + SEARCH_ROI[0],
                                corner[1] + SEARCH_ROI[1],
                            )
                        )
                    best_info = (
                        rect_box,
                        global_corners,
                        panel_mean,
                        digit_ratio,
                        blob_rect,
                        "RECT",
                    )
            finally:
                panel = None
                normalized = None
                gc.collect()

        if best_canvas is not None:
            return best_canvas, best_info

        print("PANEL_FALLBACK_START")
        return find_white_panel_fallback(search_gray)
    finally:
        search_gray = None
        gc.collect()


def classify_digit(normalized_digit, templates):
    best_digit = -1
    best_score = -2.0
    second_score = -2.0
    scores = []

    for digit in range(TEMPLATE_COUNT):
        score = difference_score(normalized_digit, templates[digit])
        scores.append(score)
        if score > best_score:
            second_score = best_score
            best_score = score
            best_digit = digit
        elif score > second_score:
            second_score = score

    margin = best_score - second_score
    accepted = (
        best_digit >= 0
        and best_score >= TEMPLATE_SCORE_THRESHOLD
        and margin >= MIN_SCORE_MARGIN
    )
    return best_digit, best_score, second_score, margin, accepted, scores


def draw_panel_outline(snapshot, corners):
    for index in range(4):
        start = corners[index]
        end = corners[(index + 1) % 4]
        snapshot.draw_line(
            (start[0], start[1], end[0], end[1]),
            color=(255, 0, 0),
            thickness=2,
        )


def run_frame(frame_id, templates):
    start_ms = time.ticks_ms()
    snapshot = sensor.snapshot()
    normalized_digit = None

    try:
        normalized_digit, panel_info = find_best_panel(snapshot)
        elapsed_ms = time.ticks_diff(time.ticks_ms(), start_ms)
        if normalized_digit is None or panel_info is None:
            snapshot.draw_rectangle(SEARCH_ROI, color=(255, 255, 255))
            print(
                "FRAME_RESULT",
                frame_id,
                "PANEL_NOT_FOUND",
                "ACCEPTED",
                False,
                "ELAPSED_MS",
                elapsed_ms,
            )
            return False, False, elapsed_ms

        snapshot.draw_rectangle(SEARCH_ROI, color=(255, 255, 255))
        draw_panel_outline(snapshot, panel_info[1])
        (
            best_digit,
            best_score,
            second_score,
            margin,
            accepted,
            scores,
        ) = classify_digit(normalized_digit, templates)
        correct = accepted and best_digit == EXPECTED_DIGIT
        elapsed_ms = time.ticks_diff(time.ticks_ms(), start_ms)

        print("FRAME_DIGIT_SCORES", frame_id, scores)
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
            "PANEL_MEAN",
            panel_info[2],
            "DIGIT_RATIO",
            panel_info[3],
            "DIGIT_BLOB",
            panel_info[4],
            "PANEL_MODE",
            panel_info[5],
            "ACCEPTED",
            accepted,
            "CORRECT",
            correct,
            "ELAPSED_MS",
            elapsed_ms,
        )
        return accepted, correct, elapsed_ms
    finally:
        normalized_digit = None
        snapshot = None
        gc.collect()


def run_result(result_id, templates):
    result_start_ms = time.ticks_ms()

    for attempt in range(1, MAX_TEST_FRAMES_PER_RESULT + 1):
        frame_id = "%d_%d" % (result_id, attempt)
        print("RESULT_ATTEMPT", result_id, attempt)
        try:
            accepted, correct, frame_elapsed_ms = run_frame(
                frame_id,
                templates,
            )
        except Exception as error:
            print_error("RESULT_ATTEMPT_FAIL_" + frame_id, error)
            accepted = False
            correct = False
            frame_elapsed_ms = 0
            gc.collect()

        if accepted:
            result_elapsed_ms = time.ticks_diff(
                time.ticks_ms(),
                result_start_ms,
            )
            print(
                "RESULT_FINAL",
                result_id,
                "ATTEMPTS",
                attempt,
                "ACCEPTED",
                True,
                "CORRECT",
                correct,
                "LAST_FRAME_MS",
                frame_elapsed_ms,
                "ELAPSED_MS",
                result_elapsed_ms,
            )
            return True, correct, result_elapsed_ms, attempt

    result_elapsed_ms = time.ticks_diff(time.ticks_ms(), result_start_ms)
    print(
        "RESULT_FINAL",
        result_id,
        "ATTEMPTS",
        MAX_TEST_FRAMES_PER_RESULT,
        "ACCEPTED",
        False,
        "CORRECT",
        False,
        "ELAPSED_MS",
        result_elapsed_ms,
    )
    return False, False, result_elapsed_ms, MAX_TEST_FRAMES_PER_RESULT


print("DIGIT_PERSPECTIVE_TEST_V1")
print("SEARCH_ROI", SEARCH_ROI)
print("EXPECTED_DIGIT", EXPECTED_DIGIT)
print("TEST_FRAME_COUNT", TEST_FRAME_COUNT)
print("MAX_TEST_FRAMES_PER_RESULT", MAX_TEST_FRAMES_PER_RESULT)
print("TEMPLATE_SCORE_THRESHOLD", TEMPLATE_SCORE_THRESHOLD)
print("MIN_SCORE_MARGIN", MIN_SCORE_MARGIN)

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

templates = None
if camera_ready:
    templates = load_normalized_templates()

correct_count = 0
wrong_accepted_count = 0
rejected_count = 0
completed_frames = 0
elapsed_sum_ms = 0
max_elapsed_ms = 0
total_captured_frames = 0

if camera_ready and templates is not None:
    for frame_id in range(1, TEST_FRAME_COUNT + 1):
        try:
            accepted, correct, elapsed_ms, captured_frames = run_result(
                frame_id,
                templates,
            )
            completed_frames += 1
            total_captured_frames += captured_frames
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
print("TOTAL_CAPTURED_FRAMES", total_captured_frames)
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
print("DIGIT_PERSPECTIVE_TEST_DONE")

while True:
    time.sleep_ms(1000)
