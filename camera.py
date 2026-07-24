import sensor
import time
from machine import UART
GRID_COLS = 16
GRID_ROWS = 12
VISIBLE_COLS = GRID_COLS - 2
VISIBLE_ROWS = GRID_ROWS - 2
FRAME_W = 320
FRAME_H = 240
MAP_ROI = (47, 50, 210, 148)
MAP_CORNERS = ((47, 50), (257, 50), (257, 195), (47, 195))
INNER_SIZE_RATIO = 0.61
STATE_ROAD = 0
STATE_WALL = 1
STATE_GOAL = 2
STATE_BOX = 3
STATE_CAR = 4
STATE_BOMB = 5
STATE_PRIORITY = {
    STATE_WALL: 1,
    STATE_ROAD: 2,
    STATE_GOAL: 3,
    STATE_BOX: 4,
    STATE_CAR: 6,
    STATE_BOMB: 5,
}
STATE_NAMES = {
    STATE_GOAL: "goal",
    STATE_BOX: "box",
    STATE_BOMB: "bomb",
}
DEBUG_DRAW_GRID = False
DEBUG_PRINT_STATUS = False
car_debug = True
DEBUG_PRINT_EVERY_N_FRAMES = 10
DEBUG_PRINT_GRID = False
DEBUG_DRAW_CAR = True
DEBUG_PRINT_BODY_LAB = True
ENABLE_UART = True
UART_ID = 12
UART_BAUD = 115200
MAP_SEND_EVERY_N_FRAMES = 3
POSE_VERSION = 1
POSE_FLAG_VALID = 0x01
POSE_INVALID_COORD = 0xFFFF
POSE_HEAD = (0xA4, 0xB4)
POSE_TAIL = 0xC4
CAR_PAIR_MIN_DISTANCE = 0.10
CAR_PAIR_MAX_DISTANCE = 1.20
CAR_PAIR_AMBIGUITY_RATIO = 0.90
CAR_PAIR_MIN_CONFIDENCE = 60
CAR_MARKER_MAX_SPAN_CELL = 1.10
CAR_TRACK_MAX_JUMP_CELL = 1.50
CAR_TRACK_LOST_FRAMES = 15
CAR_BODY_MIN_CELL_SIZE = 0.20
CAR_BODY_MAX_CELL_SIZE = 1.30
CAR_FALLBACK_TRACK_MAX_JUMP_CELL = 0.75
CAR_FALLBACK_START_CONFIRM_FRAMES = 3
CAR_FALLBACK_TRACK_CONFIRM_FRAMES = 2
CAR_FALLBACK_CONFIDENCE = 65
FIXED_EXPOSURE_US = 200
CAMERA_BRIGHTNESS = -2
RAW_THRESHOLDS = {
    "box": (0, 100, -53, 127, 127, 49),
    "goal": (100, 0, 80, 127, 127, -128),
    "car_head": (70, 100, -60, -20, -45, -5),
    "car_tail": (70, 100, -95, -54, 28, 85),
    "car_body": (0, 100, -44, -128, -128, 84),
    "road": (0, 100, -128, 80, -128, -57),
    "bomb": (0, 100, 26, 127, 127, -38),
}
ROAD_PIXELS_THRESHOLD = 23
ROAD_AREA_THRESHOLD = 23
GOAL_PIXELS_THRESHOLD = 5
GOAL_AREA_THRESHOLD = 4
BOX_PIXELS_THRESHOLD = 5
BOX_AREA_THRESHOLD = 4
CAR_PIXELS_THRESHOLD = 15
CAR_AREA_THRESHOLD = 15
CAR_MARKER_PIXELS_THRESHOLD = 10
CAR_MARKER_AREA_THRESHOLD = 10
BOMB_PIXELS_THRESHOLD = 10
BOMB_AREA_THRESHOLD = 5
def normalize_lab_threshold(threshold):
    l0, l1, a0, a1, b0, b1 = threshold
    if l0 > l1: l0, l1 = l1, l0
    if a0 > a1: a0, a1 = a1, a0
    if b0 > b1: b0, b1 = b1, b0
    return (l0, l1, a0, a1, b0, b1)
THRESHOLDS = {}
for key in RAW_THRESHOLDS:
    THRESHOLDS[key] = normalize_lab_threshold(RAW_THRESHOLDS[key])
GRID_THRESHOLD_RULES = (
    (STATE_BOMB, THRESHOLDS["bomb"], BOMB_PIXELS_THRESHOLD, BOMB_AREA_THRESHOLD),
    (STATE_BOX, THRESHOLDS["box"], BOX_PIXELS_THRESHOLD, BOX_AREA_THRESHOLD),
    (STATE_GOAL, THRESHOLDS["goal"], GOAL_PIXELS_THRESHOLD, GOAL_AREA_THRESHOLD),
    (STATE_ROAD, THRESHOLDS["road"], ROAD_PIXELS_THRESHOLD, ROAD_AREA_THRESHOLD),
)
GRID_THRESHOLDS = [rule[1] for rule in GRID_THRESHOLD_RULES]
GRID_MIN_PIXELS_THRESHOLD = min([rule[2] for rule in GRID_THRESHOLD_RULES])
GRID_MIN_AREA_THRESHOLD = min([rule[3] for rule in GRID_THRESHOLD_RULES])
car_last_valid_pose = None
car_frames_since_valid = 0
car_fallback_pending_cell = None
car_fallback_pending_count = 0
car_debug_rectangles = []
car_debug_lines = []
def clamp(value, min_value, max_value):
    if value < min_value:
        return min_value
    if value > max_value:
        return max_value
    return value
def solve_linear_system(matrix, values):
    size = len(values)
    augmented = []
    for row in range(size):
        augmented.append(list(matrix[row]) + [values[row]])
    for column in range(size):
        pivot = column
        for row in range(column + 1, size):
            if abs(augmented[row][column]) > abs(augmented[pivot][column]):
                pivot = row
        if abs(augmented[pivot][column]) < 0.000001:
            raise ValueError("map corners do not define a homography")
        if pivot != column:
            augmented[column], augmented[pivot] = augmented[pivot], augmented[column]
        divisor = augmented[column][column]
        for index in range(column, size + 1):
            augmented[column][index] /= divisor
        for row in range(size):
            if row == column:
                continue
            factor = augmented[row][column]
            if factor == 0:
                continue
            for index in range(column, size + 1):
                augmented[row][index] -= factor * augmented[column][index]
    return [augmented[row][size] for row in range(size)]
def build_map_homography(corners):
    target = (
        (0.5, 0.5),
        (GRID_COLS - 1.5, 0.5),
        (GRID_COLS - 1.5, GRID_ROWS - 1.5),
        (0.5, GRID_ROWS - 1.5),
    )
    matrix = []
    values = []
    for index in range(4):
        px, py = corners[index]
        gx, gy = target[index]
        matrix.append([px, py, 1.0, 0.0, 0.0, 0.0, -gx * px, -gx * py])
        values.append(gx)
        matrix.append([0.0, 0.0, 0.0, px, py, 1.0, -gy * px, -gy * py])
        values.append(gy)
    return solve_linear_system(matrix, values)
def pixel_to_grid(px, py, homography):
    denominator = homography[6] * px + homography[7] * py + 1.0
    if abs(denominator) < 0.000001:
        return None
    grid_x = (homography[0] * px + homography[1] * py + homography[2]) / denominator
    grid_y = (homography[3] * px + homography[4] * py + homography[5]) / denominator
    return (grid_x, grid_y)
def create_grid(default_state):
    return [[default_state for _ in range(GRID_COLS)] for _ in range(GRID_ROWS)]
def inner_map_roi(map_roi):
    return map_roi
def grid_cell_roi(map_roi, row, col):
    rx, ry, rw, rh = map_roi
    cell_w = rw / VISIBLE_COLS
    cell_h = rh / VISIBLE_ROWS
    inner_size = int(min(cell_w, cell_h) * INNER_SIZE_RATIO)
    if inner_size < 2:
        inner_size = 2
    half = inner_size // 2
    cx = int(rx + (col - 0.5) * cell_w)
    cy = int(ry + (row - 0.5) * cell_h)
    x0 = clamp(cx - half, 0, FRAME_W - 1)
    y0 = clamp(cy - half, 0, FRAME_H - 1)
    roi_w = clamp(inner_size, 1, FRAME_W - x0)
    roi_h = clamp(inner_size, 1, FRAME_H - y0)
    return (x0, y0, roi_w, roi_h)
def draw_grid(img, map_roi):
    if not DEBUG_DRAW_GRID:
        return
    rx, ry, rw, rh = map_roi
    cell_w = rw / VISIBLE_COLS
    cell_h = rh / VISIBLE_ROWS
    inner_size = int(min(cell_w, cell_h) * INNER_SIZE_RATIO)
    if inner_size < 2:
        inner_size = 2
    half = inner_size // 2
    for row in range(1, GRID_ROWS - 1):
        for col in range(1, GRID_COLS - 1):
            cx = int(rx + (col - 0.5) * cell_w)
            cy = int(ry + (row - 0.5) * cell_h)
            x0 = clamp(cx - half, 0, FRAME_W - 1)
            y0 = clamp(cy - half, 0, FRAME_H - 1)
            x1 = clamp(cx + half - 1, 0, FRAME_W - 1)
            y1 = clamp(cy + half - 1, 0, FRAME_H - 1)
            img.draw_rectangle((x0, y0, x1 - x0 + 1, y1 - y0 + 1),
                               color=(200, 200, 200))
def classify_grid_blobs(blobs):
    state = STATE_WALL
    for blob in blobs:
        code = blob.code()
        pixels = blob.pixels()
        area = blob.area()
        for index in range(len(GRID_THRESHOLD_RULES)):
            candidate, _, pixels_threshold, area_threshold = GRID_THRESHOLD_RULES[index]
            if ((code & (1 << index)) and
                pixels >= pixels_threshold and area >= area_threshold and
                STATE_PRIORITY[candidate] > STATE_PRIORITY[state]):
                state = candidate
    return state
def build_grid(img, map_roi):
    grid = create_grid(STATE_WALL)
    for row in range(1, GRID_ROWS - 1):
        for col in range(1, GRID_COLS - 1):
            roi = grid_cell_roi(map_roi, row, col)
            blobs = img.find_blobs(
                GRID_THRESHOLDS,
                roi=roi,
                pixels_threshold=GRID_MIN_PIXELS_THRESHOLD,
                area_threshold=GRID_MIN_AREA_THRESHOLD,
                merge=False,
            )
            grid[row][col] = classify_grid_blobs(blobs)
    return grid
def center_grid_in_map(center_grid):
    return (center_grid is not None and
            center_grid[0] >= 0.5 and center_grid[0] <= GRID_COLS - 1.5 and
            center_grid[1] >= 0.5 and center_grid[1] <= GRID_ROWS - 1.5)
def blob_grid_center(blob, homography):
    return pixel_to_grid(blob.x() + blob.w() * 0.5,
                         blob.y() + blob.h() * 0.5,
                         homography)
def marker_grid_center(blob, homography):
    return pixel_to_grid(blob.cx(), blob.cy(), homography)
def print_car_blob(name, index, blob, center_grid, cell_w, cell_h, result):
    if center_grid is None:
        grid_text = "none"
    else:
        grid_text = "(%.2f,%.2f)" % (center_grid[0], center_grid[1])
    print("%s[%d] rect=%s ratio=(%.2f,%.2f) pixels=%d area=%d grid=%s result=%s" %
          (name, index, str(blob.rect()), blob.w() / cell_w, blob.h() / cell_h,
           blob.pixels(), blob.area(), grid_text, result))
def statistics_field(statistics, name):
    value = getattr(statistics, name)
    if callable(value):
        return value()
    return value
def print_body_lab_grid(img, index, blob):
    if not DEBUG_PRINT_BODY_LAB or blob.w() < 3 or blob.h() < 3:
        return
    cells = []
    for row in range(3):
        row_cells = []
        y0 = blob.y() + (blob.h() * row) // 3
        y1 = blob.y() + (blob.h() * (row + 1)) // 3
        for col in range(3):
            x0 = blob.x() + (blob.w() * col) // 3
            x1 = blob.x() + (blob.w() * (col + 1)) // 3
            statistics = img.get_statistics(
                roi=(x0, y0, max(1, x1 - x0), max(1, y1 - y0)))
            row_cells.append("(%d,%d,%d)" % (
                statistics_field(statistics, "l_mean"),
                statistics_field(statistics, "a_mean"),
                statistics_field(statistics, "b_mean"),
            ))
        cells.append(",".join(row_cells))
    print("body_lab[%d] 3x3=%s" % (index, "/".join(cells)))
def reset_car_debug_overlay():
    global car_debug_rectangles
    global car_debug_lines
    car_debug_rectangles = []
    car_debug_lines = []
def draw_car_debug_overlay(img):
    if not DEBUG_DRAW_CAR:
        return
    for rect, color in car_debug_rectangles:
        img.draw_rectangle(rect, color=color)
    for line, color in car_debug_lines:
        img.draw_line(line, color=color)
def draw_blob(img, blob, color):
    if DEBUG_DRAW_CAR:
        car_debug_rectangles.append((blob.rect(), color))
def find_marker_candidates(img, threshold, name, color, map_roi,
                           homography, debug):
    candidates = []
    cell_w = map_roi[2] / VISIBLE_COLS
    cell_h = map_roi[3] / VISIBLE_ROWS
    max_span_w = cell_w * CAR_MARKER_MAX_SPAN_CELL
    max_span_h = cell_h * CAR_MARKER_MAX_SPAN_CELL
    blobs = img.find_blobs(
        [threshold],
        roi=inner_map_roi(map_roi),
        pixels_threshold=1,
        area_threshold=1,
        merge=False,
    )
    for index in range(len(blobs)):
        blob = blobs[index]
        center_grid = marker_grid_center(blob, homography)
        if blob.pixels() < CAR_MARKER_PIXELS_THRESHOLD:
            result = "pixels_insufficient"
        elif blob.area() < CAR_MARKER_AREA_THRESHOLD:
            result = "area_insufficient"
        elif blob.w() > max_span_w or blob.h() > max_span_h:
            result = "too_large"
        elif not center_grid_in_map(center_grid):
            result = "out_of_map"
        else:
            result = "accepted"
            candidates.append((blob, center_grid))
        if debug:
            print_car_blob(name, index, blob, center_grid,
                           cell_w, cell_h, result)
            draw_blob(img, blob, color)
    if debug and len(blobs) == 0:
        print("%s blobs=0 result=no_blob" % name)
    return candidates
def pair_continuity_distance_sq(center_x, center_y):
    if car_last_valid_pose is None:
        return 0.0
    dx = center_x - car_last_valid_pose[0]
    dy = center_y - car_last_valid_pose[1]
    return dx * dx + dy * dy
def draw_pair(img, head, tail):
    if not DEBUG_DRAW_CAR:
        return
    head_x = head.cx()
    head_y = head.cy()
    tail_x = tail.cx()
    tail_y = tail.cy()
    center_x = (head_x + tail_x) // 2
    center_y = (head_y + tail_y) // 2
    car_debug_lines.append(
        ((head_x, head_y, tail_x, tail_y), (255, 255, 255)))
    car_debug_rectangles.append(
        ((clamp(center_x - 2, 0, FRAME_W - 1),
          clamp(center_y - 2, 0, FRAME_H - 1), 5, 5),
         (255, 255, 255)))
def find_car_pair(img, map_roi, homography, debug):
    head_candidates = find_marker_candidates(
        img, THRESHOLDS["car_head"], "head", (255, 0, 0),
        map_roi, homography, debug)
    tail_candidates = find_marker_candidates(
        img, THRESHOLDS["car_tail"], "tail", (0, 0, 255),
        map_roi, homography, debug)
    min_distance_sq = CAR_PAIR_MIN_DISTANCE * CAR_PAIR_MIN_DISTANCE
    max_distance_sq = CAR_PAIR_MAX_DISTANCE * CAR_PAIR_MAX_DISTANCE
    max_jump_sq = CAR_TRACK_MAX_JUMP_CELL * CAR_TRACK_MAX_JUMP_CELL
    best = None
    best_score = -1.0
    second_score = -1.0
    pair_index = 0
    for head, head_grid in head_candidates:
        for tail, tail_grid in tail_candidates:
            dx = head_grid[0] - tail_grid[0]
            dy = head_grid[1] - tail_grid[1]
            distance_sq = dx * dx + dy * dy
            center_x = (head_grid[0] + tail_grid[0]) * 0.5
            center_y = (head_grid[1] + tail_grid[1]) * 0.5
            continuity_sq = pair_continuity_distance_sq(center_x, center_y)
            reason = "accepted"
            if distance_sq < min_distance_sq:
                reason = "pair_too_close"
            elif distance_sq > max_distance_sq:
                reason = "pair_too_far"
            elif not center_grid_in_map((center_x, center_y)):
                reason = "out_of_map"
            elif (car_last_valid_pose is not None and
                  car_frames_since_valid <= CAR_TRACK_LOST_FRAMES and
                  continuity_sq > max_jump_sq):
                reason = "track_jump"
            if reason == "accepted":
                support = head.pixels() + tail.pixels()
                score = support / (1.0 + continuity_sq)
                if score > best_score:
                    second_score = best_score
                    best_score = score
                    best = (center_x, center_y, head, tail, support)
                elif score > second_score:
                    second_score = score
            if debug:
                print("pair[%d] center=(%.2f,%.2f) distance=%.2f continuity=%.2f result=%s" %
                      (pair_index, center_x, center_y, distance_sq ** 0.5,
                       continuity_sq ** 0.5, reason))
            pair_index += 1
    if best is None:
        if debug:
            print("pair accepted=0 result=no_valid_pair")
        return None
    if (second_score >= 0.0 and
        second_score >= best_score * CAR_PAIR_AMBIGUITY_RATIO):
        if debug:
            print("pair result=ambiguous best=%.2f second=%.2f" %
                  (best_score, second_score))
        return None
    if second_score < 0.0:
        confidence = 100
    else:
        score_ratio = second_score / best_score
        confidence = clamp(
            int(100.0 - 40.0 * score_ratio / CAR_PAIR_AMBIGUITY_RATIO),
            CAR_PAIR_MIN_CONFIDENCE, 100)
    if debug:
        print("pair pose=(%.2f,%.2f) support=%d confidence=%d" %
              (best[0], best[1], best[4], confidence))
        draw_pair(img, best[2], best[3])
    return (best[0], best[1], confidence, "pair", best[2], best[3])
def candidate_object_state(img, center_grid, map_roi):
    col = int(center_grid[0] + 0.5)
    row = int(center_grid[1] + 0.5)
    if (row <= 0 or row >= GRID_ROWS - 1 or
        col <= 0 or col >= GRID_COLS - 1):
        return None
    blobs = img.find_blobs(
        GRID_THRESHOLDS,
        roi=grid_cell_roi(map_roi, row, col),
        pixels_threshold=GRID_MIN_PIXELS_THRESHOLD,
        area_threshold=GRID_MIN_AREA_THRESHOLD,
        merge=False,
    )
    return classify_grid_blobs(blobs)
def classify_car_body_blob(img, blob, center_grid, map_roi,
                           min_w, min_h, max_w, max_h):
    if blob.pixels() < CAR_PIXELS_THRESHOLD:
        return "pixels_insufficient"
    if blob.area() < CAR_AREA_THRESHOLD:
        return "area_insufficient"
    if blob.w() < min_w or blob.h() < min_h:
        return "too_small"
    if blob.w() > max_w or blob.h() > max_h:
        return "too_large"
    if not center_grid_in_map(center_grid):
        return "out_of_map"
    object_state = candidate_object_state(img, center_grid, map_roi)
    if object_state in (STATE_BOX, STATE_GOAL, STATE_BOMB):
        return "object_%s" % STATE_NAMES[object_state]
    return "accepted"
def find_car_body_candidates(img, map_roi, homography, debug):
    candidates = []
    cell_w = map_roi[2] / VISIBLE_COLS
    cell_h = map_roi[3] / VISIBLE_ROWS
    min_w = cell_w * CAR_BODY_MIN_CELL_SIZE
    min_h = cell_h * CAR_BODY_MIN_CELL_SIZE
    max_w = cell_w * CAR_BODY_MAX_CELL_SIZE
    max_h = cell_h * CAR_BODY_MAX_CELL_SIZE
    selection_blobs = img.find_blobs(
        [THRESHOLDS["car_body"]],
        roi=inner_map_roi(map_roi),
        pixels_threshold=CAR_PIXELS_THRESHOLD,
        area_threshold=CAR_AREA_THRESHOLD,
        merge=True,
        margin=0,
    )
    for blob in selection_blobs:
        center_grid = blob_grid_center(blob, homography)
        result = classify_car_body_blob(
            img, blob, center_grid, map_roi, min_w, min_h, max_w, max_h)
        if result == "accepted":
            candidates.append((blob, center_grid))
    if debug:
        for index in range(len(candidates)):
            print_body_lab_grid(img, index, candidates[index][0])
        diagnostic_blobs = img.find_blobs(
            [THRESHOLDS["car_body"]],
            roi=inner_map_roi(map_roi),
            pixels_threshold=1,
            area_threshold=1,
            merge=True,
            margin=0,
        )
        for index in range(len(diagnostic_blobs)):
            blob = diagnostic_blobs[index]
            center_grid = blob_grid_center(blob, homography)
            result = classify_car_body_blob(
                img, blob, center_grid, map_roi,
                min_w, min_h, max_w, max_h)
            print_car_blob("body", index, blob, center_grid,
                           cell_w, cell_h, result)
            draw_blob(img, blob, (255, 255, 0))
        if len(diagnostic_blobs) == 0:
            print("body blobs=0 result=no_blob")
    return candidates
def fallback_candidate_near_last(candidates):
    if car_last_valid_pose is None:
        return candidates
    max_jump_sq = (CAR_FALLBACK_TRACK_MAX_JUMP_CELL *
                   CAR_FALLBACK_TRACK_MAX_JUMP_CELL)
    near = []
    for candidate in candidates:
        center_grid = candidate[1]
        dx = center_grid[0] - car_last_valid_pose[0]
        dy = center_grid[1] - car_last_valid_pose[1]
        if dx * dx + dy * dy <= max_jump_sq:
            near.append(candidate)
    return near
def find_car_body_fallback(img, map_roi, homography, debug):
    global car_last_valid_pose
    global car_frames_since_valid
    global car_fallback_pending_cell
    global car_fallback_pending_count
    candidates = find_car_body_candidates(
        img, map_roi, homography, debug)
    tracked_candidates = fallback_candidate_near_last(candidates)
    if len(tracked_candidates) != 1:
        car_fallback_pending_cell = None
        car_fallback_pending_count = 0
        if debug:
            reason = "no_candidate" if len(tracked_candidates) == 0 else "ambiguous"
            print("fallback candidates=%d result=%s" %
                  (len(tracked_candidates), reason))
        return None
    blob, center_grid = tracked_candidates[0]
    cell = (int(center_grid[1] + 0.5), int(center_grid[0] + 0.5))
    if cell == car_fallback_pending_cell:
        car_fallback_pending_count += 1
    else:
        car_fallback_pending_cell = cell
        car_fallback_pending_count = 1
    if car_last_valid_pose is None:
        confirm_frames = CAR_FALLBACK_START_CONFIRM_FRAMES
    else:
        confirm_frames = CAR_FALLBACK_TRACK_CONFIRM_FRAMES
    if car_fallback_pending_count < confirm_frames:
        if debug:
            print("fallback cell=%s stable=%d/%d result=waiting" %
                  (str(cell), car_fallback_pending_count, confirm_frames))
        return None
    car_last_valid_pose = (center_grid[0], center_grid[1])
    car_frames_since_valid = 0
    car_fallback_pending_cell = cell
    car_fallback_pending_count = CAR_FALLBACK_TRACK_CONFIRM_FRAMES
    if debug:
        print("fallback pose=(%.2f,%.2f) confidence=%d result=accepted" %
              (center_grid[0], center_grid[1], CAR_FALLBACK_CONFIDENCE))
        draw_blob(img, blob, (255, 255, 255))
    return (center_grid[0], center_grid[1], CAR_FALLBACK_CONFIDENCE,
            "body_fallback", blob, None)
def find_car_pose(img, map_roi, homography, debug=False):
    global car_last_valid_pose
    global car_frames_since_valid
    global car_fallback_pending_cell
    global car_fallback_pending_count
    reset_car_debug_overlay()
    car_frames_since_valid += 1
    if (car_last_valid_pose is not None and
        car_frames_since_valid > CAR_TRACK_LOST_FRAMES):
        if debug:
            print("car track result=unlocked lost_frames=%d" %
                  car_frames_since_valid)
        car_last_valid_pose = None
        car_fallback_pending_cell = None
        car_fallback_pending_count = 0
    pair_pose = find_car_pair(img, map_roi, homography, debug)
    if pair_pose is not None:
        car_last_valid_pose = (pair_pose[0], pair_pose[1])
        car_frames_since_valid = 0
        car_fallback_pending_cell = None
        car_fallback_pending_count = 0
        if debug:
            find_car_body_candidates(img, map_roi, homography, True)
        return pair_pose
    return find_car_body_fallback(img, map_roi, homography, debug)
def car_cell_from_pose(car_pose):
    if car_pose is None:
        return None
    col = int(car_pose[0] + 0.5)
    row = int(car_pose[1] + 0.5)
    if row <= 0 or row >= GRID_ROWS - 1 or col <= 0 or col >= GRID_COLS - 1:
        return None
    return (row, col)
def flatten_grid(grid):
    out = []
    for row in grid:
        for value in row:
            out.append(value)
    return out
def encode_coord(value):
    if value is None:
        return 255
    return value
def send_uart(uart, grid, car_cell):
    car_row = None
    car_col = None
    if car_cell is not None:
        car_row, car_col = car_cell
    payload = bytearray(flatten_grid(grid) +
                        [encode_coord(car_col), encode_coord(car_row)])
    uart.write(bytearray([0xA3, 0xB3]))
    uart.write(payload)
    uart.write(bytearray([0xC3]))
def crc8_atm(data):
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc
def encode_pose_coord(value):
    encoded = int(value * 100.0 + 0.5)
    return clamp(encoded, 0, 0xFFFE)
def send_pose_uart(uart, frame_id, car_pose):
    valid = car_pose is not None
    if valid:
        x100 = encode_pose_coord(car_pose[0])
        y100 = encode_pose_coord(car_pose[1])
        confidence = car_pose[2]
        flags = POSE_FLAG_VALID
    else:
        x100 = POSE_INVALID_COORD
        y100 = POSE_INVALID_COORD
        confidence = 0
        flags = 0
    payload = bytearray([
        POSE_VERSION,
        frame_id & 0xFF,
        flags,
        x100 & 0xFF,
        (x100 >> 8) & 0xFF,
        y100 & 0xFF,
        (y100 >> 8) & 0xFF,
        confidence,
    ])
    uart.write(bytearray(POSE_HEAD))
    uart.write(payload)
    uart.write(bytearray([crc8_atm(payload), POSE_TAIL]))
def print_status(frame_id, fps, car_pose, exposure_us):
    if (not DEBUG_PRINT_STATUS or
        (frame_id % DEBUG_PRINT_EVERY_N_FRAMES) != 0):
        return
    print("frame=%d fps=%.2f pose=%s exp=%d" %
          (frame_id, fps, str(car_pose), int(exposure_us)))
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=300)
try:
    sensor.set_brightness(CAMERA_BRIGHTNESS)
except Exception:
    pass
sensor.skip_frames(time=1000)
sensor.set_auto_gain(False, gain_db=0)
sensor.set_auto_whitebal(False)
sensor.skip_frames(time=200)
sensor.set_auto_exposure(False, exposure_us=FIXED_EXPOSURE_US)
sensor.skip_frames(time=500)
clock = time.clock()
uart = None
if ENABLE_UART:
    uart = UART(UART_ID, baudrate=UART_BAUD)
frame_id = 0
runtime_exposure_us = sensor.get_exposure_us()
active_map_roi = inner_map_roi(MAP_ROI)
map_homography = build_map_homography(MAP_CORNERS)
while True:
    clock.tick()
    frame_id += 1
    img = sensor.snapshot()
    car_pose = find_car_pose(
        img,
        MAP_ROI,
        map_homography,
        car_debug and (frame_id % DEBUG_PRINT_EVERY_N_FRAMES) == 0,
    )
    car_cell = car_cell_from_pose(car_pose)
    print_status(frame_id, clock.fps(), car_pose, runtime_exposure_us)
    if ENABLE_UART and uart is not None:
        send_pose_uart(uart, frame_id, car_pose)
    if (frame_id % MAP_SEND_EVERY_N_FRAMES) == 0:
        grid = build_grid(img, MAP_ROI)
        if car_cell is not None:
            row, col = car_cell
            if grid[row][col] in (STATE_WALL, STATE_ROAD):
                grid[row][col] = STATE_ROAD
        if ENABLE_UART and uart is not None:
            send_uart(uart, grid, car_cell)
        if DEBUG_PRINT_GRID:
            for row in grid:
                print(row)
            print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
    img.draw_rectangle(active_map_roi, color=(0,255,0))
    draw_grid(img, MAP_ROI)
    draw_car_debug_overlay(img)
