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
DEBUG_DRAW_GRID = False
DEBUG_PRINT_STATUS = False
car_debug = True
DEBUG_PRINT_EVERY_N_FRAMES = 10
DEBUG_PRINT_GRID = False
ENABLE_UART = True
UART_ID = 12
UART_BAUD = 115200
MAP_SEND_EVERY_N_FRAMES = 3
POSE_VERSION = 1
POSE_FLAG_VALID = 0x01
POSE_INVALID_COORD = 0xFFFF
POSE_HEAD = (0xA4, 0xB4)
POSE_TAIL = 0xC4
CAR_BODY_AMBIGUITY_RATIO = 0.90
CAR_BODY_MIN_CELL_SIZE = 0.50
CAR_BODY_MAX_CELL_SIZE = 1.30
FIXED_EXPOSURE_US = 200
CAMERA_BRIGHTNESS = -2
RAW_THRESHOLDS = {
    "box": (0, 100, -53, 127, 127, 49),
    "goal": (100, 0, 80, 127, 127, -128),
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
            roi_w = clamp(inner_size, 1, FRAME_W - x0)
            roi_h = clamp(inner_size, 1, FRAME_H - y0)
            roi = (x0, y0, roi_w, roi_h)
            blobs = img.find_blobs(
                GRID_THRESHOLDS,
                roi=roi,
                pixels_threshold=GRID_MIN_PIXELS_THRESHOLD,
                area_threshold=GRID_MIN_AREA_THRESHOLD,
                merge=False,
            )
            grid[row][col] = classify_grid_blobs(blobs)
    return grid
def find_car_body_candidates(img, map_roi):
    candidates = []
    cell_w = map_roi[2] / VISIBLE_COLS
    cell_h = map_roi[3] / VISIBLE_ROWS
    min_w = cell_w * CAR_BODY_MIN_CELL_SIZE
    min_h = cell_h * CAR_BODY_MIN_CELL_SIZE
    max_w = cell_w * CAR_BODY_MAX_CELL_SIZE
    max_h = cell_h * CAR_BODY_MAX_CELL_SIZE
    blobs = img.find_blobs(
        [THRESHOLDS["car_body"]],
        roi=inner_map_roi(map_roi),
        pixels_threshold=CAR_PIXELS_THRESHOLD,
        area_threshold=CAR_AREA_THRESHOLD,
        merge=True,
        margin=0,
    )
    for blob in blobs:
        if (blob.w() >= min_w and blob.w() <= max_w and
            blob.h() >= min_h and blob.h() <= max_h):
            candidates.append(blob)
    return (blobs, candidates)
def find_car_pose(img, map_roi, homography, debug=False):
    blobs, candidates = find_car_body_candidates(img, map_roi)
    best = None
    best_score = -1
    second_score = -1
    for blob in candidates:
        score = blob.pixels()
        if score > best_score:
            second_score = best_score
            best_score = score
            best = blob
        elif score > second_score:
            second_score = score
    if best is None:
        if debug:
            reason = "no_blob" if len(blobs) == 0 else "size_rejected"
            print("car blobs=%d accepted=%d best_pixels=0 second_pixels=0 reason=%s" %
                  (len(blobs), len(candidates), reason))
        return None
    if second_score >= 0 and second_score >= best_score * CAR_BODY_AMBIGUITY_RATIO:
        if debug:
            print("car blobs=%d accepted=%d best_pixels=%d second_pixels=%d reason=ambiguous" %
                  (len(blobs), len(candidates), best_score, second_score))
        return None
    center_px = best.x() + best.w() * 0.5
    center_py = best.y() + best.h() * 0.5
    center_grid = pixel_to_grid(center_px, center_py, homography)
    if (center_grid is None or
        center_grid[0] < 0.5 or center_grid[0] > GRID_COLS - 1.5 or
        center_grid[1] < 0.5 or center_grid[1] > GRID_ROWS - 1.5):
        if debug:
            print("car blobs=%d accepted=%d best_pixels=%d second_pixels=%d reason=out_of_map" %
                  (len(blobs), len(candidates), best_score, second_score))
        return None
    if second_score < 0:
        confidence = 100
    else:
        confidence = clamp(int(100 * (best_score - second_score) / best_score), 0, 100)
    if debug:
        print("car blobs=%d accepted=%d best_pixels=%d second_pixels=%d" %
              (len(blobs), len(candidates), best_score, second_score))
        print("car pose=(%.2f,%.2f) confidence=%d" %
              (center_grid[0], center_grid[1], confidence))
    return (center_grid[0], center_grid[1], confidence, best)
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
