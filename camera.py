import sensor
import time
from machine import UART
GRID_COLS = 16
GRID_ROWS = 12
FRAME_W = 320
FRAME_H = 240
MAP_ROI = (36, 21, 240, 174)
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
car_debug = False
DEBUG_PRINT_EVERY_N_FRAMES = 10
DEBUG_PRINT_GRID = True
ENABLE_UART = True
UART_ID = 12
UART_BAUD = 115200
FIXED_EXPOSURE_US = 326
CAMERA_BRIGHTNESS = -2
RAW_THRESHOLDS = {
	"box": (47, 100, -44, 127, 127, -8),
	"goal": (100, 0, 80, 127, 127, -128),
	"car_head": (0, 100, -6, -128, -128, -15),
	"car_tail": (42, 47, 127, -43, -9, 127),
	"road": (0, 51, 42, 127, -128, 127),
	"bomb": (0, 100, 27, 127, 127, -39),
}
ROAD_PIXELS_THRESHOLD = 19
ROAD_AREA_THRESHOLD = 19
GOAL_PIXELS_THRESHOLD = 8
GOAL_AREA_THRESHOLD = 7
BOX_PIXELS_THRESHOLD = 8
BOX_AREA_THRESHOLD = 7
CAR_PIXELS_THRESHOLD = 5
CAR_AREA_THRESHOLD = 4
BOMB_PIXELS_THRESHOLD = 14
BOMB_AREA_THRESHOLD = 7
def normalize_lab_threshold(threshold):
	l0, l1, a0, a1, b0, b1 = threshold
	if l0 > l1: l0, l1 = l1, l0
	if a0 > a1: a0, a1 = a1, a0
	if b0 > b1: b0, b1 = b1, b0
	return (l0, l1, a0, a1, b0, b1)
THRESHOLDS = {}
for key in RAW_THRESHOLDS:
	THRESHOLDS[key] = normalize_lab_threshold(RAW_THRESHOLDS[key])
CAR_THRESHOLDS = [THRESHOLDS["car_head"], THRESHOLDS["car_tail"]]
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
def create_grid(default_state):
	return [[default_state for _ in range(GRID_COLS)] for _ in range(GRID_ROWS)]
def inner_map_roi(map_roi):
	rx, ry, rw, rh = map_roi
	x0 = int(rx + rw / GRID_COLS + 0.5)
	y0 = int(ry + rh / GRID_ROWS + 0.5)
	x1 = int(rx + rw * (GRID_COLS - 1) / GRID_COLS + 0.5)
	y1 = int(ry + rh * (GRID_ROWS - 1) / GRID_ROWS + 0.5)
	return (x0, y0, x1 - x0, y1 - y0)
def draw_grid(img, map_roi):
	if not DEBUG_DRAW_GRID:
		return
	rx, ry, rw, rh = map_roi
	cell_w = rw / GRID_COLS
	cell_h = rh / GRID_ROWS
	inner_size = int(min(cell_w, cell_h) * INNER_SIZE_RATIO)
	if inner_size < 2:
		inner_size = 2
	half = inner_size // 2
	for row in range(1, GRID_ROWS - 1):
		for col in range(1, GRID_COLS - 1):
			cx = int(rx + (col + 0.5) * cell_w)
			cy = int(ry + (row + 0.5) * cell_h)
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
	cell_w = rw / GRID_COLS
	cell_h = rh / GRID_ROWS
	inner_size = int(min(cell_w, cell_h) * INNER_SIZE_RATIO)
	if inner_size < 2:
		inner_size = 2
	half = inner_size // 2
	for row in range(1, GRID_ROWS - 1):
		for col in range(1, GRID_COLS - 1):
			cx = int(rx + (col + 0.5) * cell_w)
			cy = int(ry + (row + 0.5) * cell_h)
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
def find_largest_blob(blobs):
	best_blob = None
	best_pixels = -1
	for blob in blobs:
		pixels = blob.pixels()
		if pixels > best_pixels:
			best_pixels = pixels
			best_blob = blob
	return best_blob
def find_car_blob(img, map_roi):
	cars = img.find_blobs(
		CAR_THRESHOLDS,
		roi=inner_map_roi(map_roi),
		pixels_threshold=CAR_PIXELS_THRESHOLD,
		area_threshold=CAR_AREA_THRESHOLD,
		merge=True,
	)
	return find_largest_blob(cars)
def car_cell_from_blob(car_blob, map_roi):
	if car_blob is None:
		return None
	rx, ry, rw, rh = map_roi
	cx = car_blob.cx()
	cy = car_blob.cy()
	if cx < rx or cx >= rx + rw or cy < ry or cy >= ry + rh:
		return None
	cell_w = rw / GRID_COLS
	cell_h = rh / GRID_ROWS
	col = clamp(int((cx - rx) / cell_w), 0, GRID_COLS - 1)
	row = clamp(int((cy - ry) / cell_h), 0, GRID_ROWS - 1)
	if (row == 0 or row == GRID_ROWS - 1 or
		col == 0 or col == GRID_COLS - 1):
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
def print_status(frame_id, fps, car_cell, exposure_us):
	if (not DEBUG_PRINT_STATUS or
		(frame_id % DEBUG_PRINT_EVERY_N_FRAMES) != 0):
		return
	print("frame=%d fps=%.2f car=%s exp=%d" %
		  (frame_id, fps, str(car_cell), int(exposure_us)))
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=300)
try:
	sensor.set_brightness(CAMERA_BRIGHTNESS)
except Exception:
	pass
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(False, exposure_us=FIXED_EXPOSURE_US)
sensor.skip_frames(time=500)
clock = time.clock()
uart = None
if ENABLE_UART:
	uart = UART(UART_ID, baudrate=UART_BAUD)
frame_id = 0
runtime_exposure_us = sensor.get_exposure_us()
active_map_roi = inner_map_roi(MAP_ROI)
while True:
	clock.tick()
	frame_id += 1
	img = sensor.snapshot()
	img.draw_rectangle(active_map_roi, color=(0,255,0))
	if car_debug == True:
		blobs = img.find_blobs(CAR_THRESHOLDS,roi=active_map_roi,area_threshold=5, pixels_threshold=5, merge=True)
		if blobs:
			for blob in blobs:
				print(blob.cx()," ",blob.cy())
	car_blob = find_car_blob(img, MAP_ROI)
	car_cell = car_cell_from_blob(car_blob, MAP_ROI)
	grid = build_grid(img, MAP_ROI)
	if car_cell is not None:
		row, col = car_cell
		if grid[row][col] in (STATE_WALL, STATE_ROAD):
			grid[row][col] = STATE_ROAD
	print_status(frame_id, clock.fps(), car_cell, runtime_exposure_us)
	if ENABLE_UART and uart is not None:
		send_uart(uart, grid, car_cell)
	draw_grid(img, MAP_ROI)
	if DEBUG_PRINT_GRID:
		for row in grid:
			print(row)
		print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
