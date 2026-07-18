import gc
import image
import sensor
import sys
import tf
import time
from machine import UART
MODEL1_PATH = "/sd/model1.tflite"
TEMPLATE_DIRECTORY = "/sd/templates"
TEMPLATE_COUNT = 10
TEMPLATE_SCORE_THRESHOLD = 0.83
MIN_SCORE_MARGIN = 0.05
ROI = (95, 70, 180, 165)
SEARCH_ROI = (68, 70, 210, 170)
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
MAX_DIGIT_ATTEMPTS = 5
RX_FRAME_LEN = 8
MODEL_OUTPUT_COUNT = 10
uart = None
rx_frame = bytearray(RX_FRAME_LEN)
rx_frame_index = 0
active_model = None
active_model_type = None
digit_templates = None
def print_error(stage, error):
	print(stage)
	try:
		sys.print_exception(error)
	except Exception:
		print(repr(error))
def parse_uart_byte(byte):
	global rx_frame_index
	if rx_frame_index == 0:
		if byte == 0xA5:
			rx_frame[0] = byte
			rx_frame_index = 1
		return None
	if rx_frame_index == 1:
		if byte == 0xB5:
			rx_frame[1] = byte
			rx_frame_index = 2
		elif byte == 0xA5:
			rx_frame[0] = byte
		else:
			rx_frame_index = 0
		return None
	rx_frame[rx_frame_index] = byte
	rx_frame_index += 1
	if rx_frame_index < RX_FRAME_LEN:
		return None
	object_type = rx_frame[2]
	object_index = rx_frame[3]
	object_row = rx_frame[4]
	object_col = rx_frame[5]
	check = rx_frame[6]
	expected_check = (object_type ^ object_index ^
					  object_row ^ object_col)
	if rx_frame[7] == 0xC5 and check == expected_check:
		rx_frame_index = 0
		return object_type, object_index, object_row, object_col
	if byte == 0xA5:
		rx_frame[0] = byte
		rx_frame_index = 1
	else:
		rx_frame_index = 0
	return None
def check_uart_cmd():
	global rx_frame_index
	try:
		if uart.any() <= 0:
			return None
		data = uart.read(1)
		if not data:
			return None
		return parse_uart_byte(data[0])
	except Exception as error:
		rx_frame_index = 0
		print_error("RX_FAIL", error)
		return None
def release_active_model():
	global active_model
	global active_model_type
	start_ms = time.ticks_ms()
	print("MODEL_RELEASE_START")
	had_active_model = active_model is not None
	active_model = None
	active_model_type = None
	try:
		if had_active_model:
			tf.free_from_fb()
	except Exception as error:
		print_error("MODEL_RELEASE_FAIL", error)
		gc.collect()
		return False
	gc.collect()
	print("MODEL_RELEASE_DONE", "ELAPSED_MS",
		  time.ticks_diff(time.ticks_ms(), start_ms))
	return True
def ensure_model(object_type):
	global active_model
	global active_model_type
	if object_type != 0:
		return False
	model_path = MODEL1_PATH
	if active_model is not None and active_model_type == object_type:
		return True
	if active_model is not None:
		release_active_model()
	try:
		active_model = tf.load(model_path, load_to_fb=True)
		active_model_type = object_type
		return True
	except Exception as error:
		active_model = None
		active_model_type = None
		gc.collect()
		print_error("MODEL_LOAD_FAIL", error)
		return False
def read_model_output():
	img = sensor.snapshot()
	output = None
	for obj in tf.classify(
			active_model,
			img,
			roi=ROI,
			min_scale=1.0,
			scale_mul=0.5,
			x_overlap=0.0,
			y_overlap=0.0):
		if output is None:
			output = list(obj.output())
	if output is None or len(output) != MODEL_OUTPUT_COUNT:
		return None
	return output
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
		if blob_width > ((width * 8) // 10) and blob_height < (height // 4):
			continue
		if blob_width < 3 or blob_height < 8:
			continue
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
			print("TEMPLATE_READY", digit,
				  "THRESHOLD", threshold,
				  "DIGIT_RATIO", digit_ratio,
				  "BLOB", blob_rect)
		except Exception as error:
			print_error("TEMPLATE_LOAD_FAIL " + path, error)
			return None
		finally:
			template = None
			normalized = None
			gc.collect()
	return templates
def ensure_digit_templates():
	global digit_templates
	if digit_templates is not None:
		return True
	print("DIGIT_TEMPLATES_LOAD_START")
	digit_templates = load_normalized_templates()
	if digit_templates is None:
		print("DIGIT_TEMPLATES_LOAD_FAIL")
		gc.collect()
		return False
	print("DIGIT_TEMPLATES_LOAD_DONE", len(digit_templates))
	return True
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
			("APPROX_ROI", (0, 0, approximate.width(), approximate.height()))
		)
		for mode, candidate in candidate_rois:
			panel = approximate.copy(roi=candidate)
			panel_mean = panel.get_statistics()[0]
			if panel_mean < MIN_PANEL_MEAN:
				print("PANEL_FALLBACK_REJECT_MEAN", mode,
					  candidate, panel_mean)
				panel = None
				continue
			normalized, threshold, digit_ratio, blob_rect = (
				extract_normalized_digit(panel)
			)
			if normalized is None:
				print("PANEL_FALLBACK_REJECT_NO_DIGIT", mode, candidate)
				panel = None
				continue
			if digit_ratio < MIN_DIGIT_RATIO or digit_ratio > MAX_DIGIT_RATIO:
				print("PANEL_FALLBACK_REJECT_DIGIT_RATIO", mode,
					  candidate, digit_ratio)
				normalized = None
				panel = None
				continue
			global_rect = (
				SEARCH_ROI[0] + APPROX_TARGET_ROI_LOCAL[0] + candidate[0],
				SEARCH_ROI[1] + APPROX_TARGET_ROI_LOCAL[1] + candidate[1],
				candidate[2],
				candidate[3],
			)
			print("PANEL_FALLBACK_ACCEPT", mode,
				  "RECT", global_rect,
				  "MEAN", panel_mean,
				  "WHITE_THRESHOLD", white_threshold,
				  "DIGIT_THRESHOLD", threshold,
				  "DIGIT_RATIO", digit_ratio,
				  "DIGIT_BLOB", blob_rect)
			info = (
				global_rect,
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
			if rect_box[2] < MIN_PANEL_WIDTH or rect_box[3] < MIN_PANEL_HEIGHT:
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
					print("PANEL_REJECT_MEAN", rect_index,
						  rect_box, panel_mean)
					continue
				normalized, threshold, digit_ratio, blob_rect = (
					extract_normalized_digit(panel)
				)
				if normalized is None:
					print("PANEL_REJECT_NO_DIGIT", rect_index, rect_box)
					continue
				if digit_ratio < MIN_DIGIT_RATIO or digit_ratio > MAX_DIGIT_RATIO:
					print("PANEL_REJECT_DIGIT_RATIO", rect_index,
						  rect_box, digit_ratio)
					continue
				center_x = rect_box[0] + (rect_box[2] // 2)
				center_y = rect_box[1] + (rect_box[3] // 2)
				center_distance = (
					abs(center_x - search_center_x)
					+ abs(center_y - search_center_y)
				)
				area = rect_box[2] * rect_box[3]
				rank = area + (rect.magnitude() // 10) - (20 * center_distance)
				print("PANEL_CANDIDATE", rect_index,
					  "RECT", rect_box,
					  "MEAN", panel_mean,
					  "THRESHOLD", threshold,
					  "DIGIT_RATIO", digit_ratio,
					  "RANK", rank)
				if rank > best_rank:
					best_rank = rank
					best_canvas = normalized
					normalized = None
					best_info = (
						rect_box,
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
def classify_digit(normalized_digit):
	best_digit = -1
	best_score = -2.0
	second_score = -2.0
	scores = []
	for digit in range(TEMPLATE_COUNT):
		score = difference_score(normalized_digit, digit_templates[digit])
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
def send_result(object_type, object_index, label):
	check = object_type ^ object_index ^ label
	frame = bytearray([
		0xA4, 0xB4,
		object_type,
		object_index,
		label,
		check,
		0xC4,
	])
	start_ms = time.ticks_ms()
	print("TX_START", "TYPE", object_type,
		  "INDEX", object_index, "LABEL", label)
	try:
		written = uart.write(frame)
		print("TX_DONE", "BYTES", written, "ELAPSED_MS",
			  time.ticks_diff(time.ticks_ms(), start_ms))
		return True
	except Exception as error:
		print_error("TX_FAIL", error)
		return False
def match_digit_and_reply(object_type, object_index):
	match_start_ms = time.ticks_ms()
	print("DIGIT_REQUEST_RECEIVED", "INDEX", object_index)
	try:
		if not release_active_model():
			return send_result(object_type, object_index, 0)
		if not ensure_digit_templates():
			return send_result(object_type, object_index, 0)
		best_digit = -1
		best_score = -2.0
		second_score = -2.0
		margin = 0.0
		accepted = False
		for attempt in range(1, MAX_DIGIT_ATTEMPTS + 1):
			img = None
			normalized_digit = None
			accepted = False
			attempt_start_ms = time.ticks_ms()
			print("DIGIT_ATTEMPT_START", attempt)
			try:
				img = sensor.snapshot()
				print("SNAPSHOT_DONE", "ATTEMPT", attempt,
					  "ELAPSED_MS",
					  time.ticks_diff(time.ticks_ms(), attempt_start_ms))
				normalized_digit, panel_info = find_best_panel(img)
				if normalized_digit is None or panel_info is None:
					print("DIGIT_ATTEMPT_REJECT", attempt, "PANEL_NOT_FOUND")
					continue
				(
					best_digit,
					best_score,
					second_score,
					margin,
					accepted,
					scores,
				) = classify_digit(normalized_digit)
				print("FRAME_DIGIT_SCORES", attempt, scores)
				print("DIGIT_ATTEMPT_RESULT", attempt,
					  "BEST_DIGIT", best_digit,
					  "BEST_SCORE", best_score,
					  "SECOND_SCORE", second_score,
					  "MARGIN", margin,
					  "PANEL_MEAN", panel_info[1],
					  "DIGIT_RATIO", panel_info[2],
					  "DIGIT_BLOB", panel_info[3],
					  "PANEL_MODE", panel_info[4],
					  "ACCEPTED", accepted,
					  "ELAPSED_MS",
					  time.ticks_diff(time.ticks_ms(), attempt_start_ms))
				if accepted:
					break
			except Exception as error:
				accepted = False
				print_error("DIGIT_ATTEMPT_FAIL_%d" % attempt, error)
			finally:
				normalized_digit = None
				img = None
				gc.collect()
		label = 0
		if accepted:
			label = best_digit + 1
		print("TEMPLATE_MATCH", "BEST_DIGIT", best_digit,
			  "BEST_SCORE", best_score,
			  "SECOND_SCORE", second_score,
			  "MARGIN", margin,
			  "ACCEPTED", accepted,
			  "LABEL", label,
			  "ELAPSED_MS",
			  time.ticks_diff(time.ticks_ms(), match_start_ms))
		return send_result(object_type, object_index, label)
	except Exception as error:
		print_error("TEMPLATE_MATCH_FAIL", error)
		return send_result(object_type, object_index, 0)
def classify_and_reply(object_type, object_index):
	if object_type == 1:
		return match_digit_and_reply(object_type, object_index)
	if not ensure_model(object_type):
		return False
	score_sum = [0.0] * MODEL_OUTPUT_COUNT
	try:
		output = read_model_output()
		if output is None:
			return False
		for output_index in range(MODEL_OUTPUT_COUNT):
			score_sum[output_index] += output[output_index]
		best_index = 0
		best_score = score_sum[0]
		for output_index in range(1, MODEL_OUTPUT_COUNT):
			if score_sum[output_index] > best_score:
				best_index = output_index
				best_score = score_sum[output_index]
		return send_result(object_type, object_index, best_index + 1)
	except Exception as error:
		print_error("INFER_FAIL", error)
		return False
uart_ready = True
try:
	uart = UART(12, baudrate=115200)
except Exception as error:
	print_error("UART_INIT_FAIL", error)
	uart_ready = False
camera_ready = True
try:
	sensor.reset()
	sensor.set_pixformat(sensor.RGB565)
	sensor.set_framesize(sensor.QVGA)
	sensor.set_hmirror(True)
	sensor.skip_frames(time = 200)
	sensor.set_vflip(True)
	sensor.skip_frames(time=1000)
	sensor.set_auto_whitebal(False)
	sensor.set_auto_gain(
		False, gain_db=0
	)
	sensor.set_auto_exposure(
		False, exposure_us=500
	)
	sensor.skip_frames(time=300)
except Exception as error:
	print_error("CAMERA_INIT_FAIL", error)
	camera_ready = False
while uart_ready and camera_ready:
	result = check_uart_cmd()
	if result is not None:
		object_type, object_index, object_row, object_col = result
		if object_type == 0 or object_type == 1:
			classify_and_reply(object_type, object_index)
	time.sleep_ms(1)
while True:
	time.sleep_ms(1000)
