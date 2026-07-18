import pyb
import sensor, image, time, math
import os, tf
from machine import UART
uart = UART(12, baudrate=115200)
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_brightness(500)
sensor.skip_frames(time = 300)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(True,(0,0x80,0))
clock = time.clock()
roi12 = (35, 10, 225, 160)
net_path ="mobilenet_v2-2026-07-10T07-36-10.164Z_in-int8_out-int8_channel_ptq.tflite"
labels = [line.rstrip() for line in open("/sd/labels.txt")]
net = tf.load(net_path, load_to_fb=True)
net_path2 ="mobilenet_v2-2026-07-13T09-04-45.448Z_channel_ptq.tflite"
labels2 = [line.rstrip() for line in open("/sd/labels2.txt")]
net2 = tf.load(net_path2, load_to_fb=True)
RX_FRAME_LEN = 8
rx_buffer = bytearray()

def check_uart_cmd(uart):
    count = uart.any()
    if count > 0:
        data = uart.read(count)
        if data:
            rx_buffer.extend(data)

    while True:
        if len(rx_buffer) == 0:
            return None

        # 单独留下可能的帧头A5，其他单字节乱码直接丢弃
        if len(rx_buffer) == 1:
            if rx_buffer[0] != 0xA5:
                del rx_buffer[:]
            return None

        # 在缓冲区中搜索连续帧头 A5 B5
        frame_start = -1
        for i in range(len(rx_buffer) - 1):
            if rx_buffer[i] == 0xA5 and rx_buffer[i + 1] == 0xB5:
                frame_start = i
                break

        if frame_start < 0:
            # 保留末尾A5，它可能是下一帧帧头的第一字节
            keep_a5 = rx_buffer[-1] == 0xA5
            del rx_buffer[:]
            if keep_a5:
                rx_buffer.append(0xA5)
            return None

        # 丢掉帧头前面的乱码
        if frame_start > 0:
            del rx_buffer[:frame_start]

        # 已找到帧头，但完整帧还没有接收完
        if len(rx_buffer) < RX_FRAME_LEN:
            return None

        c1 = rx_buffer[2]
        c2 = rx_buffer[3]
        c3 = rx_buffer[4]
        c4 = rx_buffer[5]
        check = rx_buffer[6]

        if (rx_buffer[7] == 0xC5 and
                check == (c1 ^ c2 ^ c3 ^ c4)):
            del rx_buffer[:RX_FRAME_LEN]
            return c1, c2, c3, c4, check

        # 帧尾或校验错误：只丢掉当前A5，然后重新搜索
        # 不能一次丢掉8字节，否则可能把下一个有效帧一起丢掉
        del rx_buffer[0]
def send_uart(object_type, object_index, label):
    check = object_type ^ object_index ^ label
    uart.write(bytearray([
        0xA4, 0xB4,
        object_type,
        object_index,
        label,
        check,
        0xC4
    ]))
while(True):
	img = sensor.snapshot()
	uart_num = uart.any()
	img.draw_rectangle(roi12, color= (255,255,255))
	result = check_uart_cmd(uart)
	if result is not None:
		c1, c2, c3, c4, c5 = result
		if c5 == (c1 ^ c2 ^ c3 ^ c4):
			if c1 == 0:
				for obj in tf.classify(net , img, roi = roi12,min_scale=1.0, scale_mul=0.5, x_overlap=0.0, y_overlap=0.0):
					sorted_list = sorted(zip(labels, obj.output()), key = lambda x: x[1], reverse = True)
					temp1 = 0
					if ((sorted_list[0][0]) == "00mickey_mouse"):
						temp1 = 1
						send_uart(c1,c2,temp1)
					elif ((sorted_list[0][0]) == "01pikachu"):
						temp1 = 2
						send_uart(c1,c2,temp1)
					elif ((sorted_list[0][0]) == "02spongebob_squarepants"):
						temp1 = 3
						send_uart(c1,c2,temp1)
					elif ((sorted_list[0][0]) == "03pleasant_sheep"):
						temp1 = 4
						send_uart(c1,c2,temp1)
					elif ((sorted_list[0][0]) == "04donald_duck"):
						temp1 = 5
						send_uart(c1,c2,temp1)
					elif ((sorted_list[0][0]) == "05nezha"):
						temp1 = 6
						send_uart(c1,c2,temp1)
					elif ((sorted_list[0][0]) == "06big_head_son"):
						temp1 = 7
						send_uart(c1,c2,temp1)
					elif ((sorted_list[0][0]) == "07gg_bond"):
						temp1 = 8
						send_uart(c1,c2,temp1)
					elif ((sorted_list[0][0]) == "08calabash_brothers"):
						temp1 = 9
						send_uart(c1,c2,temp1)
					elif ((sorted_list[0][0]) == "09grey_wolf"):
						temp1 = 10
						send_uart(c1,c2,temp1)
			elif c1 == 1:
				for obj in tf.classify(net2 , img, roi = roi12,min_scale=1.0, scale_mul=0.5, x_overlap=0.0, y_overlap=0.0):
					sorted_list = sorted(zip(labels2, obj.output()), key = lambda x: x[1], reverse = True)
					temp2 = 0
					if ((sorted_list[0][0]) == "0"):
						temp2 = 1
						send_uart(c1,c2,temp2)
					elif ((sorted_list[0][0]) == "1"):
						temp2 = 2
						send_uart(c1,c2,temp2)
					elif ((sorted_list[0][0]) == "2"):
						temp2 = 3
						send_uart(c1,c2,temp2)
					elif ((sorted_list[0][0]) == "3"):
						temp2 = 4
						send_uart(c1,c2,temp2)
					elif ((sorted_list[0][0]) == "4"):
						temp2 = 5
						send_uart(c1,c2,temp2)
					elif ((sorted_list[0][0]) == "5"):
						temp2 = 6
						send_uart(c1,c2,temp2)
					elif ((sorted_list[0][0]) == "6"):
						temp2 = 7
						send_uart(c1,c2,temp2)
					elif ((sorted_list[0][0]) == "7"):
						temp2 = 8
						send_uart(c1,c2,temp2)
					elif ((sorted_list[0][0]) == "8"):
						temp2 = 9
						send_uart(c1,c2,temp2)
					elif ((sorted_list[0][0]) == "9"):
						temp2 = 10
						send_uart(c1,c2,temp2)
	time.sleep_ms(10)