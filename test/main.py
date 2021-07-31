# 测试send_direction_packet能否被正常接收
# - By: asus1 - 周三 7月 28 2021
import sensor, image, time, pyb, network, usocket, sys, struct

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)
red_led = pyb.LED(1)
green_led = pyb.LED(2)
blue_led = pyb.LED(3)

clock = time.clock()

C = ord('C') #逆时针
F = ord('F') #顺时针
G = ord('G') #前进
B = ord('B') #后退
R = ord('R') #右
L = ord('L') #左
D = ord('D') #下降
U = ord('U') #上升
E = ord('E') #降落
S = ord('S') #停止
H = ord('H') #允许控制
velocity = [15, 15, 25, 25]
uart = pyb.UART(3, 500000, timeout_char = 1000)

def send_direction_packet(direct, velocity): # 封包函数，只取八位
    s = 0xAA + 0x8C + direct + (int(velocity/256)) + (int(velocity%256))
    s = int(s % 256)
    temp_flow = struct.pack("<BBBBhB", 0xAA, 0x89, 03, direct, velocity, s)
    uart.write(temp_flow)

flag = 1

while(True):
    print(1 and None)
