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
    while(flag):
        if uart.any():
            if uart.readchar() == H:
                flag = 0

    green_led.on()
    send_direction_packet(G, velocity[0])
    pyb.delay(2000)
    green_led.off()

    green_led.on()
    send_direction_packet(S, 0)
    pyb.delay(5000)
    green_led.off()


    green_led.on()
    send_direction_packet(B, velocity[0])
    pyb.delay(2000)
    green_led.off()

    green_led.on()
    send_direction_packet(S, 0)
    pyb.delay(5000)
    green_led.off()

    green_led.on()
    send_direction_packet(G, velocity[0])
    pyb.delay(2000)
    green_led.off()

    green_led.on()
    send_direction_packet(S, 0)
    pyb.delay(5000)
    green_led.off()


    green_led.on()
    send_direction_packet(B, velocity[0])
    pyb.delay(2000)
    green_led.off()

    green_led.on()
    send_direction_packet(S, 0)
    pyb.delay(5000)
    green_led.off()

    green_led.on()
    send_direction_packet(G, velocity[0])
    pyb.delay(2000)
    green_led.off()

    green_led.on()
    send_direction_packet(S, 0)
    pyb.delay(5000)
    green_led.off()


    green_led.on()
    send_direction_packet(B, velocity[0])
    pyb.delay(2000)
    green_led.off()

    green_led.on()
    send_direction_packet(S, 0)
    pyb.delay(5000)
    green_led.off()

    send_direction_packet(E, 0)
    '''
    pyb.delay(4000)
    red_led.on()
    send_direction_packet(R, velocity[2])
    pyb.delay(4000)
    red_led.off()
    send_direction_packet(L, velocity[3])
    pyb.delay(4000)
    red_led.on()
    send_direction_packet(E, 0)
    pyb.delay(200)
    red_led.off()
    '''

