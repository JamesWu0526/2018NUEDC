import sensor, image, time, pyb, math, struct
from pid import PID

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)
#VGA:640X480 QVGA：320X240 QQVGA:160X120 QQQVGA:80X60 QQQQVGA:40X30  飞行方向↑
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
uart = pyb.UART(3, 500000, timeout_char = 1000)
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
red_led   = pyb.LED(1)
green_led = pyb.LED(2)
blue_led  = pyb.LED(3)
clock = time.clock()

flagHoverHeight = 1 # 开始定高标志位
flagStartCrclDect = 0 # 开始寻找起飞位置
flagHoverOnCrclTime = 0 # 开始悬停计时
flagStartTiming = 0 # 未计时标志位
flagPathFollowing = 0 # 开始循迹

HOVERTIME = 15000 # 最大停留时间
HOVERTH = (0.06, 0.08) # 认为悬停在中心的阈值(width, height) ,越小越苛刻
PATHTH = (67, 168)
height_pid = PID(p=0.8, i=0, imax=90) # 优先水平方向对齐
width_pid = PID(p=0.8, i=0, imax=90)  # 再垂直方向对齐

def send_direction_packet(direct, velocity): # 封包函数，只取八位
    s = 0xAA + 0x8C + direct + (int(velocity/256)) + (int(velocity%256))
    s = int(s % 256)
    temp_flow = struct.pack("<BBBBhB", 0xAA, 0x89, 03, direct, velocity, s)
    uart.write(temp_flow)

def line_to_sin_rho(line):
    if line.rho() <0: # 3|4象限
        return math.sin(math.radians(line.theta() + 180)), abs(line.rho())
    else:
        return math.sin(math.radians(line.theta())), line.rho()

# ------------目前的识别模式-------------
# 模式一
# RGB565下配合 binary(PATHTH), PATHTH = (29, 69, -16, 13, -12, 29)
# 进行二值化处理
# 模式二
# GRAYSCALE下配合 PATHTH = (67, 170)
# 进行二值化处理，在开启前需要关闭白平衡和自动增益。
# sensor.set_auto_gain(False)
# ensor.set_auto_whitebal(False)
# 目前的结果发现首先利用GRAYSCALE再配合binary来实现将会更加稳定。
# 再利用erode来进行噪点消除
ROI_WIDTH = ROI_WIDTH
ROIPLACE_UP = (0, 20-ROI_WIDTH/2, 160, ROI_WIDTH)
ROIPLACE_DOWN = (0, 100-ROI_WIDTH/2, 160, ROI_WIDTH)
ROIPLACE_MID = (0, 60-ROI_WIDTH/2, 160, ROI_WIDTH)
ROIPLACE_RIGHT = (140-ROI_WIDTH/2, 0, ROI_WIDTH, 120)
ROIPLACE_LEFT = (20-ROI_WIDTH/2, 0, ROI_WIDTH, 120)
ROIPLACE_UP_ALL = (0, 0, 160, 60-ROI_WIDTH/2)
# LINE_UP, LINE_LEFT, LINE_DOWN
LINE_RIGHT = (140, 1, 140, 120)
status = 0
# 状态0：还未识别到中间位置

while(True):
    clock.tick()
    # img = sensor.snapshot().lens_corr(strength = 1.8)
    img = sensor.snapshot().lens_corr(strength = 1.8).binary([PATHTH], invert = True)
    img.erode(1, threshold = 3)
    if status == 0: # 前进到进入点
        send_direction_packet(G, 5) # 以5cm/s的速度前进
        line = img.get_regression([(255, 255)], roi = ROIPLACE_MID)
        if line:
            sin, rho = line_to_sin_rho(line)
            print(sin)
            if abs(sin) >= 0.98: # 已经前进到进入点
                status = 1 # 正式进入循迹， 开始对齐右侧。
                send_direction_packet(S, 0)
                pyb.delay(0) # 悬停一秒钟缓冲
        else:
            print(str(status) + '未找到线')
    elif status == 1: # 矫正进入点
        send_direction_packet(S, 0)
        line = img.get_regression([(255, 255)], roi = ROIPLACE_UP_ALL)
        img.draw_line(line.x1(), line.y1(), line.x2(), line.y2(), color = 255)
        if line: # 矫正横向进入位置
            sin, rho = line_to_sin_rho(line)

    '''
    elif status == 1:
        line = img.get_regression([(255, 255)], robust = True, roi = ROIPLACE_UP_ALL)
        if line:
            width_error = (line.x1() + line.x2())/2 - LINE_RIGHT[0]
            if abs(width_error) > 5:
                width_output = width_pid.get_pid(width_error, 25)
                if width_error > 0:
                    send_direction_packet(L, width_output)
                else:
                    send_direction_packet(R, width_output)
            else:
                status = 2 # 进入右侧巡线飞行阶段
        else:
            print('巡线错误' + str(status) + '未找到线')
    elif status == 2:
        line = img.get_regression([(255, 255)], robust = True, roi = ROIPLACE_RIGHT)
    print(status)
    '''
    img.draw_rectangle(ROIPLACE_UP)
    img.draw_rectangle(ROIPLACE_DOWN)
    img.draw_rectangle(ROIPLACE_MID)
    img.draw_rectangle(ROIPLACE_LEFT)
    img.draw_rectangle(ROIPLACE_RIGHT)
    # a, b = line_to_theta_and_rho_error(line, img)
    # print(a, b)
    # print(line)
