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
yaw_pid = PID(p = 0.8, i=0, imax=90) # 偏航方向PID参数
REACTIONTIME = 0 # 指令执行时间
MAXSPEED = 5

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

def saturation(inputValue, thresholds):
    if abs(inputValue) >= thresholds:
        return thresholds
    else:
        return abs(int(inputValue))

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
ROIPLACE_UP = (0, 12, 160, 8)
ROIPLACE_DOWN = (0, 100, 160, 8)
ROIPLACE_MID = (0, 32, 160, 56)
ROIPLACE_LEFT = (12, 0, 8, 120)
ROIPLACE_RIGHT = (140, 0, 8, 120)
LINE_RIGHT = (ROIPLACE_RIGHT[0] + ROIPLACE_RIGHT[2]/2, ROIPLACE_RIGHT[1],
    ROIPLACE_RIGHT[0] + ROIPLACE_RIGHT[2]/2, ROIPLACE_RIGHT[3])
status = 0
# 状态0：还未识别到中间位置
# while(True):
    # img = sensor.snapshot().lens_corr(strength = 1.8)

while(True):
    # img = sensor.snapshot().lens_corr(strength = 1.8)
    img = sensor.snapshot().lens_corr(strength = 1.8).binary([PATHTH], invert = True)
    img.erode(1, threshold = 3)
    if status == 0: # 前进到进入点
        send_direction_packet(G, MAXSPEED) # 以5cm/s的速度前进
        lines = img.find_lines(x_stride = 5, y_stride = 2, threshold = 2200,
            theta_margin = 20, rho_margin = 5):
        stat_down = img.get_statistics([(0, 255)], roi = ROIPLACE_DOWN)
        if lines:
            line = lines[0]
            if abs(sin) >= 0.98 and stat_down.mean()/255 >= 0.4: # 已经前进到进入点
                status = 1 # 正式进入循迹， 开始对齐右侧。
                send_direction_packet(S, 0)
                pyb.delay(REACTIONTIME) # 悬停一秒钟缓冲
        else:
            print(str(status) + '未找到线')
    elif status == 1: # 矫正进入点
        line = img.get_regression([(255, 255)], roi = ROIPLACE_MID)
        if line: # 矫正横向进入位置
            img.draw_line(line.x1(), line.y1(), line.x2(), line.y2(), color = 255)
            width_error = (LINE_RIGHT[0] - (line.x1() + line.x2())/2)
            width_output = width_pid.get_pid(width_error, 1)
            if abs(width_error) < 3: # 右侧循迹线处于可接受范围
                send_direction_packet(G, MAXSPEED) # 继续前进
            else:
                send_direction_packet(G, 0) # 右侧循迹线处于不可接受范围
                if width_error > 0:
                    send_direction_packet(R, saturation(width_output, MAXSPEED))
                else:
                    send_direction_packet(L, saturation(width_output, MAXSPEED))
                pyb.delay(REACTIONTIME)
                # 暂时不考虑无人机在进行左右移动时会前后运动
        line = img.get_regression([(255, 255)], roi = ROIPLACE_UP)
        stat_up = img.get_statistics([(0, 255)], roi = ROIPLACE_UP)
        if line and stat_up.mean()/255 >= 0.4: # 顶部已经检测到直线并且不是干扰
            status = 2

    elif status == 2: # 进入前进位置
        send_direction_packet(E, 0) # 降落
    print(status)
    img.draw_rectangle(ROIPLACE_UP)
    img.draw_rectangle(ROIPLACE_DOWN)
    img.draw_rectangle(ROIPLACE_MID)
    img.draw_rectangle(ROIPLACE_LEFT)
    img.draw_rectangle(ROIPLACE_RIGHT)
    '''
    # a, b = line_to_theta_and_rho_error(line, img)
    # print(a, b)
    # print(line)
    '''
