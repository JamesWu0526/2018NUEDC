# Take Off and Path Following - By: JamesWu - 周二 7月 20 2021

import sensor, image, time, pyb, network, usocket, sys, struct
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

flagHoverHeight = 0 # 开始定高标志位
flagStartCrclDect = 0 # 开始寻找起飞位置
flagStartTiming = 0 # 未计时标志位
flagPathFollowing = 1 # 开始循迹
networkStart = 0 # 开启网络回传


def send_direction_packet(direct, velocity): # 封包函数，只取八位
    s = 0xAA + 0x8C + direct + (int(velocity/256)) + (int(velocity%256))
    s = int(s % 256)
    temp_flow = struct.pack("<BBBBhB", 0xAA, 0x89, 03, direct, velocity, s)
    uart.write(temp_flow)

def saturation(inputValue, thresholds):
    if abs(inputValue) >= thresholds:
        return thresholds
    else:
        return abs(int(inputValue))

def line_to_sin_rho(line):
    if line.rho() <0: # 3|4象限
        return math.sin(math.radians(line.theta() + 180)), abs(line.rho())
    else:
        return math.sin(math.radians(line.theta())), line.rho()


#.....................WIFI图传....................................#
#................................................................#
if networkStart:
    socket_success = 0
    SSID ='OPENMV_AP'    # Network SSID
    KEY  ='1234567890'    # Network key (must be 10 chars)
    HOST = ''           # Use first available interface
    PORT = 8080         # Arbitrary non-privileged port
    wlan = network.WINC(mode=network.WINC.MODE_AP)
    wlan.start_ap(SSID, key=KEY, security=wlan.WEP, channel=2)

    def send_frame(frame):
        cframe = frame.compressed(quality=50)
        header = "\r\n--openmv\r\n" \
                 "Content-Type: image/jpeg\r\n"\
                 "Content-Length:"+str(cframe.size())+"\r\n\r\n"
        client.send(header)
        client.send(cframe)

    while (socket_success == 0):
        red_led.on()
        pyb.delay(500)
        red_led.off()
        pyb.delay(500)
        # Create server socket
        s = usocket.socket(usocket.AF_INET, usocket.SOCK_STREAM)
        try:
            # Bind and listen
            s.bind([HOST, PORT])
            s.listen(5)

            # Set server socket timeout
            # NOTE: Due to a WINC FW bug, the server socket must be closed and reopened if
            # the client disconnects. Use a timeout here to close and re-create the socket.
            s.settimeout(3)
            print ('Waiting for connections..')
            client, addr = s.accept()
            # set client socket timeout to 2s
            client.settimeout(2.0)
            print ('Connected to ' + addr[0] + ':' + str(addr[1]))

            # Read request from client
            data = client.recv(1024)
            # Should parse client request here

            # Send multipart header
            client.send("HTTP/1.1 200 OK\r\n" \
                        "Server: OpenMV\r\n" \
                        "Content-Type: multipart/x-mixed-replace;boundary=openmv\r\n" \
                        "Cache-Control: no-cache\r\n" \
                        "Pragma: no-cache\r\n\r\n")
            # FPS clock
            # clock = time.clock()
            socket_success = 1
            continue
        except OSError as e:
            socket_success = 0
            s.close()
            print("socket error: ", e)

#-----------------------------------------------------------------------#
#-----------------------------------------------------------------------#

#--------------悬停参数--------------#
HOVERTIME = 15000 # 最大停留时间
HOVERTH = (0.09, 0.12, 0.06, 0.08) # [允许计时范围, 允许悬停范围] ,越小越苛刻
PATHTH = (0, 18, -10, 15, -15, 15)
MAXSPEED = 5
tolHoverTime = 0
REACTIONTIME = 0 # 飞行器指令响应时间：REACTIONTIMEms
height_pid = PID(p=0.8, i=0, imax=90) # 水平方向PID参数
width_pid = PID(p=0.8, i=0, imax=90)  # 垂直方向PID参数
#--------------循迹参数---------------#
ROIPLACE_MID = (1, 60-4, 160, 8)
ROIPLACE_UP_ALL = (1, 1, 160, 58)
ROIPLACE_UP = (1, 20-4, 160, 8)
ROIPLACE_RIGHT = (140-2, 1, 4, 120)
ROIPLACE_DOWN = (1, 100-2, 160, 4)
# LINE_UP, LINE_LEFT, LINE_DOWN
LINE_RIGHT = (140, 1, 140, 120)
PATHTH = (67, 168) # 识别直线的阈值
crd_line = (sensor.width()/2, sensor.height()/2) # 设置期望位置
status = 0

while(True):
    clock.tick()
    while(flagHoverHeight): # 开始定高
        blue_led.on()
        pyb.delay(500)
        blue_led.off()
        pyb.delay(500)
        if uart.readchar() == H:
           flagHoverHeight = 0 # 定高结束
           flagStartPathFollowing = 1 # 开始寻找起飞位置

# -----------------继续定位起飞/降落地点-------------------
    if  flagStartCrclDect: # 开始寻找起飞/降落位置
        img = sensor.snapshot().lens_corr(strength = 1.8)
        img.draw_rectangle(int((1/2-HOVERTH[0])*sensor.width()), int((1/2-HOVERTH[1])*sensor.height()),
          int(2*HOVERTH[0]*sensor.width()), int(2*HOVERTH[1]*sensor.height())) # 中心区域
        crcls = img.find_circles(threshold = 2600, x_margin = 10,
            y_margin = 10, r_margin = 2, r_min = 22, r_max = 30)
        if crcls: # 找到一个或多个圆
            circle = crcls[0]
            img.draw_circle(circle.x(), circle.y(), circle.r(), color = (255, 0, 0))
            img.draw_cross(circle.x(), circle.y(), color = (255, 0, 0))
            crcl_string = 'threshold:' + str(crcls[0][3]) + 'r:' + str(circle.r())
            img.draw_string(1, 1, crcl_string)
            # -----------------开始应用PID矫正位置------------------
            # if (在可接受的范围内)：悬停 else: 调整位置
            if(abs(circle.y() - sensor.height()/2) <= sensor.height()*HOVERTH[2] and
            abs(circle.x() - sensor.width()/2) <= sensor.width()*HOVERTH[3]):
                send_direction_packet(S, 0)
            else:
                width_error = (sensor.width()/2 - circle.x())*2/sensor.width()
                width_output = width_pid.get_pid(width_error, 20)
                print("水平移动速度：", saturation(width_output, MAXSPEED))
                if circle.x() > sensor.width()/2: # 应当往右飞
                    send_direction_packet(R, saturation(width_output, MAXSPEED))
                else: # 应当往左飞
                    send_direction_packet(L, saturation(width_output, MAXSPEED))
                pyb.delay(REACTIONTIME) # 留给飞行器REACTIONTIMEms的响应时间
                height_error = (sensor.height()/2 - circle.y())*2/sensor.height()
                height_output = height_pid.get_pid(height_error, 20)
                print("垂直移动速度：", saturation(height_output, MAXSPEED))
                if circle.y() > sensor.height()/2: # 应当往后飞
                    send_direction_packet(B, saturation(height_output, MAXSPEED))
                else: # 应当往前飞
                    send_direction_packet(G, saturation(height_output, MAXSPEED))
                pyb.delay(REACTIONTIME) # 留给飞行器REACTIONTIMEms的响应时间
            # -----------------判断是否悬停在起飞地点的上方并超过15秒--------------
            if(abs(circle.y() - sensor.height()/2) <= sensor.height()*HOVERTH[1] and
               abs(circle.x() - sensor.width()/2) <= sensor.width()*HOVERTH[0]):
                red_led.off()
                green_led.on()
                if(not flagStartTiming):
                    HoverTime = 0
                    start = pyb.millis()
                    flagStartTiming = 1
                else:
                   HoverTime = pyb.elapsed_millis(start)
                   if  HoverTime + tolHoverTime >= HOVERTIME:
                       flagStartTimig = 0  # 重置未计时标志位
                       flagStartCrclDect = 0 # 关闭起飞模块
                       flagPathFollowing = 1 # 开启循迹模块
                       # sensor.set_pixformat(sensor.RGB565)
                string = str(tolHoverTime + HoverTime)
            else: # 未到达目标区域
                if flagStartTiming :
                   HoverTime = pyb.elapsed_millis(start)
                   tolHoverTime += HoverTime
                green_led.off()
                red_led.on()
                start = pyb.millis() # 重置计时器
                flagStartTiming = 0 # 重置未计时标志位
                string = str(tolHoverTime)
            img.draw_string(80, 60, string, scale = 1)
        else: # 没有找到圆则悬停（后期改为按照某一规则移动）
            send_direction_packet(S, 0)

    # ------------------开始循迹-----------------------
    # 模式一
    # GRAYSCALE下配合 PATHTH = (67, 170)
    # 进行二值化处理，在开启前需要关闭白平衡和自动增益。
    # sensor.set_auto_gain(False)
    # ensor.set_auto_whitebal(False)
    # 目前的结果发现首先利用GRAYSCALE再配合binary来实现将会更加稳定。
    # 再利用erode来进行噪点消除
    # 状态0：还未识别到中间位置
    # REACTIONTIME = 1000
    if  flagPathFollowing:
        red_led.on()
        blue_led.on() #开启循迹提示灯
        # img = sensor.snapshot().lens_corr(strength = 1.8)
        send_direction_packet(G, 5) # 以5cm/s的速度前进2s
        img = sensor.snapshot().lens_corr(strength = 1.8).binary([PATHTH], invert = True)
        line = img.get_regression([(255, 255)], robust = True)
        if line:
            img.draw_line(line.x1(), line.y1(), line.x2(), line.y2(), color = [255, 0 ,0])
            crd_line = ((line.x1()+line.x2())/2, (line.y1()+line.y2())/2)
        print(crd_line)
        if abs((crd_line[0] - sensor.width()/2)) >= 5: # 需要调整无人机的左右位置
            width_error = crd_line[0]*2/sensor.width()-1
            width_output = width_pid.get_pid(width_error, 10)
            if crd_line[0] >= sensor.width()/2:
                send_direction_packet(R, saturation(width_output, MAXSPEED))
                print(width_output)
                pyb.delay(REACTIONTIME)
            else:
                send_direction_packet(L, saturation(width_output, MAXSPEED))
                print(width_output)
                pyb.delay(REACTIONTIME)

        line_up = img.get_regression([(255, 255)], robust = False, roi = ROIPLACE_UP)
        line_mid = img.get_regression([(255, 255)], robust = False, roi = ROIPLACE_MID)
        if line_up == None and line_mid == None:
            print("STOPFLYING")
            send_direction_packet(E, 0)

    if networkStart:
        send_frame(img) # WIFI发送实时图像
