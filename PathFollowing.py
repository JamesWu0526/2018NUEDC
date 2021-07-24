# Take Off and Path Following - By: JamesWu - 周二 7月 20 2021

import sensor, image, time, pyb
from pid import PID

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)
#VGA:640X480 QVGA：320X240 QQVGA:160X120 QQQVGA:80X60 QQQQVGA:40X30  飞行方向↑
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
uart = pyb.UART(3, 19200)
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
height_pid = PID(p=0.7, i=0, imax=90) # 优先水平方向对齐
width_pid = PID(p=0.7, i=0, imax=90)  # 再垂直方向对齐

# ---------已经达到悬停高度，开始找起飞点-----------
class crdOfCrcl:
    def __init__(self, x=0, y=0, r=0):
        self.x = x
        self.y = y
        self.r = r

crd_crcl = crdOfCrcl(x=0, y=0, r=0) # 初始化圆的参数

while(True):
    clock.tick()
    if  flagHoverHeight: # 开始定高
        flagHoverHeight = 0 # 定高结束
        flagStartCrclDect = 1 # 开始寻找起飞位置

    if  flagStartCrclDect: # 开始寻找起飞位置
        red_led.on()
        print('正在寻找悬停位置')
        img = sensor.snapshot().lens_corr(strength = 1.8)
        crcls = img.find_circles(threshold = 1500, x_margin = 10,
            y_margin = 10, r_margin = 20, r_min = 20)
        if crcls:
            blue_led.off()
            flagStartCrclDect = 0 # 寻找结束并记录基准圆位置
            flagHoverOnCrclTime = 1 # 开始悬停时间等待
            crd_crcl.x = crcls[0][0]
            crd_crcl.y = crcls[0][1]
            crd_crcl.r = crcls[0][2]
        else:
            blue_led.on()
            flagHoverOnCrclTime = 0 # 关闭悬停时间等待
            # 应当调整无人机位置或改变参数等尝试找圆
# -----------------开始驶向圆心----------------------
    if  flagHoverOnCrclTime: # 如果悬停时间未达到15s
        print('正在调整无人机的位置')
        # -----------------继续定位起飞地点-------------------
        img = sensor.snapshot().lens_corr(strength = 1.8)
        crcls = img.find_circles(threshold = 2300, x_margin = 10,
            y_margin = 10, r_margin = 2, r_min = 17)
        if crcls: # 找到一个或多个圆
           crd_crcl.x = crcls[0][0] # 默认记录第一个
           crd_crcl.y = crcls[0][1]
           crd_crcl.r = crcls[0][2]
           img.draw_circle(crd_crcl.x, crd_crcl.y, crd_crcl.r, color = (255, 0, 0))
           img.draw_cross(crd_crcl.x, crd_crcl.y, color = (255, 0, 0))
        else: # 没有找到圆 默认输出上一个
           img.draw_circle(crd_crcl.x, crd_crcl.y, crd_crcl.r, color = (255, 0, 0))
           img.draw_cross(crd_crcl.x, crd_crcl.y, color = (255, 0, 0))
         # -----------------开始应用PID矫正位置------------------
        if(abs(crd_crcl.y - sensor.height()/2) <= 3):
            height_output = 0
            width_error = (crd_crcl.x - sensor.width()/2)/sensor.width()
            width_output = width_pid.get_pid(width_error, 1) # 最大速度1m/s
        else:
            height_error = (crd_crcl.y - sensor.height()/2)/sensor.height()
            height_output = height_pid.get_pid(height_error, 1) # 最大速度1m/s
            width_output = 0 # 不调整垂直方向
        # output_str = "[%d, %d]" % (output_width, output_height) #PID结果是一个浮点数，待修改
        str_output = [width_output, height_output]
        print(str_output)
         # -----------------判断是否悬停在起飞地点的上方并超过15秒--------------
        img.draw_rectangle(int((1/2-HOVERTH[0])*sensor.width()), int((1/2-HOVERTH[1])*sensor.height()),
           int(2*HOVERTH[0]*sensor.width()), int(2*HOVERTH[1]*sensor.height())) # 中心区域
        if(abs(crd_crcl.y - sensor.height()/2) <= sensor.height()*HOVERTH[1] and
            abs(crd_crcl.x - sensor.width()/2) <= sensor.width()*HOVERTH[0]):
            if(not flagStartTiming):
                start = pyb.millis()
                flagStartTiming = 1
            else:
                if  pyb.elapsed_millis(start) >= HOVERTIME:
                    flagStartTimig = 0  # 重置未计时标志位
                    flagHoverOnCrclTime = 0 # 关闭悬停计时模块
                    flagPathFollowing = 1 # 开启循迹模块
            red_led.off()
            green_led.on()
            print('已到达悬停位置, 停留时间：', pyb.elapsed_millis(start))
        else: # 未到达目标区域
            green_led.off()
            red_led.on()
            start = pyb.millis() # 重置计时器
            flagStartTiming = 0 # 重置未计时标志位
            # flagHoverOnCrclTime = 1 # 执行计时模块
            # flagHoverOnCrcl = 0 # 不执行正在悬停模块
            print('未到达悬停位置')

           # print(clock.fps())
# ----------------开始循迹-------------------------
    if  flagPathFollowing:
        red_led.on()
        print('开始循迹, 同时检测火灾')
        img = sensor.snapshot().lens_corr(strength = 1.8)
        continue
    print(clock.fps())
