# 练习文档_1：颜色识别与标记 - By: JamesWu - 周四 7月 15 2021
# 实现功能：识别出图像中的圆，标记圆心并返回其与机体中心的偏差
# 数据传输：像素偏差
# 待解决问题：如何将像素偏差转化为位置偏差？

import sensor, image, time, math
thresholds=(0, 100, 18, 123, -128, 127) #火灾颜色阈值

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA) # 相机模块的帧大小
#VGA:640X480 QVGA：320X240 QQVGA:160X120 QQQVGA:80X60 QQQQVGA:40X30
sensor.set_auto_gain(False) # 关闭自动增益及白平衡
sensor.set_auto_whitebal(False)

sensor.skip_frames(time = 2000)
clock = time.clock()

# Define the position of fire
class crdOfFire:
    def __init__(self, x, y):
        self.x = x
        self.y = y

class crdOfCenter:
    def __init__(self, x, y):
        self.x = x
        self.y = y

crd_center = crdOfCenter(sensor.width()/2, sensor.height()/2)


numOfFire = 4 # initialize numOfFire
while(numOfFire): # Exit loop while all fires extinguished
    clock.tick()
    img = sensor.snapshot().lens_corr(1.8) # snapshot返回image类
    #print(clock.fps())
#---------------------查找火灾颜色并返回坐标---------------------#
    # 考虑一种简单情况：假设图中只存在一个火源
    blobs = img.find_blobs([thresholds], pixels_threshold=2, merge=True) # 查找火源
    if blobs:
        for blob in blobs:
            crd = crdOfFire(blob.cx(), blob.cy()) # record coords of fire
            img.draw_string(crd.x, crd.y, 'FireDetect')
            status = 'FireDetect'
            img.draw_circle(blob.enclosing_circle(), color = (255, 0, 0)) # 标出火源
            if abs(crd.x - crd_center.x) <= sensor.width()/8:
                status = 'AlignWithFire'
                if abs(crd.y - crd_center.y) <= sensor.height()/8:
                    status = 'OnPosition'
    else:
        status = 'FireNotDetect'

    if status == 'FireDetect' or 'FireNotDetect':
        print('move forward')
    elif status == 'AlignWithFire':
        print('move left')
    elif status == 'OnPosition':
        print('extinguishing fire')
