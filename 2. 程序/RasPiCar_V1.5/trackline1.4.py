'''

Author: Ken Kaneki
Date: 2021-05-28 16:30:09
LastEditTime: 2021-05-28 16:58:31

Description: README
FilePath: \RasPiCar_V1.6\trackline1.2.py

'''
import time
import os
import sys

import cv2
import numpy as np
from MotorModule.Motor import Motor
from AvoidanceModule.Avoidance import RGB
#max_speed=60
#ls rs ah di  way   res
#45 45 40 50  dif_  bad
#50 50 50 80  spin  good
#55 55 55 70  spin  very good
#
    
def trackstop():
    motor.cleanup()
    rgb.cleanup()
    #cap.release()
    #cv2.destroyAllWindows()
    
def Pathtrack(center, base, boundary):
    offset = center-base
    #rate = abs(offset)/base
    # 如果巡线中心点偏左，就需要左转来校正。
    if (offset < -boundary):
        #speed = 45*(1+rate)
        #print("turn left %d" % speed)
        motor.spin_left(55)
    # 如果巡线中心点偏右，就需要右转来校正。
    elif offset > boundary:
        #speed = 45*(1+rate)
        #print("turn right %d" % speed)
        motor.spin_right(55)

    # 如果巡线中心点居中，就可以直行。
    else:
        #speed = 60
        #print("go ahead %d" % speed)
        motor.ahead(55)

base = 320
center = base
motor = Motor()

rgb = RGB()
rgb.led(rgb.Color.Cyan)

#cap = cv2.VideoCapture(0)  # 实例化摄像头

while True:
    break
    '''
    ret, frame = cap.read()
    # 灰度化
    if ret is True:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    else:
        break
    # 二值化
    ret, dst = cv2.threshold(gray, 0, 255, cv2.THRESH_OTSU)
    # 膨胀，黑区域变大
    dst = cv2.dilate(dst, None, iterations=2)

    # 单看第400行的像素值
    color = dst[440]
    # 找到黑色的像素点个数
    black_count = np.sum(color == 0)
    # 找到黑色的像素点索引
    black_index = np.where(color == 0)

    # 防止black_count=0的报错
    if not black_count == 0:
        # 找到黑色像素的中心点位置
        center = (black_index[0][black_count - 1] + black_index[0][0]) / 2
    else:
        center = 0

    #print('center = %d' % center)
    #Pathtrack(center, base, 80)

    # 树莓派桌面显示二值化图像，比较占资源默认注释掉调试时可以打开
    
    cv2.imshow('BINARY', dst)

    if cv2.waitKey(1) & 0XFF == ord('q'):  # 检测到按键q退出
        break
    '''
trackstop()
os.system('python3 raspicar.py')
sys.exit()


