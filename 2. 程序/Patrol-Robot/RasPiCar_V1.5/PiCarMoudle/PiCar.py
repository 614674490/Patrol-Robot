# -*-coding:utf-8-*-
import os
import sys
import threading
import traceback
from multiprocessing import Process
from multiprocessing.managers import BaseManager
from time import sleep
import cv2
import numpy as np

import pygame
from AvoidanceModule.Avoidance import HC_SR04, RGB
from MotorModule.Motor import Motor
from pygame.locals import *
from SteeringModule.Steering import Steering

sys.path.append("..")


class Shared(object):
    Mode = {'End': -1, 'Wait': 0, 'Manual': 1,
            'Automatic-Avoidance': 2, 'Automatic-Trackline': 3}
    FLAG = Mode['Wait']
    mykey = None
    lock = threading.RLock()
    distance = 0
    rgb = RGB()
    rgb.led(rgb.Color.Red)
    print('Start Init IO ...')
    motor = Motor()
    steer = Steering()
    hc_sr04 = HC_SR04()
    #cap = cv2.VideoCapture(0)
    # DISPLAY=0.0; sudo python3 yourpythonscript.py
    pygame.init()
    screen = pygame.display.set_mode((640, 480))
    pygame.display.set_caption('RasPiCar System')
    bg = pygame.image.load(
        "/home/pi/Desktop/Patrol-Robot/RasPiCar_V1.5/PiCarMoudle/bg.jpg").convert()
    # 在窗口上绘制图片
    screen.blit(bg, (0, 0))
    # 更新图片
    pygame.display.update()
    rgb.led(rgb.Color.Cyan)
    print('IO Init OK')

    def getFLAG(self):
        return Shared.FLAG

    def getMode(self, mode):
        return Shared.Mode[mode]

    def getRGB(self):
        return Shared.rgb

    def getMotor(self):
        return Shared.motor

    def getCap(self):
        return Shared.cap

    def setFLAG(self, mode):
        Shared.FLAG = Shared.Mode[mode]

    def cleanup():
        #Shared.cap.release()
        Shared.rgb.cleanup()
        Shared.motor.cleanup()
        Shared.steer.cleanup()
        Shared.hc_sr04.cleanup()


BaseManager.register("SharedData", Shared)
manager = BaseManager()
manager.start()
data = manager.SharedData()
print('SharedData Manager OK')


class Thread_Control (threading.Thread):

    def __init__(self, threadID):

        threading.Thread.__init__(self)
        self.threadID = threadID
        self._stop_event = threading.Event()

    def stop(self):
        self._stop_event.set()

    def run(self):
        while 1:
            try:
                Shared.lock.acquire()
                for event in pygame.event.get():
                    if event.type == pygame.KEYDOWN:
                        if event.key == K_ESCAPE:
                            print('Close PiCar System')
                            Shared.FLAG = Shared.Mode['End']
                            data.setFLAG('End')
                            # Shared.cleanup()
                            pygame.quit()
                            os.system("sudo service PiCarService stop")
                            sys.exit()

                        elif Shared.FLAG == Shared.Mode['Wait'] and event.key == K_SPACE:
                            Shared.FLAG = Shared.Mode['Manual']
                            data.setFLAG('Manual')
                            Shared.rgb.led(Shared.rgb.Color.Green)
                            print('Manual Control Mode')

                        elif Shared.FLAG and event.key == K_1:
                            Shared.motor.stop()
                            Shared.FLAG = Shared.Mode['Manual']
                            data.setFLAG('Manual')
                            Shared.rgb.led(Shared.rgb.Color.Green)
                            print('Manual Control Mode')

                        elif Shared.FLAG and event.key == K_2:
                            Shared.motor.stop()
                            Shared.FLAG = Shared.Mode['Automatic-Avoidance']
                            data.setFLAG('Automatic-Avoidance')
                            Shared.rgb.led(Shared.rgb.Color.Yellow)
                            print('Automatic-Avoidance Control Mode')

                        elif Shared.FLAG and event.key == K_3:
                            Shared.motor.stop()
                            Shared.hc_sr04.servo.clear()
                            Shared.FLAG = Shared.Mode['Automatic-Trackline']
                            data.setFLAG('Automatic-Trackline')
                            Shared.rgb.led(Shared.rgb.Color.Cyan)
                            print('Automatic-Trackline Control Mode')

                        elif event.key == K_END:

                            print('End PiCar System')
                            Shared.FLAG = Shared.Mode['End']
                            data.setFLAG('End')
                            # Shared.cleanup()
                            pygame.quit()
                            os.system("sudo poweroff")
                            sys.exit()

                        elif event.key == K_HOME:

                            print('Restart PiCar System')
                            Shared.FLAG = Shared.Mode['End']
                            data.setFLAG('End')
                            # Shared.cleanup()
                            pygame.quit()
                            os.system("sudo reboot")
                            sys.exit()

                pygame.display.update()
                Shared.lock.release()

            except Exception:
                print('PiCar Exception End')
                traceback.print_exc()
                Shared.FLAG = Shared.Mode['End']
                data.setFLAG('End')
                Shared.cleanup()
                Shared.rgb.led(Shared.rgb.Color.Red)
                pygame.quit()
                sys.exit()


class Thread_Motor (threading.Thread):

    def __init__(self, threadID):

        threading.Thread.__init__(self)
        self.threadID = threadID
        self._stop_event = threading.Event()

    def stop(self):
        self._stop_event.set()

    def run(self):

        try:
            # 避障专用参数
            turn_flag = 0
            outway_flag = 0
            count = 0
            # 巡线专用参数
            base = 320
            center = base

            while Shared.FLAG == Shared.Mode['Wait']:
                pass
            while Shared.FLAG > 0:
                # Shared.lock.acquire()

                if Shared.FLAG == Shared.Mode['Manual']:
                    mykey = pygame.key.get_pressed()
                    # Chassis Control
                    if mykey[pygame.K_w]:
                        if mykey[pygame.K_a]:
                            Shared.motor.diag_WA()
                        elif mykey[pygame.K_d]:
                            Shared.motor.diag_WD()
                        else:
                            Shared.motor.ahead()
                    elif mykey[pygame.K_s]:
                        if mykey[pygame.K_a]:
                            Shared.motor.diag_SA()
                        elif mykey[pygame.K_d]:
                            Shared.motor.diag_SD()
                        else:
                            Shared.motor.rear()
                    elif mykey[pygame.K_a]:
                        if mykey[pygame.K_LCTRL]:
                            Shared.motor.spin_left()
                        else:
                            Shared.motor.left()
                    elif mykey[pygame.K_d]:
                        if mykey[pygame.K_LCTRL]:
                            Shared.motor.spin_right()
                        else:
                            Shared.motor.right()
                    else:
                        Shared.motor.stop()
                    pygame.event.pump()

                elif Shared.FLAG == Shared.Mode['Automatic-Avoidance']:
                    # Shared.lock.acquire()
                    if Shared.hc_sr04.dectect_block():
                        Shared.motor.ahead()
                    else:
                        Shared.motor.stop()
                        # 确定转向
                        if turn_flag == 0:
                            turn_flag, outway_flag = Shared.hc_sr04.right_detect()
                            # 左转没有找到出路 开始左转
                            if turn_flag == 0:
                                Shared.hc_sr04.servo.clear()
                                turn_flag, outway_flag = Shared.hc_sr04.left_detect()
                        else:
                            turn_flag, outway_flag = Shared.hc_sr04.left_detect()
                            if turn_flag:
                                Shared.hc_sr04.servo.clear()
                                turn_flag, outway_flag = Shared.hc_sr04.right_detect()

                        Shared.hc_sr04.servo.clear()

                        if not outway_flag:
                            #print('start right search ...')
                            Shared.motor.spin_right(60)

                        elif turn_flag:
                            #print('start turn right ...')
                            Shared.motor.spin_right(60)

                        else:
                            #print('turn left ...')
                            Shared.motor.spin_left(60)

                        while True:
                            count = count+1
                            if Shared.hc_sr04.dectect_block() and count > 15:  # 80 75 300
                                #print('find way ahead ...')
                                Shared.motor.stop()
                                break
                            elif count > 90:
                                break
                        count = 0
                        if not outway_flag:
                            print('no outway! switch to manualmode')
                            Shared.hc_sr04.servo.clear()
                            Shared.FLAG = Shared.Mode['Manual']
                            data.setFLAG('Manual')
                    # Shared.lock.release()
                # Shared.lock.release()
            Shared.motor.cleanup()
        except Exception:
            print('Motor Exception End')
            traceback.print_exc()
            Shared.FLAG = Shared.Mode['End']
            data.setFLAG('End')
            pygame.quit()
            Shared.motor.cleanup()
            Shared.rgb.led(Shared.rgb.Color.Red)
            sys.exit()


class Thread_Steer (threading.Thread):

    def __init__(self, threadID):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self._stop_event = threading.Event()

    def stop(self):
        self._stop_event.set()

    def run(self):
        try:
            while Shared.FLAG == Shared.Mode['Wait']:
                pass
            while Shared.FLAG > 0:
                # Shared.lock.acquire()

                mykey = pygame.key.get_pressed()
                # Gimbal Control
                if mykey[pygame.K_KP0]:
                    Shared.steer.clear()

                if mykey[pygame.K_UP]:
                    Shared.steer.Up()
                elif mykey[pygame.K_DOWN]:
                    Shared.steer.Down()
                else:
                    Shared.steer.p.stop()

                if mykey[pygame.K_LEFT]:
                    Shared.steer.Left()
                elif mykey[pygame.K_RIGHT]:
                    Shared.steer.Right()
                else:
                    Shared.steer.y.stop()
                pygame.event.pump()

                # Shared.lock.release()
            Shared.steer.cleanup()
        except Exception:
            print('Steer Exception End')
            traceback.print_exc()
            Shared.FLAG = Shared.Mode['End']
            data.setFLAG('End')
            pygame.quit()
            Shared.steer.cleanup()
            Shared.rgb.led(Shared.rgb.Color.Red)
            sys.exit()


class Thread_Dectect (threading.Thread):

    def __init__(self, threadID):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self._stop_event = threading.Event()

    def stop(self):
        self._stop_event.set()

    def run(self):
        try:
            while Shared.FLAG == Shared.Mode['Wait']:
                pass
            while Shared.FLAG > 0:
                # Shared.lock.acquire()

                if Shared.FLAG == Shared.Mode['Manual']:
                    mykey = pygame.key.get_pressed()
                    # Servo Control
                    if mykey[pygame.K_KP5]:
                        Shared.hc_sr04.servo.clear()
                    elif mykey[pygame.K_KP4]:
                        Shared.hc_sr04.Left()
                    elif mykey[pygame.K_KP6]:
                        Shared.hc_sr04.Right()
                    elif mykey[pygame.K_KP_MINUS]:
                        Shared.rgb.led(Shared.rgb.Color.Cyan)
                        Shared.distance = Shared.hc_sr04.get_distance()
                    else:
                        Shared.hc_sr04.servo.stop()
                pygame.event.pump()

                # Shared.lock.release()
            Shared.hc_sr04.cleanup()
            Shared.rgb.cleanup()
        except Exception:
            print('HC-SR04 Exception End')
            traceback.print_exc()
            Shared.FLAG = Shared.Mode['End']
            data.setFLAG('End')
            pygame.quit()
            Shared.hc_sr04.cleanup()
            Shared.rgb.led(Shared.rgb.Color.Red)
            sys.exit()


class AutoModeProcess(Process):

    def __init__(self):
        Process.__init__(self)

    def run(self):
        # 巡线专用参数
        base = 320
        center = base
        cap = cv2.VideoCapture(0)
        while data.getFLAG() == Shared.Mode['Wait']:
            pass
        while data.getFLAG() > 0:

            if data.getFLAG() == Shared.Mode['Automatic-Trackline']:
                ret, frame = cap.read()
                # 灰度化
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
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
                    center = (
                        black_index[0][black_count - 1] + black_index[0][0]) / 2
                    Shared.motor.Pathtrack(center)
                else:
                    center = 0
                    Shared.motor.stop()
                    data.setFLAG('Manual')
                '''
                cv2.imshow('BINARY', dst)

                if cv2.waitKey(1) & 0XFF == ord('q'):  # 检测到按键q退出
                    break
                '''
        cap.release()
