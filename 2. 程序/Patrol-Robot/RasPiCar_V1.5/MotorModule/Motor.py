# -*-coding:utf-8-*-
import sys
import time

import RPi.GPIO as GPIO

# 左前
default_L1_ENA = 16
default_L1_IN1 = 20
default_L1_IN2 = 21
# 左后
default_L2_ENB = 8
default_L2_IN3 = 7
default_L2_IN4 = 1
# 右前
default_L1_ENB = 13
default_L1_IN3 = 19
default_L1_IN4 = 26
# 右后
default_L2_ENA = 10
default_L2_IN1 = 9
default_L2_IN2 = 11

default_speed = 80
default_difrate = 0.5


class Motor(object):
    '''Motor control'''

    def __init__(self, L1_ENA=default_L1_ENA, L1_IN1=default_L1_IN1, L1_IN2=default_L1_IN2,
                 L1_IN3=default_L1_IN3, L1_IN4=default_L1_IN4, L1_ENB=default_L1_ENB,
                 L2_ENA=default_L2_ENA, L2_IN1=default_L2_IN1, L2_IN2=default_L2_IN2,
                 L2_IN3=default_L2_IN3, L2_IN4=default_L2_IN4, L2_ENB=default_L2_ENB):
        '''Specify motor pins'''
        self.__enab_pin = [L1_ENA, L2_ENB, L1_ENB, L2_ENA]  # Enable pins
        self.__inx_pin = [L1_IN1, L1_IN2, L2_IN3, L2_IN4,
                          L1_IN3, L1_IN4, L2_IN1, L2_IN2]  # Control pins
        self.__pwm_speed = [None, None, None, None]
        self.__LeftAheadSpeed = self.__enab_pin[0]
        self.__LeftBackSpeed = self.__enab_pin[1]
        self.__RightAheadSpeed = self.__enab_pin[2]
        self.__RightBackSpeed = self.__enab_pin[3]

        self.__LeftAhead_H = self.__inx_pin[0]
        self.__LeftAhead_L = self.__inx_pin[1]

        self.__LeftBack_H = self.__inx_pin[3]
        self.__LeftBack_L = self.__inx_pin[2]

        self.__RightAhead_H = self.__inx_pin[4]
        self.__RightAhead_L = self.__inx_pin[5]

        self.__RightBack_H = self.__inx_pin[7]
        self.__RightBack_L = self.__inx_pin[6]

        self.setup()

    def setup(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        for pin in self.__inx_pin:
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)

        pin = None
        i = 0
        for pin in self.__enab_pin:
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
            self.__pwm_speed[i] = GPIO.PWM(pin, 2000)  # 设置pwm引脚和频率为2000hz
            self.__pwm_speed[i].start(0)
            i = i+1

    def wheel_rotate(self, pin_H, pin_L, pin_PWM, speed):
        # 0<speed<100 正转
        if speed > 0:
            if speed > 100:
                speed = 100
            GPIO.output(pin_H, GPIO.HIGH)
            GPIO.output(pin_L, GPIO.LOW)
            pin_PWM.ChangeDutyCycle(speed)
        # -100<speed<0 反转
        elif speed < 0:
            if speed < -100:
                speed = -100
            speed = abs(speed)
            GPIO.output(pin_H, GPIO.LOW)
            GPIO.output(pin_L, GPIO.HIGH)
            pin_PWM.ChangeDutyCycle(speed)
        # speed=0 停止
        else:
            GPIO.output(pin_H, GPIO.LOW)
            GPIO.output(pin_L, GPIO.LOW)
            pin_PWM.ChangeDutyCycle(speed)

    def ahead(self, speed=default_speed):

        self.wheel_rotate(self.__LeftAhead_H, self.__LeftAhead_L,
                          self.__pwm_speed[0], speed)
        self.wheel_rotate(self.__LeftBack_H, self.__LeftBack_L,
                          self.__pwm_speed[1], speed)
        self.wheel_rotate(self.__RightAhead_H, self.__RightAhead_L,
                          self.__pwm_speed[2], speed)
        self.wheel_rotate(self.__RightBack_H, self.__RightBack_L,
                          self.__pwm_speed[3], speed)

    def left(self, speed=default_speed):

        self.wheel_rotate(self.__LeftAhead_H, self.__LeftAhead_L,
                          self.__pwm_speed[0], -speed)
        self.wheel_rotate(self.__LeftBack_H, self.__LeftBack_L,
                          self.__pwm_speed[1], speed)
        self.wheel_rotate(self.__RightAhead_H, self.__RightAhead_L,
                          self.__pwm_speed[2], speed)
        self.wheel_rotate(self.__RightBack_H, self.__RightBack_L,
                          self.__pwm_speed[3], -speed)

    def right(self, speed=default_speed):

        self.wheel_rotate(self.__LeftAhead_H, self.__LeftAhead_L,
                          self.__pwm_speed[0], speed)
        self.wheel_rotate(self.__LeftBack_H, self.__LeftBack_L,
                          self.__pwm_speed[1], -speed)
        self.wheel_rotate(self.__RightAhead_H, self.__RightAhead_L,
                          self.__pwm_speed[2], -speed)
        self.wheel_rotate(self.__RightBack_H, self.__RightBack_L,
                          self.__pwm_speed[3], speed)

    def rear(self, speed=default_speed):

        self.wheel_rotate(self.__LeftAhead_H, self.__LeftAhead_L,
                          self.__pwm_speed[0], -speed)
        self.wheel_rotate(self.__LeftBack_H, self.__LeftBack_L,
                          self.__pwm_speed[1], -speed)
        self.wheel_rotate(self.__RightAhead_H, self.__RightAhead_L,
                          self.__pwm_speed[2], -speed)
        self.wheel_rotate(self.__RightBack_H, self.__RightBack_L,
                          self.__pwm_speed[3], -speed)

    # 小车原地左转
    def spin_left(self, speed=default_speed):

        self.wheel_rotate(self.__LeftAhead_H, self.__LeftAhead_L,
                          self.__pwm_speed[0], -speed)
        self.wheel_rotate(self.__LeftBack_H, self.__LeftBack_L,
                          self.__pwm_speed[1], -speed)
        self.wheel_rotate(self.__RightAhead_H, self.__RightAhead_L,
                          self.__pwm_speed[2], speed)
        self.wheel_rotate(self.__RightBack_H, self.__RightBack_L,
                          self.__pwm_speed[3], speed)

    # 小车原地右转
    def spin_right(self, speed=default_speed):

        self.wheel_rotate(self.__LeftAhead_H, self.__LeftAhead_L,
                          self.__pwm_speed[0], speed)
        self.wheel_rotate(self.__LeftBack_H, self.__LeftBack_L,
                          self.__pwm_speed[1], speed)
        self.wheel_rotate(self.__RightAhead_H, self.__RightAhead_L,
                          self.__pwm_speed[2], -speed)
        self.wheel_rotate(self.__RightBack_H, self.__RightBack_L,
                          self.__pwm_speed[3], -speed)
    # 对角线移动

    def diag_WA(self, speed=default_speed):

        self.wheel_rotate(self.__LeftAhead_H, self.__LeftAhead_L,
                          self.__pwm_speed[0], 0)
        self.wheel_rotate(self.__LeftBack_H, self.__LeftBack_L,
                          self.__pwm_speed[1], speed)
        self.wheel_rotate(self.__RightAhead_H, self.__RightAhead_L,
                          self.__pwm_speed[2], speed)
        self.wheel_rotate(self.__RightBack_H, self.__RightBack_L,
                          self.__pwm_speed[3], 0)

    def diag_WD(self, speed=default_speed):

        self.wheel_rotate(self.__LeftAhead_H, self.__LeftAhead_L,
                          self.__pwm_speed[0], speed)
        self.wheel_rotate(self.__LeftBack_H, self.__LeftBack_L,
                          self.__pwm_speed[1], 0)
        self.wheel_rotate(self.__RightAhead_H, self.__RightAhead_L,
                          self.__pwm_speed[2], 0)
        self.wheel_rotate(self.__RightBack_H, self.__RightBack_L,
                          self.__pwm_speed[3], speed)

    def diag_SA(self, speed=default_speed):

        self.wheel_rotate(self.__LeftAhead_H, self.__LeftAhead_L,
                          self.__pwm_speed[0], -speed)
        self.wheel_rotate(self.__LeftBack_H, self.__LeftBack_L,
                          self.__pwm_speed[1], 0)
        self.wheel_rotate(self.__RightAhead_H, self.__RightAhead_L,
                          self.__pwm_speed[2], 0)
        self.wheel_rotate(self.__RightBack_H, self.__RightBack_L,
                          self.__pwm_speed[3], -speed)

    def diag_SD(self, speed=default_speed):

        self.wheel_rotate(self.__LeftAhead_H, self.__LeftAhead_L,
                          self.__pwm_speed[0], 0)
        self.wheel_rotate(self.__LeftBack_H, self.__LeftBack_L,
                          self.__pwm_speed[1], -speed)
        self.wheel_rotate(self.__RightAhead_H, self.__RightAhead_L,
                          self.__pwm_speed[2], -speed)
        self.wheel_rotate(self.__RightBack_H, self.__RightBack_L,
                          self.__pwm_speed[3], 0)

    def Pathtrack(self, center, base=320, boundary=70):
        offset = center-base
        #rate = abs(offset)/base
        # 如果巡线中心点偏左，就需要左转来校正。
        if offset < -boundary:
            #speed = 45*(1+rate)
            #print("turn left %d" % speed)
            self.spin_left(55)
        # 如果巡线中心点偏右，就需要右转来校正。
        elif offset > boundary:
            #speed = 45*(1+rate)
            #print("turn right %d" % speed)
            self.spin_right(55)

        # 如果巡线中心点居中，就可以直行。
        else:
            #speed = 40
            #print("go ahead %d" % speed)
            self.ahead(55)

    def stop(self):

        self.wheel_rotate(self.__LeftAhead_H, self.__LeftAhead_L,
                          self.__pwm_speed[0], 0)
        self.wheel_rotate(self.__LeftBack_H, self.__LeftBack_L,
                          self.__pwm_speed[1], 0)
        self.wheel_rotate(self.__RightAhead_H, self.__RightAhead_L,
                          self.__pwm_speed[2], 0)
        self.wheel_rotate(self.__RightBack_H, self.__RightBack_L,
                          self.__pwm_speed[3], 0)

    def cleanup(self):
        for pin in self.__inx_pin:
            GPIO.cleanup(pin)
        for pin in self.__enab_pin:
            GPIO.cleanup(pin)
        print("Close Motor")
