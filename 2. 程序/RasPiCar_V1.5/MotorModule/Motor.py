# -*-coding:utf-8-*-
import sys
import time

import RPi.GPIO as GPIO

default_ENA = 16
default_IN1 = 20
default_IN2 = 21
default_IN3 = 19
default_IN4 = 26
default_ENB = 13
default_speed = 80
default_difrate = 0.5


class Motor(object):
    '''Motor control'''

    def __init__(self, ENA=default_ENA, IN1=default_IN1, IN2=default_IN2, IN3=default_IN3, IN4=default_IN4, ENB=default_ENB):
        '''Specify motor pins'''
        self.__enab_pin = [ENA, ENB]  # Enable pins
        self.__inx_pin = [IN1, IN2, IN3, IN4]  # Control pins

        self.__LeftControl = self.__enab_pin[0]
        self.__RightControl = self.__enab_pin[1]

        self.__LeftAhead = self.__inx_pin[0]
        self.__LeftBack = self.__inx_pin[1]
        self.__RightAhead = self.__inx_pin[2]
        self.__RightBack = self.__inx_pin[3]

        self.setup()

    def setup(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        for pin in self.__inx_pin:
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)

        pin = None

        for pin in self.__enab_pin:
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
        # 设置pwm引脚和频率为2000hz
        self.LeftSpeed = GPIO.PWM(self.__LeftControl, 2000)
        self.RightSpeed = GPIO.PWM(self.__RightControl, 2000)
        self.LeftSpeed.start(0)
        self.RightSpeed.start(0)

    def ahead(self, speed=default_speed):

        GPIO.output(self.__LeftAhead, GPIO.HIGH)
        GPIO.output(self.__LeftBack, GPIO.LOW)

        GPIO.output(self.__RightAhead, GPIO.HIGH)
        GPIO.output(self.__RightBack, GPIO.LOW)

        self.LeftSpeed.ChangeDutyCycle(speed)
        self.RightSpeed.ChangeDutyCycle(speed)

    def left(self, speed=default_speed):

        GPIO.output(self.__LeftAhead, GPIO.LOW)
        GPIO.output(self.__LeftBack, GPIO.LOW)

        GPIO.output(self.__RightAhead, GPIO.HIGH)
        GPIO.output(self.__RightBack, GPIO.LOW)

        self.LeftSpeed.ChangeDutyCycle(speed)
        self.RightSpeed.ChangeDutyCycle(speed)

    def right(self, speed=default_speed):

        GPIO.output(self.__LeftAhead, GPIO.HIGH)
        GPIO.output(self.__LeftBack, GPIO.LOW)

        GPIO.output(self.__RightAhead, GPIO.LOW)
        GPIO.output(self.__RightBack, GPIO.LOW)

        self.LeftSpeed.ChangeDutyCycle(speed)
        self.RightSpeed.ChangeDutyCycle(speed)

    def rear(self, speed=default_speed):
        GPIO.output(self.__LeftAhead, GPIO.LOW)
        GPIO.output(self.__LeftBack, GPIO.HIGH)

        GPIO.output(self.__RightAhead, GPIO.LOW)
        GPIO.output(self.__RightBack, GPIO.HIGH)

        self.LeftSpeed.ChangeDutyCycle(speed)
        self.RightSpeed.ChangeDutyCycle(speed)

    # 小车原地左转
    def spin_left(self, speed=default_speed):

        GPIO.output(self.__LeftAhead, GPIO.LOW)
        GPIO.output(self.__LeftBack, GPIO.HIGH)

        GPIO.output(self.__RightAhead, GPIO.HIGH)
        GPIO.output(self.__RightBack, GPIO.LOW)

        self.LeftSpeed.ChangeDutyCycle(speed)
        self.RightSpeed.ChangeDutyCycle(speed)

    # 小车原地右转
    def spin_right(self, speed=default_speed):

        GPIO.output(self.__LeftAhead, GPIO.HIGH)
        GPIO.output(self.__LeftBack, GPIO.LOW)

        GPIO.output(self.__RightAhead, GPIO.LOW)
        GPIO.output(self.__RightBack, GPIO.HIGH)

        self.LeftSpeed.ChangeDutyCycle(speed)
        self.RightSpeed.ChangeDutyCycle(speed)

    # 差速转
    def dif_left(self, speed=default_speed, difrate=default_difrate):

        GPIO.output(self.__LeftAhead, GPIO.HIGH)
        GPIO.output(self.__LeftBack, GPIO.LOW)

        GPIO.output(self.__RightAhead, GPIO.HIGH)
        GPIO.output(self.__RightBack, GPIO.LOW)

        self.LeftSpeed.ChangeDutyCycle(speed*difrate)
        self.RightSpeed.ChangeDutyCycle(speed)

    def dif_right(self, speed=default_speed, difrate=default_difrate):

        GPIO.output(self.__LeftAhead, GPIO.HIGH)
        GPIO.output(self.__LeftBack, GPIO.LOW)

        GPIO.output(self.__RightAhead, GPIO.HIGH)
        GPIO.output(self.__RightBack, GPIO.LOW)

        self.LeftSpeed.ChangeDutyCycle(speed)
        self.RightSpeed.ChangeDutyCycle(speed*difrate)

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
        for pin in self.__inx_pin:
            GPIO.output(pin, GPIO.LOW)
        self.LeftSpeed.ChangeDutyCycle(0)
        self.RightSpeed.ChangeDutyCycle(0)

    def cleanup(self):
        for pin in self.__inx_pin:
            GPIO.cleanup(pin)
        for pin in self.__inx_pin:
            GPIO.cleanup(pin)
        print("Close Motor")
