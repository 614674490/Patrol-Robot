# -*-coding:utf-8-*-
'''
Author: Ken Kaneki
Date: 2021-05-28 16:30:08
LastEditTime: 2021-05-28 16:33:55
Description: README
FilePath: \RasPiCar_V1.6\SteeringModule\Rotation.py
'''
import time

import RPi.GPIO as GPIO


class Rotation:
    '''This class represent a SG90 module'''
    __frequency = 50  # Impulse frequency(HZ)
    delta_theta = 0.5  # Rotation interval(degree)0.2 0.02 | 0.01 0
    min_delay = 0.02  # The theoretical time to rotate delta_theta(s)
    max_delay = 0.1  # The time to rotate from 0 to 180(s)

    def __init__(self, channel, min_theta, max_theta, init_theta=90, delta_theta=0.5):
        self.__channel = channel  # Control pin
        self.__delta_theta = delta_theta
        if(min_theta < 0 or min_theta > 180):
            self.__min_theta = 0
        else:
            self.__min_theta = min_theta
        if(max_theta < 0 or max_theta > 180):
            self.__max_theta = 180
        else:
            self.__max_theta = max_theta
        if(init_theta < min_theta or init_theta > max_theta):
            self.__init_theta = (self.__min_theta+self.__max_theta)/2
        else:
            self.__init_theta = init_theta  # Initial theta
            self.__min_dutycycle = 2.5+self.__min_theta*10/180
            self.__max_dutycycle = 2.5+self.__max_theta*10/180
        self.setup()
        self.clear()

    def setup(self):
        '''Init'''
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.__channel, GPIO.OUT)
        self.__pwm = GPIO.PWM(self.__channel, Rotation.__frequency)  # PWM
        self.__pwm.start(0)

    def specifyRotation(self, theta):
        '''Rotate to specify theta'''
        if(theta < self.__min_theta or theta > self.__max_theta):
            return
        self.__dutycycle = 2.5+theta*10/180
        self.__pwm.ChangeDutyCycle(self.__dutycycle)
        time.sleep(Rotation.max_delay)
        self.__pwm.ChangeDutyCycle(0)  # 归零信号

    def positiveRotation(self,delta_theta=0.5):
        '''Positive rotation，rotating delta_theta each invoking'''
        self.__dutycycle = self.__dutycycle+delta_theta*10/180
        if self.__dutycycle > self.__max_dutycycle:
            self.__dutycycle = self.__max_dutycycle
        self.__pwm.ChangeDutyCycle(self.__dutycycle)
        # time.sleep(Rotation.min_delay)
        # self.__pwm.ChangeDutyCycle(0)

    def reverseRotation(self,delta_theta=0.5):
        '''Reverse rotation，rotating delta_theta each invoking'''
        self.__dutycycle = self.__dutycycle-delta_theta*10/180
        if self.__dutycycle < self.__min_dutycycle:
            self.__dutycycle = self.__min_dutycycle
        self.__pwm.ChangeDutyCycle(self.__dutycycle)
        # time.sleep(Rotation.min_delay)
        # self.__pwm.ChangeDutyCycle(0)

    def clear(self):
        for i in range(5):
            self.specifyRotation(self.__init_theta)

    def stop(self):
        self.__pwm.ChangeDutyCycle(0)

    def cleanup(self):
        self.__pwm.stop()
        time.sleep(Rotation.min_delay)
        GPIO.cleanup(self.__channel)
        print('Close Servo')
