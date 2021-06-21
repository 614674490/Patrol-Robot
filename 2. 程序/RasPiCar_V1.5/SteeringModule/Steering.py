# -*-coding:utf-8-*-
'''
Author: Ken Kaneki
Date: 2021-05-28 16:30:08
LastEditTime: 2021-05-28 16:34:03
Description: README
FilePath: \RasPiCar_V1.6\SteeringModule\Steering.py
'''
from SteeringModule.Rotation import Rotation

default_channelP = 17
default_channelY = 27
default_dt = 0.02


class Steering:
    def __init__(self, channelP=default_channelP, min_thetaP=60, max_thetaP=180, init_thetaP=120, channelY=default_channelY, min_thetaY=0, max_thetaY=180, init_thetaY=86):
        self.p = Rotation(channelP, min_thetaP, max_thetaP,
                          init_thetaP, delta_theta=0.01)

        self.y = Rotation(channelY, min_thetaY, max_thetaY,
                          init_thetaY, delta_theta=2.0)

    def setup(self):
        self.y.setup()
        self.p.setup()

    def Up(self):
        self.p.positiveRotation()

    def Down(self):
        self.p.reverseRotation()

    def Left(self, dt=default_dt):
        self.y.positiveRotation(dt)

    def Right(self, dt=default_dt):
        self.y.reverseRotation(dt)

    def specify(self, thetaY, thetaP):
        self.y.specifyRotation(thetaY)
        self.p.specifyRotation(thetaP)

    def stop(self):
        self.y.stop()
        self.p.stop()

    def clear(self):
        self.y.clear()
        self.p.clear()

    def cleanup(self):
        self.y.cleanup()
        self.p.cleanup()
