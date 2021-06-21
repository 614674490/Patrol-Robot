# -*-coding:utf-8-*-
import sys
from time import sleep

from PiCarMoudle.PiCar import *

control = Thread_Control(1)
control.start()

chassis = Thread_Motor(2)
chassis.start()

gimbal = Thread_Steer(3)
gimbal.start()

dectect = Thread_Dectect(4)
dectect.start()

Shared.rgb.led(Shared.rgb.Color.Blue)
print('THread Start OK')

'''
autoprocess = AutoModeProcess()
autoprocess.start()
print('Process Start OK')
'''

try:
    while Shared.FLAG == Shared.Mode['Wait']:
        pass
    while Shared.FLAG > 0:
        sleep(0.01)
        if Shared.FLAG == Shared.Mode['Manual']:
            Shared.rgb.led(Shared.rgb.Color.Green)
        elif Shared.FLAG == Shared.Mode['Automatic-Avoidance']:
            Shared.rgb.led(Shared.rgb.Color.Yellow)
        elif Shared.FLAG == Shared.Mode['Automatic-Trackline']:
            Shared.rgb.led(Shared.rgb.Color.Cyan)

except KeyboardInterrupt:
    print('Close PiCar System')
    Shared.FLAG = Shared.Mode['End']
    data.set_Flag()
    Shared.cleanup()
    pygame.quit()
    sys.exit()
finally:
    dectect.stop()
    gimbal.stop()
    chassis.stop()
    control.stop()
    print('System Ended')
