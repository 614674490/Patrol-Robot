# -*-coding:utf-8-*-
from SteeringModule.Rotation import Rotation
import RPi.GPIO as GPIO
import time
import sys
sys.path.append("..")

# pin
default_trigger_pin = 6
default_echo_pin = 5
default_servo_pin = 22
default_r_pin = 18
default_g_pin = 23
default_b_pin = 24
default_init_theta = 86
default_dt = 0.08
detect_dt = 2.0
min_dis = 20.0
max_dis = 140.0


class HC_SR04:

    def __init__(self, trigger_pin=default_trigger_pin, echo_pin=default_echo_pin, servo_pin=default_servo_pin, init_theta=default_init_theta, delta_theta=default_dt):

        self.__trigger = trigger_pin
        self.__echo = echo_pin
        self.__distance = 0.000
        self.servo = Rotation(servo_pin, 0, 180, init_theta, delta_theta)
        self.setup()

    def setup(self):

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        GPIO.setup(self.__trigger, GPIO.OUT)
        GPIO.setup(self.__echo, GPIO.IN)

    '''
    TRIG 负责发射超声波，Echo 负责接收超声波
    '''

    def __send_trigger_pulse(self):
        # 发送超声波，一直发
        GPIO.output(self.__trigger, 1)
        # 为了防止错误，因为紧接着就需要把发射端置为高电平
        time.sleep(0.0001)
        # 发射端置为高电平
        GPIO.output(self.__trigger, 0)

    '''
    ECHO 负责接收超声波
    '''

    def __wait_for_echo(self, value, timeout):

        count = timeout
        # 通过该代码持续获取ECHO的状态
        while GPIO.input(self.__echo) != value and count > 0:
            count = count-1

    '''
    计算距离
    '''

    def get_distance(self):
        # 发射
        self.__send_trigger_pulse()
        # 接收高电平 1/True
        self.__wait_for_echo(True, 10000)
        # 等待
        start = time.time()
        # 接收低电平
        self.__wait_for_echo(False, 10000)
        finish = time.time()
        pulse_len = finish-start
        distance_cm = (pulse_len/0.000058)
        self.__distance = round(distance_cm, 3)
        # print(self.__distance)
        #self.__distance=('%.3f' % distance_cm)
        return self.__distance

    def dectect_block(self):

        count = 0
        # 0->block 1->no block
        for i in range(5):
            distance = self.get_distance()
            if distance > min_dis and distance < max_dis:
                count = count+1
            else:
                count = 0
                break
        if(count < 5):
            return 0
        else:
            return 1

    def right_detect(self):
        outway_flag = 0
        turn_flag = 0
        for i in range(90):
            self.Right(detect_dt)
            time.sleep(self.servo.min_delay)
            self.servo.stop()
            if self.dectect_block():
                turn_flag = 1
                outway_flag = 1
                #print('find way turn right')
                break
        return turn_flag, outway_flag

    def left_detect(self):
        outway_flag = 0
        turn_flag = 1
        for i in range(90):
            self.Left(detect_dt)
            time.sleep(self.servo.min_delay)
            self.servo.stop()
            if self.dectect_block():
                turn_flag = 0
                outway_flag = 1
                #print('find way turn left')
                break
        return turn_flag, outway_flag

    def Left(self, dt=default_dt):
        self.servo.positiveRotation(dt)

    def Right(self, dt=default_dt):
        self.servo.reverseRotation(dt)

    def specify(self, theta):
        self.servo.specifyRotation(theta)

    def cleanup(self):

        self.servo.cleanup()
        GPIO.cleanup(self.__trigger)
        GPIO.cleanup(self.__echo)
        print('Close HC-SR04')


class RGB:

    class Color:

        Red = [100, 0, 0]  # 红色: PowerOn start init
        Cyan = [0, 100, 100]  # 青色: I/O Init OK
        Blue = [0, 0, 100]  # 蓝色: Thread Start OK
        Green = [0, 100, 0]  # 绿色: Manual Control Mode
        Yellow = [100, 100, 0]  # 黄色: Automatic Control Mode
        Purple = [100, 0, 100]  # 紫色: detected object
        Black = [0, 0, 0]

    def __init__(self, r_pin=default_r_pin, g_pin=default_g_pin, b_pin=default_b_pin):

        self.__RGB_pin = [r_pin, g_pin, b_pin]
        self.__pwm = [0, 0, 0]
        self.setup()

    def setup(self):

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        i = 0
        for pin in self.__RGB_pin:
            GPIO.setup(pin, GPIO.OUT)
            self.__pwm[i] = GPIO.PWM(pin, 50)
            self.__pwm[i].start(0)
            i = i+1

    ''' RGB_LED Tip: power on->Red->Blue->Cyan->Green  +Yellow'''

    def led(self, color):

        i = 0
        for pwm in self.__pwm:
            pwm.ChangeDutyCycle(color[i])
            i = i+1

    def close(self):
        # close led
        self.led(RGB.Color.Black)

    def cleanup(self):

        for pwm in self.__pwm:
            pwm.stop()
        for pin in self.__RGB_pin:
            GPIO.cleanup(pin)
        print('Close RGB')

