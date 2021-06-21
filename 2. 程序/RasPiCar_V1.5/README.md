<!--
 * @Author: Ken Kaneki
 * @Date: 2021-06-03 00:57:43
 * @LastEditTime: 2021-06-03 22:54:18
 * @Description: README
 * @FilePath: \undefinedd:\Learn\毕设资料\终期答辩\2. 程序\RasPiCar_V1.5\README.md
-->
# RaspberrryPi-Project-for-PiCar
    This is the program running in RaspberryPi
    The main.py is raspicar.py
```
python3 raspicar.py
```

# GPIO BCM

## Motor
    ENA->GPIO_16
    IN1->GPIO_20
    IN2->GPIO_21
    IN3->GPIO_19
    IN4->GPIO_26
    ENB->GPIO_13

## Steer
    YAW->GPIO_27
    Pitch->GPIO_17


## RGB
    Red->GPIO_18
    Green->GPIO_23
    Blue->GPIO_24

## HC-SR04
    YAW->GPIO_22
    Trig->GPIO_6
    Echo->GPIO_5

# Control

## Chassis
    W: forward
    S: Backward
    A: Turn Left [+Ctrl: spin left]
    D: Turn Right [+Ctrl: spin right]

## Gimbal
    <-: Left
    ->: Right
    up: Up
    down: Down
    0[ins]: init

## HC-SR04
    4[Num]: Left
    6[Num]: Right
    5[Num]: init

## main control
    space: start
    Esc: close
    End: close&shutdown
    Home: close&reboot
    1: Manualmodel
    2: Automatic-Avoidance
    3: Automatic-Trackline
# Model
## Motor
    底盘驱动代码，实现底盘的前进、后退、左转、右转、巡线等功能
## Steering
    舵机驱动代码，实现舵机的归中、左转、右转等功能
## Avoidance
    RGB灯和超声波驱动代码，实现自主避障、RGB灯人机交互等功能
## PiCar
    多线程、多进程代码，使用基础模块实现多线程控制各个模块，主要包括底盘线程、云台线程、探测线程、自主巡线进程
## PiCarService
    开机自启动服务，用于实现树莓派开机自动运行机器人控制和预测程序，自启动步骤详见
`/3. 环境/2. 树莓派环境/开机配置.docx`
## raspicar.py
    主控程序，根目录下运行
`python3 raspicar.py`即可运行机器人控制程序
