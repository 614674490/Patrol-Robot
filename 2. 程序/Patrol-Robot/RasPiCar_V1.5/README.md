# RaspberrryPi-Project-for-PiCar
This is the program running in RaspberryPi

#GPIO BCM

## Motor
    左前-L298N_1
    ENA->GPIO_16
    IN1->GPIO_20
    IN2->GPIO_21

    左后-L298N_2
    ENB->GPIO_8
    IN3->GPIO_7
    IN4->GPIO_1

    右前-L298N_1
    IN3->GPIO_19
    IN4->GPIO_26
    ENB->GPIO_13

    右后-L298N2
    IN1->GPIO_9
    IN2->GPIO_11
    ENA->GPIO_10

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
    esc: close
    1: turn manual/automatic
