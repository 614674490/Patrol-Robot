# RaspberrryPi-Project-for-PiCar
This is the program running in RaspberryPi

#GPIO BCM

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
    esc: close
    1: turn manual/automatic
