# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

# Simple demo of the VL53L0X distance sensor.
# Will print the sensed range/distance every second.
import time

import board
import busio

import adafruit_vl53l0x
import adafruit_tca9548a


# Initialize I2C bus and sensor.
i2c = busio.I2C(board.SCL, board.SDA)



vl53 = adafruit_vl53l0x.VL53L0X(i2c)

from gpiozero import PWMOutputDevice as PWM
from gpiozero import DigitalOutputDevice as DOD
from time import sleep

# Define the pins for the motor controller

# in1 in2 is for motor direction
AIN1_PIN = 26
AIN2_PIN = 20
PWMA_PIN = 12

BIN1_PIN = 16
BIN2_PIN = 19
PWMB_PIN = 13

# set pin mode
AIN1 = DOD(AIN1_PIN)
AIN2 = DOD(AIN2_PIN)
PWMA = PWM(PWMA_PIN)

BIN1 = DOD(BIN1_PIN)
BIN2 = DOD(BIN2_PIN)
PWMB = PWM(PWMB_PIN)

# set motor direction, in1 high, in2 low
AIN1.on()
AIN2.off()
BIN1.on()
BIN2.off()

prevDistance = 100
while True:
    print('Distance: ', vl53.range)
    
    # when distance < 9, turn left
    # if vl53.range - prevDistance < 0:
    #     PWMA.value = 0.12
    #     PWMB.value = 0.1
    # elif vl53.range - prevDistance > 0:
    #     PWMA.value = 0.1
    #     PWMB.value = 0.12
    # else:
    #     PWMA.value = 0.20
    #     PWMB.value = 0.20

    # prevDistance = vl53.range

    sleep(.3)

# stop motor
PWMA.value = 0
PWMB.value = 0