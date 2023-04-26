# TODO: add reverse command in running state
#       use new sensor to stop at middle of intersection
#       remember starting orientation & maintain current orientation
#       change user/server command from NEWS to left/right/reverse directions
#       logic for converting NEWS to left/right/reverse
#           ignoring invalid commands
#           case (reverse) if command is opposite of current direction
#           case (left/right) if command is left/right of current direction (e.g. N->E means right)
#           case (ignore) if command same as current direction or invalid


# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

# This example shows using two TSL2491 light sensors attached to TCA9548A channels 0 and 1.
# Use with other I2C sensors would be similar.
import time
import board
import os
import adafruit_vl53l0x
import adafruit_tca9548a
from lib.gyro import Gyro
from lib.motorControl import motorControl

import asyncio

# Create I2C bus as normal

i2c = board.I2C()  # uses board.SCL and board.SDA

# Create the TCA9548A object and give it the I2C bus
tca = adafruit_tca9548a.TCA9548A(i2c)

# For each sensor, create it using the TCA9548A channel instead of the I2C object
#dis0 = adafruit_vl53l0x.VL53L0X(tca[0])
dis1 = adafruit_vl53l0x.VL53L0X(tca[1])
dis2 = adafruit_vl53l0x.VL53L0X(tca[2])

dis3 = adafruit_vl53l0x.VL53L0X(tca[3])
# dis4 = adafruit_vl53l0x.VL53L0X(tca[4])
dis5 = adafruit_vl53l0x.VL53L0X(tca[5])

gyro = Gyro()
motor = motorControl()

# motor.spin_left(0.1,90)

# exit()

# After initial setup, can just use sensors as normal.
while True:

    gyro_angle = gyro.get_z_cumulative()

    #range0 = dis0.range # left sensors
    range1 = dis1.range # 

    range2 = dis2.range*0.6 # front sensors
    range3 = dis3.range*0.7-19

    #range4 = dis4.range # 
    range4 = dis5.range # right sensors




    delta = 0.03
    #rangeDiff = range0 - range2
    # print(gyro_angle)
    if gyro_angle < -1:
    # if rangeDiff > 20:
        motor.drive(0.2-delta,0.2)
    elif gyro_angle > 1:
    # elif rangeDiff < -20:
        motor.drive(0.2, 0.2-delta)
        print('angle')
    else:
        motor.drive(0.2, 0.2)
    
    if range4 < 60 :
        motor.stop()
        # motor.drive(0.2, 0, direction=False)
        motor.spin_left(0.2, 10)
        time.sleep(0.1)
        motor.drive(0.2, 0.2)
        time.sleep(0.1)
        gyro.reset()
        gyro_angle = 0
        print("right front hella close")

    if range4 < 70 :
        motor.stop()
        motor.drive(0.2, 0, direction=False)
        # motor.spin_left(0.1, 5)
        time.sleep(0.1)
        motor.drive(0.2, 0.2)
        time.sleep(0.1)
        gyro.reset()
        gyro_angle = 0
        print("right front too close")

    elif range1 < 100:
        motor.stop()
        # motor.drive(0, 0.2, direction=False)
        motor.spin_right(0.2, 10)
        time.sleep(0.15)
        motor.drive(0.2, 0.2)
        time.sleep(0.1)
        gyro.reset()
        gyro_angle = 0
        print("left front hella close")

    elif range1 < 110:
        motor.stop()
        motor.drive(0, 0.2, direction=False)
        # motor.spin_right(0.1, 5)
        time.sleep(0.15)
        motor.drive(0.2, 0.2)
        time.sleep(0.1)
        gyro.reset()
        gyro_angle = 0
        print("left front too close")


    # elif range0-range1>18:
    #     motor.stop()
    #     motor.drive(0.2, 0, direction=False)
    #     time.sleep(0.15)
    #     motor.drive(0.2, 0.2)
    #     time.sleep(0.15)
    #     gyro.reset()
    #     gyro_angle = 0

    #     print("too left")

    # elif range0-range1 < -18:
    #     motor.stop()
    #     motor.drive(0, 0.2, direction=False)
    #     time.sleep(0.15)
    #     motor.drive(0.2, 0.2)
    #     time.sleep(0.15)
    #     gyro.reset()
    #     gyro_angle = 0
    #     print("too right")

    frontAvg = (range2 + range3) / 2

    # if wall is detected, stop and wait for user/server command
    if frontAvg < 70:
        motor.stop()
        motor.restDir()
        print("WALL! STOP")
        # motor.drive(0.2, 0.2, False)
        # print("BACKING UP")
        time.sleep(1)
        if range1 < range4:
            print("OK TO GO RIGHT")
            # motor.spin_right(0.1, 90)
        elif range1 > range4:
            print("OK TO GO LEFT")
            # motor.spin_left(0.1, 90)
        elif range1 > 200 and range4 > 200:
            print("T TURN")

        # wait for user/server command
        command = input("Enter command: ")
        if command == "a":
            # left
            motor.spin_left(0.2, 79)
        elif command == "d":
            # right
            motor.spin_right(0.2, 79)
        elif command == "r":
            # turn & reverse
            motor.spin_right(0.2, 165)

        motor.drive(0.2, 0.2)
        time.sleep(0.5)
        gyro.reset()
        gyro_angle = 0
    
    # if at intersection, stop and wait for user/server command
    if (range1 > 180  or range4 > 160) and frontAvg > 200:
        print(frontAvg)
        # motor.drive(0.2, 0.2)
        time.sleep(0.42)
        motor.restDir()
        motor.stop()
        print("INTERSECTION! STOP")
        # motor.drive(0.2, 0.2, False)
        # print("BACKING UP")
        # time.sleep(1)
        if range1 < range4:
            print("OK TO GO RIGHT")
            # motor.spin_right(0.1, 90)
        elif range1 > range4:
            print("OK TO GO LEFT")
            # motor.spin_left(0.1, 90)
        elif range1 > 200 and range4 > 200:
            print("X CROSSING")

        # wait for user/server command
        command = input("Enter command: ")
        if command == "a":
            # left
            motor.spin_left(0.2, 81)
        elif command == "d":
            # right
            motor.spin_right(0.2, 81)
        elif command == "r":
            # turn & reverse
            motor.spin_right(0.2, 165)
        elif command == "w":
            # go straight
            print("ahhahaah")

            
        motor.drive(0.2, 0.2)
        time.sleep(0.5)
        gyro.reset()
        gyro_angle = 0




    # Wait for a short period of time before printing the next set of readings
    time.sleep(0.05)
