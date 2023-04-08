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
dis0 = adafruit_vl53l0x.VL53L0X(tca[0])
dis1 = adafruit_vl53l0x.VL53L0X(tca[1])
dis2 = adafruit_vl53l0x.VL53L0X(tca[2])

gyro = Gyro()
motor = motorControl()

# motor.spin_left(0.1,90)

# exit()

# After initial setup, can just use sensors as normal.
while True:

    gyro_angle = gyro.get_z_cumulative()

    range0 = dis0.range
    range1 = dis1.range
    range2 = dis2.range

    # Scale the range readings to fit within a 40-character width
    width = 40
    scaled0 = int(range0 / 1000 * width)
    scaled1 = int(range1 / 1000 * width)
    scaled2 = int(range2 / 1000 * width)

    # Create a bar graph using ASCII art to represent the range readings
    bar0 = "=" * scaled0 + " " * (width - scaled0)
    bar1 = "=" * scaled1 + " " * (width - scaled1)
    bar2 = "=" * scaled2 + " " * (width - scaled2)

    # # Print the bar graph for each sensor
    # print(f"Sensor 0 [{bar0}] {range0} mm")
    # print(f"Sensor 1 [{bar1}] {range1} mm")
    # print(f"Sensor 2 [{bar2}] {range2} mm")

        # go st
        # raight when gyro_z_cumulative is 0
    delta = 0.03
    rangeDiff = range0 - range2
    print(gyro_angle)
    if gyro_angle < -2:
    # if rangeDiff > 20:
        motor.drive(0.2, 0.2 - delta)
    elif gyro_angle > 2:
    # elif rangeDiff < -20:
        motor.drive(0.2 - delta, 0.2)
    else:
        motor.drive(0.2, 0.2)

    if range2 < 60:
        motor.stop()
        
        motor.drive(0, 0.2, direction=False)
        time.sleep(0.15)
        motor.drive(0.2, 0.2)
        time.sleep(0.15)
        gyro.reset()
        gyro_angle = 0
    elif range0 < 60:
        motor.stop()
        motor.drive(0.2, 0, direction=False)
        time.sleep(0.15)
        motor.drive(0.2, 0.2)
        time.sleep(0.15)
        gyro.reset()
        gyro_angle = 0


    if range1 < 120:
        motor.stop()
        print("WALL! STOP")
        # motor.drive(0.2, 0.2, False)
        # print("BACKING UP")
        time.sleep(1)
        if range0 < range2:
            print("OK TO GO RIGHT")
            # motor.spin_right(0.1, 90)
        elif range0 > range2:
            print("OK TO GO LEFT")
            # motor.spin_left(0.1, 90)
        elif range0 > 200 and range2 > 200:
            print("T TURN")

        # wait for user/server command
        



    # Wait for a short period of time before printing the next set of readings
    time.sleep(0.02)
