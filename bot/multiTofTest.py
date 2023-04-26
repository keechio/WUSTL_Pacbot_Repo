# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

# This example shows using two TSL2491 light sensors attached to TCA9548A channels 0 and 1.
# Use with other I2C sensors would be similar.
import time
import board
import os
import adafruit_vl53l0x
import adafruit_tca9548a

# Create I2C bus as normal

i2c = board.I2C()  # uses board.SCL and board.SDA

# Create the TCA9548A object and give it the I2C bus
tca = adafruit_tca9548a.TCA9548A(i2c)

# For each sensor, create it using the TCA9548A channel instead of the I2C object
dis0 = adafruit_vl53l0x.VL53L0X(tca[0])
dis1 = adafruit_vl53l0x.VL53L0X(tca[1])
dis2 = adafruit_vl53l0x.VL53L0X(tca[2])
dis3 = adafruit_vl53l0x.VL53L0X(tca[3])
dis4 = adafruit_vl53l0x.VL53L0X(tca[4])
dis5 = adafruit_vl53l0x.VL53L0X(tca[5])

cnt = 0
# After initial setup, can just use sensors as normal.
while True:

    # print(dis0.range, dis1.range, dis2.range)
    # time.sleep(0.1)

    range0 = dis0.range # left sensors
    range1 = dis1.range # 

    range2 = dis2.range*0.6 # front sensors
    range3 = dis3.range*0.7-19

    range4 = dis4.range # 
    range5 = dis5.range # right sensors


    # Scale the range readings to fit within a 40-character width
    width = 40
    scaled0 = int(range0 / 1000 * width)
    scaled1 = int(range1 / 1000 * width)
    scaled2 = int(range2 / 1000 * width)
    scaled3 = int(range3 / 1000 * width)
    scaled4 = int(range4 / 1000 * width)
    scaled5 = int(range5 / 1000 * width)

    

    # Create a bar graph using ASCII art to represent the range readings
    bar0 = "=" * scaled0 + " " * (width - scaled0)
    bar1 = "=" * scaled1 + " " * (width - scaled1)
    bar2 = "=" * scaled2 + " " * (width - scaled2)
    bar3 = "=" * scaled3 + " " * (width - scaled3)
    bar4 = "=" * scaled4 + " " * (width - scaled4)
    bar5 = "=" * scaled5 + " " * (width - scaled5)

    # Print the bar graph for each sensor
    print(f"Sensor 0 [{bar0}] {range0} mm")
    print(f"Sensor 1 [{bar1}] {range1} mm")
    print(f"Sensor 2 [{bar2}] {range2} mm")
    print(f"Sensor 3 [{bar3}] {range3} mm")
    print(f"Sensor 4 [{bar4}] {range4} mm")
    print(f"Sensor 5 [{bar5}] {range5} mm")



    # Wait for a short period of time before printing the next set of readings
    time.sleep(0.02)
    cnt += 1
