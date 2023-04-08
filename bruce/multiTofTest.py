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

# After initial setup, can just use sensors as normal.
while True:

    # print(dis0.range, dis1.range, dis2.range)
    # time.sleep(0.1)

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

    # Print the bar graph for each sensor
    print(f"Sensor 0 [{bar0}] {range0} mm")
    print(f"Sensor 1 [{bar1}] {range1} mm")
    print(f"Sensor 2 [{bar2}] {range2} mm")



    # Wait for a short period of time before printing the next set of readings
    time.sleep(0.02)
