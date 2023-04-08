import time
import board
from lib.motorControl import motorControl
from lib.gyro import Gyro
from gpiozero import DistanceSensor

i2c = board.I2C()
gyro = Gyro()
motor = motorControl()

distanceSensor = DistanceSensor(echo=25, trigger=9)

gyro.reset()
gyro_angle = gyro.get_z_cumulative()
left = 0.4
right = 0.4
motor.drive(left, right)
time.sleep(2)
print("Gyro: ", gyro_angle)
deviation = gyro_angle
reverse = -1
while abs(deviation) > 0.01:
    gyro_angle = gyro.get_z_cumulative()
    left = 0.4 - deviation * gyro_angle / 1800
    right = 0.4 + deviation * gyro_angle / 1800
    motor.drive(reverse * left, reverse * right)
    time.sleep(2)
    print("Gyro: ", gyro_angle)
    deviation = gyro_angle
    reverse = -1 * reverse


with open('calibrated.txt', 'w') as f:
    f.write(str(left) + " " + str(right))
