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
power = 0.4
left = power
right = power
motor.drive(left, right)
time.sleep(2)
print("Gyro: ", gyro.get_z_cumulative())
deviation = gyro.get_z_cumulative()
reverse = True
motor.stop()
time.sleep(0.5)
while abs(deviation) > 0.01:
    gyro.reset()
    left = power - deviation / 180 / 5
    right = power + deviation / 180 / 5
    motor.drive(left, right, reverse)
    for i in range(20):
        print("Gyro: ", gyro.get_z_cumulative())
        time.sleep(.1)
    print("Gyro: ", gyro.get_z_cumulative())
    deviation = gyro.get_z_cumulative()
    motor.stop()
    time.sleep(0.5)


with open('calibrated.txt', 'w') as f:
    f.write(str(left) + " " + str(right))
