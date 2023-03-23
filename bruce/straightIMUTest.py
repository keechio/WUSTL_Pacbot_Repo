import time
import board
from motorControl import motorControl
from gyro import Gyro
from gpiozero import DistanceSensor

i2c = board.I2C()
gyro = Gyro()
motor = motorControl()

distanceSensor = DistanceSensor(echo=25, trigger=9)

gyro.reset()

while True:
    gyro_angle = gyro.get_z_cumulative()

    print("Gyro: ", gyro_angle)
    
    # go straight when gyro_z_cumulative is 0
    if gyro_angle < -1:
        motor.drive(0.4, 0.3)
    elif gyro_angle > 1:
        motor.drive(0.3, 0.4)
    else:
        motor.drive(0.4, 0.4)

    # # when distance < 9, print "turn left"
    # if distanceSensor.distance * 100 > 9:
    #     motor.spin_left(0.2)
    #     time.sleep(1)
    #     break
    time.sleep(0.1)

