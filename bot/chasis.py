# test the motors

from lib.motorControl import motorControl

motor = motorControl()


while True:
    motor.drive(0.2, 0.2)