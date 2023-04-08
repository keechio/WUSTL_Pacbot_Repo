from bruce.lib.motorControl import motorControl
from time import sleep

motor = motorControl()

motor.set_speed_direction(0.1,0)
# go forward
motor.drive(0.5,0.5)
sleep(1)

# go left 90 ˚
motor.drive(0.38, 0)
sleep(0.5)

# turn right 90 ˚
motor.drive(0,0.34)
sleep(0.5)

motor.drive(0.5,0.5)
sleep(0.2)

