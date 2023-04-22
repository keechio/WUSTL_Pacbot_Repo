# Importing modules and classes
import time
from gpiozero import RotaryEncoder
from lib.motorControl import motorControl
import math

inA = 17
inB = 27

# Assigning parameter values
ppr = 180  # Pulses Per Revolution of the encoder
tsample = 0.02  # Sampling period for code execution (s)

# Creating encoder object using GPIO pins 17 and 27
encoder = RotaryEncoder(17, 27, max_steps=0)
motor = motorControl()

wheel_diameter = 1.575

def degToInches(deg):
    return deg * wheel_diameter / 180

def inchToDeg(inch):
    return inch / wheel_diameter * 180

# Initializing previous values and starting main clock
anglecurr = 0
tprev = 0
tcurr = 0
tstart = time.perf_counter()
angle_target = -inchToDeg(3)
max_power = 0.8
min_power = 0.2



while anglecurr > angle_target:
    # Pausing for `tsample` to give CPU time to process encoder signal
    time.sleep(tsample)
    # Getting current time (s)
    tcurr = time.perf_counter() - tstart
    # Getting angular position of the encoder
    # roughly every `tsample` seconds (deg.)
    anglecurr = 360 / ppr * encoder.steps
    print("Angle = {:0.0f} deg".format(anglecurr) + ", Distance = {:0.1f} in.".format(degToInches(anglecurr)))
    # Updating previous values
    tprev = tcurr
    slow_angle = 200
    reduction = 100
    # Power follows logistic curve between max to min power based on distance until targe angle
    power = min_power + (max_power-min_power)/(1 + slow_angle*math.exp(-1/reduction*(abs(angle_target-anglecurr)-slow_angle)))
    print(power)
    motor.drive(power, power)
motor.stop()
while anglecurr < angle_target:
    # Pausing for `tsample` to give CPU time to process encoder signal
    time.sleep(tsample)
    # Getting current time (s)
    tcurr = time.perf_counter() - tstart
    # Getting angular position of the encoder
    # roughly every `tsample` seconds (deg.)
    anglecurr = 360 / ppr * encoder.steps
    print("Angle = {:0.0f} deg".format(anglecurr) + ", Distance = {:0.1f} in.".format(degToInches(anglecurr)))
    motor.drive(min_power, min_power, direction=False)
motor.stop()
print('Done.')
# Releasing GPIO pins
encoder.close()