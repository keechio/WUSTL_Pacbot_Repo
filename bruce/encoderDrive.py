# Importing modules and classes
import time
import numpy as np
from gpiozero import RotaryEncoder
from motorControl import motorControl

inA = 17
inB = 27
pwmA = 22

# Assigning parameter values
ppr = 260  # Pulses Per Revolution of the encoder
tsample = 0.02  # Sampling period for code execution (s)

# Creating encoder object using GPIO pins 24 and 25
encoder = RotaryEncoder(17, 27, max_steps=0)

motor = motorControl()

# Initializing previous values and starting main clock
anglecurr = 0
tprev = 0
tcurr = 0
tstart = time.perf_counter()

angle_target = -1000


while anglecurr >= angle_target:
    # Pausing for `tsample` to give CPU time to process encoder signal
    time.sleep(tsample)
    # Getting current time (s)
    tcurr = time.perf_counter() - tstart
    # Getting angular position of the encoder
    # roughly every `tsample` seconds (deg.)
    anglecurr = 360 / ppr * encoder.steps
    print("Angle = {:0.0f} deg".format(anglecurr))
    # Updating previous values
    tprev = tcurr

    motor.drive(0.4, 0.4)

print('Done.')
# Releasing GPIO pins
encoder.close()