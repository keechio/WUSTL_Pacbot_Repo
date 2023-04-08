# Importing modules and classes
import time
from gpiozero import RotaryEncoder

class Encoder:
    def __init__(self, inA, inB, ppr):
        self.inA = inA
        self.inB = inB
        self.ppr = ppr
        self.encoder = RotaryEncoder(inA, inB, max_steps=0)
        self.anglecurr = 0
        self.tprev = 0
        self.tcurr = 0
        self.tstart = time.perf_counter()
        self.tsample = 0.02

    def getAngle(self):
        # Pausing for `tsample` to give CPU time to process encoder signal
        time.sleep(self.tsample)
        # Getting current time (s)
        self.tcurr = time.perf_counter() - self.tstart
        # Getting angular position of the encoder
        # roughly every `tsample` seconds (deg.)
        self.anglecurr = 360 / self.ppr * self.encoder.steps
        # Updating previous values
        self.tprev = self.tcurr
        return self.anglecurr
