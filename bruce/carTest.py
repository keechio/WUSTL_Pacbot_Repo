#    C2Plabs.com 
#
#

from gpiozero import PWMOutputDevice as PWM
from gpiozero import DigitalOutputDevice as DOD
from time import sleep

# Define the pins for the motor controller

# in1 in2 is for motor direction
AIN1_PIN = 26
AIN2_PIN = 20
PWMA_PIN = 12

BIN1_PIN = 16
BIN2_PIN = 19
PWMB_PIN = 13

# set pin mode
AIN1 = DOD(AIN1_PIN)
AIN2 = DOD(AIN2_PIN)
PWMA = PWM(PWMA_PIN)

BIN1 = DOD(BIN1_PIN)
BIN2 = DOD(BIN2_PIN)
PWMB = PWM(PWMB_PIN)

# set motor direction, in1 high, in2 low
AIN1.on()
AIN2.off()
BIN1.on()
BIN2.off()

# set motor speed
PWMA.value = 0.3
PWMB.value = 0.3

sleep(5)

# stop motor
PWMA.value = 0
PWMB.value = 0

# turn around

AIN1.on()
AIN2.off()
BIN1.off()
BIN2.on()

PWMA.value = 0.3
PWMB.value = 0.3

sleep(5)

# stop motor
PWMA.value = 0
PWMB.value = 0