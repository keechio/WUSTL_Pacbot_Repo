#    C2Plabs.com 
#
#

from gpiozero import PWMOutputDevice as PWM
from gpiozero import DigitalOutputDevice as DOD
from time import sleep
from gpiozero import DistanceSensor

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

sensor = DistanceSensor(echo=25, trigger=9)
prevDistance = 10
while True:
    print('Distance: ', sensor.distance * 100)
    
    # when distance < 9, turn left
    if sensor.distance * 100 - prevDistance < 0:
        PWMA.value = 0.12
        PWMB.value = 0.1
    elif sensor.distance * 100 - prevDistance > 0:
        PWMA.value = 0.1
        PWMB.value = 0.12
    else:
        PWMA.value = 0.20
        PWMB.value = 0.20

    prevDistance = sensor.distance * 100

    sleep(.3)


# stop motor
PWMA.value = 0
PWMB.value = 0