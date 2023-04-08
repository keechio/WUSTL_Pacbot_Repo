from gpiozero import DistanceSensor
from time import sleep

sensor = DistanceSensor(echo=25, trigger=9)
while True:
    print('Distance: ', sensor.distance * 100)
    sleep(.5)