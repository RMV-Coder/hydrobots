from gpiozero import DistanceSensor
from time import sleep
sensor = DistanceSensor(20,21)

while True:
    print('Distance is ', sensor.distance * 100, 'cm')
    sleep(0.5)