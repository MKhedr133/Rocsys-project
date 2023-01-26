import RPi.GPIO as gpio
import time

gpio.setmode(gpio.BCM)
limit_pin = 21

gpio.setup(limit_pin, gpio.IN, pull_up_down=gpio.PUD_UP)

while True:
    if gpio.input(limit_pin) == 0:
        print('Pressed')
    else:
        print('Not Pressed')

gpio.cleanup()