import RPi.GPIO as GPIO
import time


servo_pin = 21

GPIO.setmode(GPIO.BCM)

GPIO.setup(servo_pin, GPIO.OUT)

p = GPIO.PWM(servo_pin , 100)

p.start(0)

def set_angle(angle):
    duty = angle/18 + 2
    GPIO.output(servo_pin, True)
    p.ChangeDutyCycle(duty)
    time.sleep(0.5)
    GPIO.output(servo_pin, False)
    p.ChangeDutyCycle(0)

set_angle(90)
set_angle(270)




p.stop()

GPIO.cleanup()
