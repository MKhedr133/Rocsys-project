from gpiozero import AngularServo
from time import sleep

servo = AngularServo(12)

servo.angle = -30

sleep(2)