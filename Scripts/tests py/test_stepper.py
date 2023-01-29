import RPi.GPIO as gpio
from time import sleep


gpio.setmode(gpio.BCM)

dir_pin = 21
step_pin = 20
en_pin = 16

gpio.setup(dir_pin, gpio.OUT)
gpio.setup(step_pin, gpio.OUT)
gpio.setup(en_pin, gpio.OUT)


def calculate_stepper_steps(distance):
        """ Calculates stepper motor steps using the pitch of the lead-screw and the motor's step angle.

        Args:
            distance: The distance that needs to be moved by a stepper motor in millimeters.

        Returns:
            The number of stepper motor steps
        """
        pitch = 8
        stepper_steps = int((distance * 360) / (pitch * 1.8))
        return stepper_steps

def move_stepper(distance):
    steps = calculate_stepper_steps(distance)

    for i in range(steps):
        gpio.output(step_pin, gpio.HIGH)
        sleep(0.0001)
        gpio.output(step_pin, gpio.LOW)
        sleep(0.0001)

gpio.output(en_pin, gpio.LOW)
        
gpio.output(dir_pin, gpio.LOW) #CW

move_stepper(1000)

gpio.cleanup()  