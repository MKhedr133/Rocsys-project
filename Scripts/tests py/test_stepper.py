import RPi.GPIO as gpio
from time import sleep

gpio.setmode(gpio.BCM)

dir_pin_z1  = 21
step_pin_z1 = 20
en_pin_z1 = 16

dir_pin_z2  = 7
step_pin_z2 = 8
en_pin_z2 = 25

gpio.setup(dir_pin_z1, gpio.OUT)
gpio.setup(step_pin_z1, gpio.OUT)
gpio.setup(en_pin_z1, gpio.OUT)

gpio.setup(dir_pin_z2, gpio.OUT)
gpio.setup(step_pin_z2, gpio.OUT)
gpio.setup(en_pin_z2, gpio.OUT)

def calculate_stepper_steps(distance):
        """ Calculates stepper motor steps using the pitch of the lead-screw and the motor's step angle.

        Args:
            distance: The distance that needs to be moved by a stepper motor in millimeters.

        Returns:
            The number of stepper motor steps
        """
        stepper_steps = int(100 * distance)
        return stepper_steps

def move_stepper_z2(distance):
    steps = calculate_stepper_steps(distance)

    for i in range(steps):
        gpio.output(step_pin_z2, gpio.HIGH)
        sleep(0.0001)
        gpio.output(step_pin_z2, gpio.LOW)
        sleep(0.0001)

gpio.output(en_pin_z2, gpio.LOW)
        
gpio.output(dir_pin_z2, gpio.HIGH) #CW

move_stepper_z2(100)

gpio.cleanup()  