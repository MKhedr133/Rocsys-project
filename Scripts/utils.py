from time import sleep
import RPi.GPIO as gpio


class PlacementUtils:
    """ This class contains the functions that the main.py executes for automatic placement.
    """

    def __init__(self, auto_option, frequency_servos):
        """ This is the initialization part when a class instance is called.
        """
        self.auto_option = auto_option
        self.high = gpio.HIGH
        self.low = gpio.LOW

        # Set GPIO numbering mode
        gpio.setmode(gpio.BCM)

        # Set pins as output and define them as PWM pins for servomotors
        self.pin_rx = 12
        self.pin_ry = 13
        self.pin_rz = 14
        self.frequency_servos = frequency_servos
        gpio.setup(self.pin_rx, gpio.OUT)
        gpio.setup(self.pin_ry, gpio.OUT)
        gpio.setup(self.pin_rz, gpio.OUT)
        self.servo_rx = gpio.PWM(self.pin_rx, self.frequency_servos)
        self.servo_ry = gpio.PWM(self.pin_ry, self.frequency_servos)
        self.servo_rz = gpio.PWM(self.pin_rz, self.frequency_servos)

        # Start the PWM
        self.servo_rx.start(0)
        self.servo_ry.start(0)
        self.servo_rz.start(0)

        # Set pins as output for the TMC2208
        self.dir_pin_z1 = 0
        self.step_pin_z1 = 1
        self.en_pin_z1 = 2
        gpio.setup(self.dir_pin_z1, gpio.OUT)
        gpio.setup(self.step_pin_z1, gpio.OUT)
        gpio.setup(self.en_pin_z1, gpio.OUT)

        self.dir_pin_z2 = 3
        self.step_pin_z2 = 4
        self.en_pin_z2 = 5
        gpio.setup(self.dir_pin_z2, gpio.OUT)
        gpio.setup(self.step_pin_z2, gpio.OUT)
        gpio.setup(self.en_pin_z2, gpio.OUT)

        self.dir_pin_y = 6
        self.step_pin_y = 7
        self.en_pin_y = 8
        gpio.setup(self.dir_pin_y, gpio.OUT)
        gpio.setup(self.step_pin_y, gpio.OUT)
        gpio.setup(self.en_pin_y, gpio.OUT)

        self.cw = self.low
        self.ccw = self.high

        # Set the initial state for the enable pin
        gpio.output(self.en_pin_z1, self.low)
        gpio.output(self.en_pin_z2, self.low)
        gpio.output(self.en_pin_y, self.low)

        # Set limit switch pins as input
        self.limit_switch_pin_z1 = 9
        self.limit_switch_pin_z2 = 10
        self.limit_switch_pin_y = 11
        gpio.setup(self.limit_switch_pin_z1, gpio.IN, pull_up_down=gpio.PUD_UP)
        gpio.setup(self.limit_switch_pin_z2, gpio.IN, pull_up_down=gpio.PUD_UP)
        gpio.setup(self.limit_switch_pin_y, gpio.IN, pull_up_down=gpio.PUD_UP)
        self.home_z1 = False
        self.home_z2 = False
        self.home_y = False

        self.delay_stepper = 0.0001
        self.delay_servo = 0.5

    def ask_user(self):
        """ Asks the user's permission to go to the next pose.

            Returns:
                answer: The answer of the permission (y: yes or n: no).
        """
        answer = input('Go to the next position (y:yes, n: no): ')
        if answer != 'y' or answer != 'n':
            self.ask_user()
        else:
            return answer

    @staticmethod
    def calculate_stepper_steps(distance):
        """ Calculates stepper motor steps using the pitch of the lead-screw and the motor's step angle.

        Args:
            distance: The distance that needs to be moved by a stepper motor in millimeters.

        Returns:
            The number of stepper motor steps
        """
        stepper_steps = int((distance * 360) / (2 * 1.8))
        return stepper_steps

    def move_steppers_z(self, direction, distance) -> None:
        """ Determines the direction of the two stepper motors on the z-axis and moves to the specified distance.

        Args:
            direction: The direction the stepper motors will be moving. (cw: down, ccw: up)
            distance: The distance the stepper motors will be moving in millimeters.
        """
        if direction == 'cw':
            gpio.output(self.dir_pin_z1, self.cw)
            gpio.output(self.dir_pin_z2, self.cw)
        else:
            gpio.output(self.dir_pin_z1, self.ccw)
            gpio.output(self.dir_pin_z2, self.ccw)

        steps = self.calculate_stepper_steps(distance)

        for i in range(steps):
            gpio.output(self.step_pin_z1, self.high)
            gpio.output(self.step_pin_z2, self.high)
            sleep(self.delay_stepper)
            gpio.output(self.step_pin_z1, self.low)
            gpio.output(self.step_pin_z2, self.low)
            sleep(self.delay_stepper)

    def move_stepper_y(self, direction, distance):
        """ Determines the direction of the stepper motor on the y-axis and moves to the specified distance.

               Args:
                   direction: The direction the stepper motors will be moving. (cw: right, ccw: left)
                   distance: The distance the stepper motors will be moving in millimeters.
        """
        if direction == 'cw':
            gpio.output(self.dir_pin_y, self.cw)
        else:
            gpio.output(self.dir_pin_y, self.ccw)

        steps = self.calculate_stepper_steps(distance)

        for i in range(steps):
            gpio.output(self.step_pin_y, self.high)
            sleep(self.delay_stepper)
            gpio.output(self.step_pin_y, self.low)
            sleep(self.delay_stepper)

    def rotate_servo_rx(self, angle):
        """ Rotates the servo in the x-axis with desired angles

        Args:
            angle: The to be rotated angle in degrees.
        """
        angle = int(angle)
        duty = angle / 18 + 2

        gpio.output(self.pin_rx, True)
        self.servo_rx.ChangeDutyCycle(duty)
        sleep(self.delay_servo)
        gpio.output(self.pin_rx, False)
        self.servo_rx.ChangeDutyCycle(0)

    def rotate_servo_ry(self, angle):
        """ Rotates the servo in the y-axis with desired angles

        Args:
            angle: The to be rotated angle in degrees.
        """
        angle = int(angle)
        duty = angle / 18 + 2

        gpio.output(self.pin_ry, True)
        self.servo_ry.ChangeDutyCycle(duty)
        sleep(self.delay_servo)
        gpio.output(self.pin_ry, False)
        self.servo_ry.ChangeDutyCycle(0)

    def rotate_servo_rz(self, angle):
        """ Rotates the servo in the y-axis with desired angles

        Args:
            angle: The to be rotated angle in degrees.
        """
        angle = int(angle)
        duty = angle / 18 + 2

        gpio.output(self.pin_rz, True)
        self.servo_rz.ChangeDutyCycle(duty)
        sleep(self.delay_servo)
        gpio.output(self.pin_rz, False)
        self.servo_rz.ChangeDutyCycle(0)

    def home(self):
        """ The homing sequence of the program.
        """
        # Rotates the servo's to 0 degrees
        self.rotate_servo_rx(0)
        self.rotate_servo_ry(0)
        self.rotate_servo_rz(0)

        # Moves the steppers 40 mm from the limit switch to be able to home
        self.move_steppers_z('ccw', 40)
        self.move_stepper_y('ccw', 40)

        # The steppers move until all limit switches are pressed
        while True:
            if gpio.input(self.limit_switch_pin_z1) == 0:
                self.home_z1 = True
            else:
                self.home_z1 = False

            if gpio.input(self.limit_switch_pin_z2) == 0:
                self.home_z2 = True
            else:
                self.home_z2 = False

            if gpio.input(self.limit_switch_pin_y) == 0:
                self.home_y = True
            else:
                self.home_y = False

            if not self.home_z1:
                gpio.output(self.dir_pin_z1, self.cw)
                gpio.output(self.step_pin_z1, self.high)
                sleep(self.delay_stepper)
                gpio.output(self.step_pin_z1, self.low)

            if not self.home_z2:
                gpio.output(self.dir_pin_z2, self.cw)
                gpio.output(self.step_pin_z2, self.high)
                sleep(self.delay_stepper)
                gpio.output(self.step_pin_z2, self.low)

            if not self.home_y:
                gpio.output(self.dir_pin_y, self.cw)
                gpio.output(self.step_pin_y, self.high)
                sleep(self.delay_stepper)
                gpio.output(self.step_pin_y, self.low)

            if self.home_z1 and self.home_z2 and self.home_y:
                break

    def stop_servos(self):
        """ Stops the PWM of the servomotors.
        """
        self.servo_rx.stop()
        self.servo_ry.stop()
        self.servo_rz.stop()

    def stop_steppers(self):
        """ Disables the step and enable pins of the stepper motors.
        """
        gpio.output(self.step_pin_z1, gpio.LOW)
        gpio.output(self.step_pin_z2, gpio.LOW)
        gpio.output(self.step_pin_y, gpio.LOW)

        gpio.output(self.en_pin_z1, gpio.LOW)
        gpio.output(self.en_pin_z2, gpio.LOW)
        gpio.output(self.en_pin_y, gpio.LOW)

    def end_demonstration(self):
        """ Stops the servo and stepper motors and clears gpio addresses.
        """
        self.stop_servos()
        self.stop_steppers()
        gpio.cleanup()
        