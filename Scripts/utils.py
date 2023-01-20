from time import sleep
import RPi.GPIO as gpio


class PlacementUtils:
    """ This class contains the functions that the main.py executes for automatic placement.
    """

    def __init__(self, auto_option: bool) -> None:
        """ This is the initialization part when a class instance is called.
        """
        self.auto_option = auto_option

        # Set GPIO numbering mode
        gpio.setmode(gpio.BOARD)

        # Set pins as output and define them as PWM pins for servomotors
        pin_rx = 12
        pin_ry = 13
        pin_rz = 17
        frequency = 50
        gpio.setup(pin_rx, gpio.OUTPUT)
        gpio.setup(pin_ry, gpio.OUTPUT)
        gpio.setup(pin_rz, gpio.OUTPUT)
        self.servo_rx = gpio.PWM(pin_rx, frequency)
        self.servo_ry = gpio.PWM(pin_ry, frequency)
        self.servo_rz = gpio.PWM(pin_rz, frequency)

        # Start PWM running with value 0 (servo off)
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

        # Set the initial state for the enable pin
        gpio.output(self.en_pin_z1, gpio.HIGH)
        gpio.output(self.en_pin_z2, gpio.HIGH)
        gpio.output(self.en_pin_y, gpio.HIGH)

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

    def ask_user(self) -> str:
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
    def calculate_stepper_steps(distance: float) -> int:
        """ Calculates stepper motor steps using the pitch of the lead-screw and the motor's step angle.

        Args:
            distance: The distance that needs to be moved by a stepper motor in millimeters.

        Returns:
            The number of stepper motor steps
        """
        stepper_steps = int((distance * 360) / (2 * 1.8))
        return stepper_steps

    def move_steppers_z(self, direction: str, distance: float) -> None:
        """ Determines the direction of the two stepper motors on the z-axis and moves to the specified distance.

        Args:
            direction: The direction the stepper motors will be moving. (CW: down, CCW: up)
            distance: The distance the stepper motors will be moving in millimeters.
        """
        if direction == 'CW':
            gpio.output(self.dir_pin_z1, gpio.HIGH)
            gpio.output(self.dir_pin_z2, gpio.HIGH)
        else:
            gpio.output(self.dir_pin_z1, gpio.LOW)
            gpio.output(self.dir_pin_z2, gpio.LOW)

        steps = self.calculate_stepper_steps(distance)

        for i in range(steps):
            gpio.output(self.step_pin_z1, gpio.HIGH)
            gpio.output(self.step_pin_z2, gpio.HIGH)
            sleep(0.001)
            gpio.output(self.step_pin_z1, gpio.LOW)
            gpio.output(self.step_pin_z2, gpio.LOW)
            sleep(0.001)

    def move_stepper_y(self, direction: str, distance: float) -> None:
        """ Determines the direction of the stepper motor on the y-axis and moves to the specified distance.

               Args:
                   direction: The direction the stepper motors will be moving. (CW: right, CCW: left)
                   distance: The distance the stepper motors will be moving in millimeters.
        """
        if direction == 'CW':
            gpio.output(self.dir_pin_y, gpio.HIGH)
        else:
            gpio.output(self.dir_pin_y, gpio.LOW)

        steps = self.calculate_stepper_steps(distance)

        for i in range(steps):
            gpio.output(self.step_pin_y, gpio.HIGH)
            sleep(0.001)
            gpio.output(self.step_pin_y, gpio.LOW)
            sleep(0.001)

    def home(self) -> None:
        """ The homing sequence of the program.
        """
        # Rotates the servo's to 0 degrees
        self.rotate_servos(['rx', 'ry', 'rz'], [0, 0, 0])

        # Moves the steppers 40 mm from the limit switch to be able to home
        self.move_steppers_z('CW', 40)
        self.move_stepper_y('CW', 40)

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
                gpio.output(self.dir_pin_z1, gpio.LOW)
                gpio.output(self.step_pin_z1, gpio.HIGH)
                sleep(0.001)
                gpio.output(self.step_pin_z1, gpio.LOW)

            if not self.home_z2:
                gpio.output(self.dir_pin_z2, gpio.LOW)
                gpio.output(self.step_pin_z2, gpio.HIGH)
                sleep(0.001)
                gpio.output(self.step_pin_z2, gpio.LOW)

            if not self.home_y:
                gpio.output(self.dir_pin_y, gpio.LOW)
                gpio.output(self.step_pin_y, gpio.HIGH)
                sleep(0.001)
                gpio.output(self.step_pin_y, gpio.LOW)

            if self.home_z1 and self.home_z2 and self.home_y:
                break

    def rotate_servos(self, rotation_axis: list, angle: list) -> None:
        """ Rotates one or more servo with desired angles

        Args:
            rotation_axis: Definition to which servo should rotated (rx, ry, rz).
            angle: The to be rotated angle in degrees.
        """
        rx, ry, rz = angle

        for i in range(len(rotation_axis)):
            rotation_axis = rotation_axis[i]
            if rotation_axis == 'rx':
                self.servo_rx.ChangeDutyCycle(2 + (rx / 18))
            elif rotation_axis == 'ry':
                self.servo_ry.ChangeDutyCycle(2 + (ry / 18))
            elif rotation_axis == 'rz':
                self.servo_rz.ChangeDutyCycle(2 + (rz / 18))

        sleep(0.5)
        self.servo_rx.ChangeDutyCycle(0)
        self.servo_ry.ChangeDutyCycle(0)
        self.servo_rz.ChangeDutyCycle(0)

    def end_demonstration(self):
        self.servo_rx.stop()
        self.servo_ry.stop()
        self.servo_rz.stop()

        gpio.output(self.step_pin_z1, gpio.LOW)
        gpio.output(self.step_pin_z2, gpio.LOW)
        gpio.output(self.step_pin_y, gpio.LOW)

        gpio.output(self.en_pin_z1, gpio.LOW)
        gpio.output(self.en_pin_z2, gpio.LOW)
        gpio.output(self.en_pin_y, gpio.LOW)

        gpio.cleanup()