from time import sleep
import RPi.GPIO as gpio


class PlacementUtils:
    """ This class contains the functions that the main.py executes for automatic placement.
    """

    def __init__(self) -> None:
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
        dir_pin_z1 = 0
        step_pin_z1 = 1
        en_pin_z1 = 2
        gpio.setup(dir_pin_z1, gpio.OUT)
        gpio.setup(step_pin_z1, gpio.OUT)
        gpio.setup(en_pin_z1, gpio.OUT)

        dir_pin_z2 = 3
        step_pin_z2 = 4
        en_pin_z2 = 5
        gpio.setup(dir_pin_z2, gpio.OUT)
        gpio.setup(step_pin_z2, gpio.OUT)
        gpio.setup(en_pin_z2, gpio.OUT)

        dir_pin_y = 6
        step_pin_y = 7
        en_pin_y = 8
        gpio.setup(dir_pin_y, gpio.OUT)
        gpio.setup(step_pin_y, gpio.OUT)
        gpio.setup(en_pin_y, gpio.OUT)

        # Set the initial state for the enable pin
        gpio.output(en_pin_z1, gpio.HIGH)
        gpio.output(en_pin_z2, gpio.HIGH)
        gpio.output(en_pin_y, gpio.HIGH)

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

    def home(self):
        self.rotate_servo(['rx', 'ry', 'rz'], [0, 0, 0])

    def rotate_servo(self, rotation_axis: list, angle: list) -> None:
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