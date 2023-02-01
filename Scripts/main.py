from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter, Namespace
from typing import Optional
from utils import PlacementUtils


def parse_args():
    """ Parses the command line arguments.

    Returns:
        The parsed command line arguments.
    """
    parser = ArgumentParser(
        description="Script for automatic placement of the calibration board.",
        formatter_class=ArgumentDefaultsHelpFormatter,
        allow_abbrev=False,
    )
    parser.add_argument(
        '-auto',
        type=bool,
        metavar='-automatic_placement',
        required=False,
        default=True,
        help="This parameter decides if the script going to ask the user everytime "
             "the calibration board going to a new pose. "
             "True: it will not ask the user"
             "False: it will"
    )
    parser.add_argument(
        '-freq_servos',
        type=int,
        metavar='frequency_servos',
        required=False,
        default=50,
        help="The PWM frequency for the servomotors. It can only be between 50-200 Hertz."
    )

    # Parse arguments.
    args = parser.parse_args()
    return args


def main(args):
    if args is None:
        args = parse_args()

    # Saving args parameters to variables.
    auto = args.auto
    freq_servos = args.freq_servos

    demo_class_instance = PlacementUtils(auto, freq_servos)

    try:
        demo_class_instance.move_stepper_y('left', 200)
        demo_class_instance.move_stepper_y('right', 200)
        demo_class_instance.end_demonstration()

    except KeyboardInterrupt:
        demo_class_instance.end_demonstration()

main(None)

