from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter, Namespace
from typing import Optional
from utils import PlacementUtils

def parse_args() -> Namespace:
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
        required=True,
        default=False,
        help="This parameter decides if the script going to ask the user everytime "
             "the calibration board going to a new pose. "
             "True: it will not ask the user"
             "False: it will"
    )

    # Parse arguments.
    args = parser.parse_args()
    return args


def main(args: Optional[Namespace] = None):
    if args is None:
        args = parse_args()

    # Saving args parameters to variables.
    auto = args.auto
    demo_class_instance = PlacementUtils(auto)

    try:
        demo_class_instance = PlacementUtils(auto)

    except KeyboardInterrupt:
        demo_class_instance.end_demonstration()