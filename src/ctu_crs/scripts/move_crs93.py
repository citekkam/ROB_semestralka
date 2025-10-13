#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2025-10-02
#

import argparse
import numpy as np
from ctu_crs import CRS93


def main():
    """Moves the CRS93 robot by a relative offset for specified joints."""
    parser = argparse.ArgumentParser(
        description="Moves the CRS93 robot by a relative offset for specified joints.",
        epilog="""
Example:
  # Move joint q0 by 10 degrees and joint q1 by -5 degrees
  move_crs93 q0 10 q1 -5

  # Move joint q2 by 20.5 degrees without homing the robot first
  move_crs93 --nohome q2 20.5
""",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--nohome",
        action="store_true",
        help="Initialize the robot without performing the homing sequence.",
    )
    parser.add_argument(
        "joint_value_pairs",
        metavar="<joint> <angle>",
        nargs="+",
        help="""One or more pairs of joint names (q0-q5) and their corresponding relative
angles in degrees. Each joint name must be followed by its angle.""",
    )

    args = parser.parse_args()

    if len(args.joint_value_pairs) % 2 != 0:
        parser.error("Invalid number of arguments. Please provide joint-value pairs.")
        return

    joint_offsets = {}
    for i in range(0, len(args.joint_value_pairs), 2):
        joint_name = args.joint_value_pairs[i]
        try:
            joint_index = int(joint_name.replace("q", ""))
            angle_deg = float(args.joint_value_pairs[i + 1])
            if joint_index in joint_offsets:
                joint_offsets[joint_index] += angle_deg
            else:
                joint_offsets[joint_index] = angle_deg
        except (ValueError, IndexError):
            parser.error(
                f"Invalid joint-value pair: {joint_name}, {args.joint_value_pairs[i+1]}"
            )
            return

    robot = CRS93()
    robot.initialize(home=not args.nohome)

    q_current = robot.get_q()
    q_target = q_current.copy()

    for joint_index, angle_deg in joint_offsets.items():
        if 0 <= joint_index < len(q_target):
            q_target[joint_index] += np.deg2rad(angle_deg)
        else:
            print(f"Warning: Joint index {joint_index} is out of bounds. Ignoring.")

    print(
        f"Moving to target joint configuration (deg): {np.rad2deg(q_target).round(2)}"
    )
    robot.move_to_q(q_target)

    robot.close()
    print("Movement complete.")


if __name__ == "__main__":
    main()
