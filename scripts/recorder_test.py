#!/usr/bin/env python2

import rospy
import argparse
import intera_interface
from intera_interface import CHECK_VERSION
from test_moveit.recorder import Recorder


def main():
    """Pose Recorder

    Record timestamped pose to a file for
    later play back.

    Run this example while moving the robot's arm to record a
    time series of end effector pose positions to a
    new csv file with the provided *filename*. This example can
    be run in parallel with any other example or standalone
    (moving the arms in zero-g mode).

    You can later play the movements back using one of the
    *_file_playback examples.
    """
    epilog = """
    Related examples:
    joint_position_file_playback.py; joint_trajectory_file_playback.py.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-f', '--file', dest='filename', required=True,
        help='the file name to record to'
    )
    parser.add_argument(
        '-r', '--record-rate', type=int, default=100, metavar='RECORDRATE',
        help='rate at which to record (default: 100)'
    )
    args = parser.parse_args(rospy.myargv()[1:])


    print("Initializing node... ")
    rospy.init_node("sdk_joint_recorder")
    print("Getting robot state... ")
    rs = intera_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()

    recorder = Recorder(args.filename, args.record_rate)
    rospy.on_shutdown(recorder.stop)



    print("Recording. Press Ctrl-C to stop.")
    recorder.record_demonstration()

    print("\nDone.")

if __name__ == '__main__':
    main()