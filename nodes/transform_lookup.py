#!/usr/bin/env python3
import rospy
import argparse
from robot_clients.transform_clients import TransformLookupClient

if __name__ == "__main__":
    rospy.init_node("transform_lookup_client")
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt)
    parser.add_argument(
        '-p', '--parent', default="world",
        help='The parent frame'
    )

    parser.add_argument(
        '-c', '--child', default="right_gripper_tip",
        help='The child frame'
    )

    args = parser.parse_args(rospy.myargv()[1:])

    service = TransformLookupClient(service_name='transform_lookup_service')
    command = ''
    while not rospy.is_shutdown():
        command = input("Proceed with service call (any key) or q to quit  ")
        if command == "q":
            exit()
        print(service.call(args.parent, args.child))
