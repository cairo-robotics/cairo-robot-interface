#!/usr/bin/env python3
import rospy
import argparse
from robot_clients.transform_clients import TransformLookupClient

if __name__ == "__main__":
    rospy.init_node("transform_lookup_client")
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt)
    parser.add_argument(
        '-p', '--parent', default="base",
        help='The parent frame'
    )

    parser.add_argument(
        '-c', '--child', default="base",
        help='The parent frame'
    )

    args = parser.parse_args(rospy.myargv()[1:])

    while not rospy.is_shutdown():
        service = TransformLookupClient(service_name='transform_lookup_service')
        print(service.call(args.parent, args.child))
        rospy.sleep(.05)
