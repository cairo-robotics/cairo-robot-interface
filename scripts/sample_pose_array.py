#!/usr/bin/env python2

'''sample of publishing a pose array for waypoint planing'''

import rospy
import math
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray



def main():
    PoseArray_pub = rospy.Publisher("/commander/pose_array", PoseArray, queue_size=1)
    rospy.init_node("sample_pose_array")
    num_waypoints = 10
    pose_array = []

    base_pose = Pose()
    base_pose.orientation.w = 0
    base_pose.orientation.x = 0
    base_pose.orientation.y = 1
    base_pose.orientation.z = 0
    base_pose.position.x = .5
    base_pose.position.y = -.6
    base_pose.position.z = .3

    dt = (2*math.pi)/(num_waypoints-1)

    for i in range(num_waypoints):
        base_pose.position.x = base_pose.position.x - .1
        pose_array.append(base_pose)
        print base_pose

    PoseArray_pub.publish(poses=pose_array)







if __name__ == '__main__':
    main()
