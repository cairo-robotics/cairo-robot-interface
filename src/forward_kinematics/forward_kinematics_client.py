#!/usr/bin/env python

import sys
import rospy
from cairo_sawyer_interface.srv import ForwardKinematics
from std_msgs.msg import Float32MultiArray


def get_end_effector_pose(joint_values):
    rospy.wait_for_service('forward_kinematic_service')
    try:
        foward_kinematics = rospy.ServiceProxy('forward_kinematic_service', ForwardKinematics)
        resp1 = foward_kinematics(joint_values)
        return resp1.end_effector_pose
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == "__main__":
    joints = Float32MultiArray()
    joints.data = [-.1, -.5, -3.0, -1.5, -.05, -.6, -1.5]
    print "End effector pose: %s"%(get_end_effector_pose(joints))