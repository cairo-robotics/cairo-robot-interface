#!/usr/bin/env python

import sys
import rospy
from cairo_sawyer_interface.srv import ForwardKinematics
from std_msgs.msg import Float32MultiArray


class FowardKinematicsClient():

    def __init__(self):
        rospy.wait_for_service('forward_kinematic_service', timeout = 5)
        self.fk_server = rospy.ServiceProxy('forward_kinematic_service', ForwardKinematics)

    def call(self, joint_multiarray):
        try:
            resp = self.fk_server(joint_multiarray)
            return resp.end_effector_pose
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def convert_joint_value_list(self, joint_value_list):
        joints = Float32MultiArray()
        joints.data = joint_value_list
        return joints

if __name__ == "__main__":
    fk_client = FowardKinematicsClient()
    joint_multiarray = fk_client.convert_joint_value_list([-.1, -.5, -3.0, -1.5, -.05, -.6, -1.5])
    resp = fk_client.call(joint_multiarray)
    print "End effector pose: %s"%resp