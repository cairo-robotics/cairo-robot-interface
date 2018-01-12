#!/usr/bin/env python2
'''simple script that publishes joint message for reference'''
import rospy

import std_msgs.msg
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension


def main():
    joint_pub = rospy.Publisher("/commander/joint_state", Float32MultiArray, queue_size=1)
    '''NOTE call node after pub subs created if not in classes'''
    rospy.init_node('sawyer_joint_sample')
    joint_state = Float32MultiArray()
    dim = MultiArrayDimension()
    dim.label = "test"
    dim.size = 7
    dim.stride = 0
    '''dim is a list that needs to appended?'''
    joint_state.layout.dim.append(dim)
    joint_state.layout.data_offset = 0
    joint_state.data = [0] *7
    joint_state.data[0] = 1.3
    joint_state.data[1] = .5
    joint_state.data[2] = -1.2
    joint_state.data[3] = 1.4
    joint_state.data[4] = -1.1
    joint_state.data[5] = -1.2
    joint_state.data[6] = -4.7
    joint_pub.publish(joint_state)


if __name__ == '__main__':
    main()
    rospy.sleep(5)
