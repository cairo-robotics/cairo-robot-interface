#!/usr/bin/env python2
"""
Creates the SawyerServer module node with a start position loader
"""
import os
import rospy
import rospkg
import rosparam
import signal

from sawyer_interface.moveit_interface import  SawyerMoveitInterface

def main():
    #TODO better name than bender commander?
    bender =  SawyerMoveitInterface()
    rospy.init_node('bender_commander')

    #loader for start pose
    rospack = rospkg.RosPack()
    pkg_dir = rospack.get_path('cairo_sawyer_interface')
    pose_csv = rospy.get_param('/sawyer_server/starting_pose')
    #TODO convert to JSON files to store multiple default poses
    with open(pkg_dir + '/std_poses/'+ pose_csv, 'r') as filestrm:
        for line in filestrm:
            crnt = line.split(',')
    start_jpose = []
    for char in crnt:
        start_jpose.append(float(char))
    bender.group.clear_pose_targets()
    bender.group.set_joint_value_target(start_jpose)
    bender.group.go()
    rospy.spin()

if __name__ == '__main__':
    main()
