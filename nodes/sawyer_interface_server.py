#!/usr/bin/env python2
import os
import rospy
import rospkg
import rosparam
import signal

from sawyer_interface.sawyer_server import SawyerServer

class DeathNote(object):
    '''DeathNote for exiting runaway loops'''
    write_name = False
    def __init__(self):
        signal.signal(signal.SIGINT, self.ryuk)
        signal.signal(signal.SIGTERM, self.ryuk)

    def ryuk(self, signum, frame):
        '''name written kill the loop'''
        self.write_name = True


def main():
    '''creates node and pub sub when called main'''
    rospy.init_node('sawyer_commander')
    #kill class
    yagami = DeathNote()
    rospack = rospkg.RosPack()
    pkg_dir = rospack.get_path('cairo_sawyer_interface')
    pose_csv = rospy.get_param('/sawyer_server/starting_pose')
    with open(pkg_dir + '/std_poses/'+ pose_csv, 'r') as filestrm:
        for line in filestrm:
            crnt = line.split(',')
            print crnt
    print crnt[0]
    start_jpose = []
    for char in crnt:
        start_jpose.append(float(char))

    bender = SawyerServer()
    bender.group.clear_pose_targets()
    bender.group.set_joint_value_target(start_jpose)
    bender.group.go()



    pose1 = [-.04, 1.0, -3.0, 2.0, -0.1, 0.5, -1.0]
    pose2 = [-.04, 1.0, -3.0, 1.0, -0.1, 0.5, -1.0]



    while True:
        if yagami.write_name: #execute death note
            break
    print "Hey, all it did was go around in a full circle."


if __name__ == '__main__':
    main()
