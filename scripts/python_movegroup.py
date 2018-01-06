#!/usr/bin/env python2
'''
module description
example_pose
    pose_target.orientation.w = 0
    pose_target.orientation.x = 0
    pose_target.orientation.y = 1
    pose_target.orientation.z = 0
    pose_target.position.x = .8
    pose_target.position.y = -.1
    pose_target.position.z = .3
'''
#import sys
import signal

import rospy
import moveit_commander

import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from std_msgs.msg import String


class DeathNote(object):
    '''DeathNote for exiting runaway loops'''
    write_name = False
    def __init__(self):
        signal.signal(signal.SIGINT, self.ryuk)
        signal.signal(signal.SIGTERM, self.ryuk)

    def ryuk(self, signum, frame):
        '''name written kill the loop'''
        self.write_name = True

class SawyerClass(object):
    '''creat various servers for reading in pose and joint data and moving
    sawyer accordingly'''

    def __init__(self, PLANNING_GROUP="right arm", sub_topic="/commander/pose",
                 pub_topic="/test_topic"):
        '''creates subs pubs and moveit_commander groups'''

        self.subscriber = rospy.Subscriber(sub_topic, Pose,
                                           self._moveit_pose_callback,
                                           queue_size=1)
        self.diagnostic = rospy.Publisher(pub_topic, String)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander(PLANNING_GROUP)

    def _moveit_pose_callback(self, pose):
        '''pose callback function'''
        self.group.set_pose_target(pose)
        self.plan = self.group.plan()
        rospy.sleep(5)
        self.group.execute(self.plan)
        self.diagnostic.publish("future completion status message")

    #added methods b/c unable to inherit moveit_commander to sawyer_class
    #TODO find a better way? my guess is that there multiple classes to inherit
    def set_pose_target(self, pose):
        '''set the pose target using the MoveGroupCommander'''
        self.group.set_pose_target(pose)

    def plan(self):
        '''set the pose plan using the MoveGroupCommander'''
        self.plan = self.group.plan()

    def execute(self):
        '''execute the plan using MoveGroupCommander'''
        self.group.execute(plan)

    def move_pose(self, pose):
        '''use pose to set target, plan, and execute'''
        self.set_pose_target(pose)
        self.plan()
        rospy.sleep(4)
        self.group.execute(self.plan)


def main():
    '''creates node and pub sub when called main'''
    #kill command
    yagami = DeathNote()
    print "hello world"
    rospy.init_node('python_commander')
    bender = SawyerClass("right_arm")

    while(True):
        if yagami.write_name: #execute death note
            break
    print "Hey, all it did was go around in a full circle."



if __name__ == '__main__':
    main()
