#!/usr/bin/env python2


import sys
import signal

import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from geometry_msgs.msg import Pose
from std_msgs.msg import String


'''
example_pose
    pose_target.orientation.w = 0
    pose_target.orientation.x = 0
    pose_target.orientation.y = 1
    pose_target.orientation.z = 0
    pose_target.position.x = .8
    pose_target.position.y = -.1
    pose_target.position.z = .3
'''


class deathnote:
    write_name = False
    def __init__(self):
        signal.signal(signal.SIGINT, self.ryuk)
        signal.signal(signal.SIGTERM, self.ryuk)

    def ryuk(self, signum, frame):
        self.write_name = True


class sawyer_class():

    def __init__(self, PLANNING_GROUP = "right arm", sub_topic="/commander/pose", pub_topic = "/test_topic"):
        self.subscriber = rospy.Subscriber(sub_topic, Pose, self.moveit_callback, queue_size = 1)
        self.diagnostic = rospy.Publisher(pub_topic, String)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander(PLANNING_GROUP)


    def moveit_callback(self, pose):
        self.group.set_pose_target(pose)
        self.plan = self.group.plan()
        rospy.sleep(5)
        self.group.execute(self.plan)
        self.diagnostic.publish("future completion status message")


    #added methods b/c unable to inherit moveit_commander to sawyer_class
    #TODO find a better way? my guess is that there multiple classes to inherit
    def set_pose_target(self, pose):
        self.group.set_pose_target(pose)

    def plan(self):
        self.plan = self.group.plan()

    def execute():
        self.group.execute(plan)

    def move_pose(self, pose):
        self.set_pose_target(pose)
        self.plan()
        rospy.sleep(4)
        self.group.execute(self.plan)


if __name__=='__main__':
    #kill command
    yagami = deathnote()
    print "hello world"

    rospy.init_node('python_commander')
    bender = sawyer_class("right_arm")

    #unused? it publishes anyways? TODO figure this thing out
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size = 20)

    while(1):
        if yagami.write_name: #execute death note
            break
    print "Hey, all it did was go around in a full circle."
