from copy import copy

import rospy

import actionlib

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

import intera_interface

from intera_interface import CHECK_VERSION


class JointPositionTrajectory(object):
    """
    Class that creates a ROS Action client in order to make calls to execute a joint trajectory on the robot.

    Note that this action client expects joint positions and durations, but does not support feed forward inverse dynamics.
    In other words, you cannot supply joint velocities and acceleration but only positions and duration from start at which
    you intend that position to be reached.

    Attributes
    ----------
    limb : str
        The default Intera limb to use. Defaults to "right" for Sawyer.
    joint_names : list
        List of joint names. Defaults to Saywer's 7 joint names.
    """
    def __init__(self, limb="right", joint_names=None):
        self._joint_names = joint_names if joint_names is not None else ['right_j0', 'right_j1', 'right_j2', 'right_j3',
                          'right_j4', 'right_j5', 'right_j6']
        ns = 'robot/limb/' + limb + '/'
        self._client = actionlib.SimpleActionClient(
            ns + "follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        self._goal = FollowJointTrajectoryGoal()
        self._goal_time_tolerance = rospy.Time(0.1)
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear(limb)

    def add_point(self, positions, time):
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time.now()
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def clear(self, limb):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self._goal.trajectory.joint_names = self._joint_names
