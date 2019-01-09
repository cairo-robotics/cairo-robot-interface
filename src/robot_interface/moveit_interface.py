from abc import ABCMeta, abstractmethod
import moveit_commander
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
import rospy
from kinematics_interface.kinematics_servers import MoveItForwardKinematicsServer, MoveItInverseKinematicsServer, RobotStateValidityServer


class AbstractMoveitInterface:
    __metaclass__ = ABCMeta

    @abstractmethod
    def set_velocity_scaling(self, velocity_scaling):
        pass

    @abstractmethod
    def set_acceleration_scaling(self, acceleration_scaling):
        pass

    @abstractmethod
    def set_pose_target(self, pose):
        pass

    @abstractmethod
    def set_joint_target(self, joints):
        pass

    @abstractmethod
    def plan(self):
        pass

    @abstractmethod
    def execute(self, plan):
        pass

    @abstractmethod
    def get_FK_pose(self, joints):
        pass

    @abstractmethod
    def get_IK_pose(self, pose):
        pass

    @abstractmethod
    def check_point_validity(self):
        pass

    @abstractmethod
    def create_robot_state(self, joints):
        pass


class SawyerMoveitInterface(AbstractMoveitInterface):
    '''creat various servers for reading in pose and joint data and moving
    sawyer accordingly'''

    def __init__(self, planning_group="right_arm"):
        '''creates subs pubs and moveit_commander groups'''
        super(AbstractMoveitInterface, self).__init__()
        self.group = moveit_commander.MoveGroupCommander(planning_group)
        self.fk_server = MoveItForwardKinematicsServer()
        self.ik_server = MoveItInverseKinematicsServer()
        self.rs_validity = RobotStateValidityServer()

    def set_velocity_scaling(self, velocity_scaling):
        self.group.set_max_velocity_scaling_factor(velocity_scaling)

    def set_acceleration_scaling(self, acceleration_scaling):
        self.group.set_max_acceleration_scaling_factor(acceleration_scaling)

    def set_pose_target(self, pose):
        '''set the pose target using the MoveGroupCommander'''
        self.group.set_pose_target(pose)

    def set_joint_target(self, joints):
        '''set the joint targets using the MoveGroupCommander'''
        self.group.set_joint_value_target(joints)

    def plan(self):
        '''set the pose plan using the MoveGroupCommander'''
        return self.group.plan()

    def execute(self, plan):
        '''execute the plan using MoveGroupCommander'''
        self.group.execute(plan)

    def move_to_joint_targets(self, joint_target_list):
        for joint_target in joint_target_list:
            self.set_joint_target(joint_target)
            plan = self.plan()
            self.execute(plan)

    # def get_FK_pose(self,
    #                 joint_positions,
    #                 joint_names=['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6'],
    #                 tip_names=["right_hand"]):
    #     return self.fk_server.call(joint_positions, joint_names=joint_names, tip_names=tip_names)

    # def get_IK_pose(self, pose, tip_name="/right_hand", limb="left", use_advanced_options=False):
    #     return self.ik_server.call(pose, tip_name=tip_name, limb=limb, use_advanced_options=use_advanced_options)

    def get_FK_pose(self, joint_positions):
        return self.fk_server.call(joint_positions)

    def get_IK_pose(self, pose):
        return self.ik_server.call(pose)

    def check_point_validity(self, robot_state, group_name="right_arm"):
        if type(robot_state) is not RobotState:
            rospy.logerror("check_point_validity expects a RobotState message to check point validity.")
            return None
        return self.rs_validity.call(robot_state, group_name=group_name)

    def create_robot_state(self, joints, joint_names=['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']):
        robot_state = RobotState()
        joint_state = JointState()
        joint_state.name = joint_names
        joint_state.position = joints
        robot_state.joint_state = joint_state
        return robot_state

    def get_end_effector_pose(self, joint_positions, limb="right"):
        # DEPRECATED
        return self.get_FK_pose(joint_positions)
