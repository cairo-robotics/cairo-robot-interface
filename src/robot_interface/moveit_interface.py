from abc import ABCMeta, abstractmethod
import moveit_commander
from kinematics_interface.servers import SawyerForwardKinematicsServer, SaywerInverseKinematicsServer, RobotStateValidityServer


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


class SawyerMoveitInterface(AbstractMoveitInterface):
    '''creat various servers for reading in pose and joint data and moving
    sawyer accordingly'''

    def __init__(self, planning_group="right_arm"):
        '''creates subs pubs and moveit_commander groups'''

        self.group = moveit_commander.MoveGroupCommander(planning_group)
        self.fk_server = SawyerForwardKinematicsServer()
        self.ik_server = SaywerInverseKinematicsServer()
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

        def get_FK_pose(self,
                        joint_positions,
                        joint_names=['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6'],
                        tip_names=["right_wrist"],
                        limb="right"):
            return self.fk_server.call(joint_positions, joint_names=joint_names, tip_names=tip_names, limb=limb)

        def get_IK_pose(self, pose, tip_name="/right_gripper", limb="left", use_advanced_options=True):
            return self.ik_server.call(pose, tip_name=tip_name, limb=limb, use_advanced_options=use_advanced_options)

        def check_point_validity(self, robot_state, group_name="right_arm"):
            return self.rs_validity.call(robot_state, group_name=group_name)  

        def get_end_effector_pose(self, joint_positions, limb="right"):
            # DEPRECATED
            return self.get_FK_pose(joint_positions, limb="right")    