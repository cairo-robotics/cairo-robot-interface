from abc import ABCMeta, abstractmethod
import moveit_commander
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
import rospy
from robot_clients.kinematics_clients import MoveitForwardKinematicsClient, MoveitInverseKinematicsClient, MoveitRobotStateValidityClient


class AbstractMoveitInterface:
    """
    Abstract class to program to specific interface for interacting with Moveit.
    """
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
    """
    Class that provides a simplified interface to Moveit Python API

    Attributes
    ----------
    group : moveit_commander.MoveGroupCommander
        The MoveGroupCommander providing core MoveGroup functionality.
    robot : moveit_commander.RobotCommander()
        The RobotCommander used to get RobotState
    fk_client : MoveitForwardKinematicsClient
        The fMoveitForwardKinematicsClient to make Forward Kinematics calculation requests.
    ik_client : MoveitInverseKinematicsClient
        The fMoveitForwardKinematicsClient to make Inverse Kinematics calculation requests.
    rsv_client : MoveitRobotStateValidityClient
        The MoveitRobotStateValidityClient to check validity of robot states.
    """

    def __init__(self, planning_group="right_arm"):
        """
        Parameters
        ----------
        planning_group : str
            Name of MoveGroup to pass into MoveGroupCommander initialization.
        group : moveit_commander.MoveGroupCommander
            The MoveGroupCommander providing core MoveGroup functionality.
        robot : moveit_commander.RobotCommander()
            The RobotCommander used to get RobotState
        fk_client : MoveitForwardKinematicsClient
            The fMoveitForwardKinematicsClient to make Forward Kinematics calculation requests.
        ik_client : MoveitInverseKinematicsClient
            The fMoveitForwardKinematicsClient to make Inverse Kinematics calculation requests.
        rsv_client : MoveitRobotStateValidityClient
            The MoveitRobotStateValidityClient to check validity of robot states.
        """
        super(AbstractMoveitInterface, self).__init__()
        self.group = moveit_commander.MoveGroupCommander(planning_group)
        self.robot = moveit_commander.RobotCommander()
        self.fk_client = MoveitForwardKinematicsClient()
        self.ik_client = MoveitInverseKinematicsClient()
        self.rsv_client = MoveitRobotStateValidityClient()

    def get_robot_state(self):
        """
        Get the current state of the robot according to Moveit.

        Returns
        -------
        : RobotState
            The current state of the robot.
        """
        return self.robot.get_current_state()

    def set_velocity_scaling(self, velocity_scaling):
        """
        Set the velocity scaling parameter for Moveit. Generally a fractional percentage of the maximum velocity the
        robot.

        Parameters
        ----------
        velocity_scaling : float
            Fractional percentage valued from 0 to 1.
        """
        self.group.set_max_velocity_scaling_factor(velocity_scaling)

    def set_acceleration_scaling(self, acceleration_scaling):
        """
        Set the acceleration scaling parameter for Moveit. Generally a fractional percentage of the maximum acceleration the
        robot.

        Parameters
        ----------
        acceleration_scaling : float
            Fractional percentage valued from 0 to 1.
        """
        self.group.set_max_acceleration_scaling_factor(acceleration_scaling)

    def set_pose_target(self, pose):
        """
        Sets the pose target using the MoveGroupCommander.

        Parameters
        ----------
        pose : geometry_msgs.Pose
            Pose to set as the target.
        """
        self.group.set_pose_target(pose)

    def set_joint_target(self, joints, joint_names=['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']):
        """
        Sets the joint targets using the MoveGroupCommander.

        Parameters
        ----------
        joints : list
            List of joint angles.
        joint_names : list
            List of corresponding join names for given angles.
        """
        joint_state = JointState()
        joint_state.name = joint_names
        joint_state.position = joints
        try:
            self.group.set_joint_value_target(joint_state)
        except moveit_commander.exception.MoveItCommanderException as e:
            # https://github.com/Kinovarobotics/kinova-ros/issues/110
            rospy.loginfo("False negative from Python for invalid joint state.")

    def plan(self):
        """
        Generate plan via MoveGroupCommander.

        Returns
        -------
        : RobotTrajectory
            A RobotTrajectory representing the plan to execute.
        """
        return self.group.plan()

    def execute(self, plan):
        """
        Executes a generated plan.

        Parameters
        ----------
        plan : RobotTrajectory
            A RobotTrajectory representing the plan to execute.
        """
        self.group.execute(plan)

    def move_to_joint_targets(self, joint_target_list):
        """
        Generates an executes plans for a series of joint targets.

        Parameters
        ----------
        joint_target_list : list
            A list of joint targets (joint angles/positions).
        """
        for joint_target in joint_target_list:
            self.set_joint_target(joint_target)
            plan = self.plan()
            self.execute(plan)

    def get_FK_pose(self, joint_positions):
        """
        Gets the pose for a given joint angle arrangement via Forward Kinematics.

        Parameters
        ----------
        joint_positions : list
            A list of joint angles/positions.

        Returns
        -------
        : geometry_msgs.Pose / None
            Returns the calculate Pose, or None if the response is invalid.
        """
        resp = self.fk_client.call(joint_positions)
        if resp.valid:
            pose = resp.pose
            return pose
        else:
            return None

    def get_IK_pose(self, pose):
        """
        Gets the joint angles for a given pose via Inverse Kinematics.

        Parameters
        ----------
        pose : geometry_msgs.Pose
           The geometry_msgs.Pose for which to find to find joint angles via Inverse Kinematics.

        Returns
        -------
        : list
            Returns a list of joint angles / positions.
        """
        resp = self.ik_client.call(pose)
        if resp.valid:
            return resp.joint_state.position
        else:
            return None

    def check_point_validity(self, robot_state, group_name="right_arm"):
        """
        Checks a robot state for validity. Generally this checks a point to see if it is in collision given a planning scene.

        Parameters
        ----------
        robot_state : geometry_msgs.Pose
            RobotState for which to check validity.
        group_name : str
            The Move Group required to check state validity. 

        Returns
        -------
        : GetStateValidityResponse
            The GetStateValidityResponse response from the /check_state_validity service.
        """
        if type(robot_state) is not RobotState:
            rospy.logerror(
                "check_point_validity expects a RobotState message to check point validity.")
            return None
        return self.rsv_client.call(robot_state, group_name=group_name)

    def create_robot_state(self, joints, joint_names=['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']):
        """
        Creates a RobotState for a given set of joint angles and corresponding names.

        Parameters
        ----------
        joints : list
            List of joint angles.
        joint_names : list
            List of corresponding join names for given angles.

        Returns
        -------
        robot_state : geometry_msgs.Pose
            The generated RobotState
        """
        robot_state = RobotState()
        joint_state = JointState()
        joint_state.name = joint_names
        joint_state.position = joints
        robot_state.joint_state = joint_state
        return robot_state

    def get_end_effector_pose(self, joint_positions, limb="right"):
        # DEPRECATED
        return self.get_FK_pose(joint_positions)
