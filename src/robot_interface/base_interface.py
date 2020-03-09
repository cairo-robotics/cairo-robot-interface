from abc import ABCMeta, abstractmethod


class AbstractRobotInterface:
    """
    Abstract class to program to specific interface for interacting with Moveit.
    """
    __metaclass__ = ABCMeta

    @abstractmethod
    def get_FK_pose(self, joints):
        pass

    @abstractmethod
    def get_IK_pose(self, pose):
        pass

    @abstractmethod
    def check_point_validity(self):
        pass


class AbstractMoveitInterface:
    """
    Abstract class to program to specific interface for interacting with Moveit.
    """
    __metaclass__ = ABCMeta

    @abstractmethod
    def set_planner(self, planner):
        pass

    @abstractmethod
    def get_robot_state(self):
        pass

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
    def create_robot_state(self, joints):
        pass