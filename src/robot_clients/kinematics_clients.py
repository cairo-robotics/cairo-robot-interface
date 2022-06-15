import rospy
from robot_clients.abstract_clients import AbstractROSClient
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest, GetPositionIK, \
    GetPositionIKRequest, GetStateValidity, GetStateValidityRequest
from rospy.service import ServiceException
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

from intera_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
    SolvePositionFK,
    SolvePositionFKRequest
)


class ForwardKinematicsResponse():

    def __init__(self, valid, pose=None):
        self.valid = valid
        self.pose = pose


class InverseKinematicsResponse():

    def __init__(self, valid, joint_state=None):
        self.valid = valid
        self.joint_state = joint_state


class SawyerForwardKinematicsClient(AbstractROSClient):
    """
    Class that creates a ROS client in order to make service calls to calculate
    forward kinematics  give a set of joint positions. This client connects to
    Rethink RObotics Intera Interface FKService service.

    Attributes
    ----------
    ns : str
        The Service name space
    service : ServiceProxy
        The ROS Service proxy object
    """
    def __init__(self, limb="right"):
        """
        Parameters
        ----------
        limb : str
            The limb for the service name; defaults to 'right'
        """
        self.ns = "ExternalTools/" + limb + "/PositionKinematicsNode/FKService"
        self.service = rospy.ServiceProxy(self.ns, SolvePositionFK, persistent=True)
        rospy.loginfo("Connecting to Forward Kinematics service.")
        try:
            self.service.wait_for_service()
        except ServiceException as e:
            rospy.logerr("Service call failed: {}".format(e))
        except rospy.ROSException as e:
            rospy.logerr("General ROS Exception: {}".format(e))

    def close(self):
        """
        Closes the connection to the service
        """
        self.service.close()

    def call(self,
             joint_positions,
             joint_names=['right_j0', 'right_j1', 'right_j2', 'right_j3',
                          'right_j4', 'right_j5', 'right_j6'],
             tip_names=["right_gripper_tip"]):
        """
        Call 'FKService' service

        Parameters
        ----------
        joint_positions : list
            List of joint angle values (radians) on which to perform forward kinematics.
        joint_names : list
            List of joint names for the corresponding joint positions.
        tip_names : list
            List of tip names to which the forward kinematics are calculated.

        Returns
        -------
        : ForwardKinematicsResponse
            Returns a ForwardKinematicsResponse built from the SolvePositionFKResponse object returned by the service.
        """
        fkreq = SolvePositionFKRequest()
        joints = JointState()
        joints.name = joint_names
        joints.position = joint_positions
        # Add desired pose for forward kinematics
        fkreq.configuration.append(joints)
        # Request forward kinematics from base to "right_hand" link
        fkreq.tip_names = tip_names

        try:
            rospy.wait_for_service(self.ns, 5.0)
            resp = self.service(fkreq)
        except ServiceException as e:
            rospy.logerr("Service call failed: {}".format(e))
            return ForwardKinematicsResponse(False, None)
        except rospy.ROSException as e:
            rospy.logerr("General ROS Exception: {}".format(e))
            return ForwardKinematicsResponse(False, None)
            
        # Check if result valid
        if (resp.isValid[0]):
            rospy.logdebug("SUCCESS - Valid Cartesian Solution Found")
            rospy.logdebug("\nFK Cartesian Solution:\n")
            rospy.logdebug("------------------")
            rospy.logdebug("Response Message:\n%s", resp)
            return ForwardKinematicsResponse(True, resp.pose_stamp[0].pose)
        else:
            rospy.logerr("INVALID JOINTS - No Cartesian Solution Found.")
            return ForwardKinematicsResponse(False, resp.pose_stamp[0].pose)


class SawyerInverseKinematicsClient(AbstractROSClient):
    """
    Class that creates a ROS client in order to make service calls to calculate
    inverse kinematics given a set of joint positions. This class makes use of
    Rethink Robotics Intera Interface service IKService.

    Attributes
    ----------
    ns : str
        The Service name space
    service : ServiceProxy
        The ROS Service proxy object
    """
    def __init__(self, limb="right"):
        """
        Parameters
        ----------
        limb : str
            The limb for the service name; defaults to 'right'
        """
        self.ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self.service = rospy.ServiceProxy(ns, SolvePositionIK, persistent=True)
        rospy.loginfo("Connecting to Inverse Kinematics service.")
        try:
            self.service.wait_for_service()
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % (e,))
        except rospy.ROSException as e:
            rospy.logerr("General ROS Exception: %s" % (e,))

    def close(self):
        """
        Closes the connection to the service
        """
        self.service.close()

    def call(self, pose, tip_name="/right_gripper_tip"):
        """
        Call 'IKService' service


        Parameters
        ----------
        pose : geometry_msgs.PosedStamped
            The pose on which to perform Inverse Kinematics.
        tip_name : str
            The ending frame name in the transform tree from which to compute Inverse Kinematics.

        Returns
        -------
        response : InverseKinematicsResponse
            The InverseKinematicsResponse response built from the IKService service response.
        """

        ikreq = SolvePositionIKRequest()
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        poses = pose
        poses.hdr = hdr
        # Add desired pose for inverse kinematics
        ikreq.pose_stamp.append(poses[self.limb])
        # Request inverse kinematics from base to "right_hand" link
        ikreq.tip_names.append(tip_name)

        try:
            resp = self.service(ikreq)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % (e,))
        except rospy.ROSException as e:
            rospy.logerr("General ROS Exception: %s" % (e,))

        # Check if result valid, and type of seed ultimately used to get solution
        if (resp.result_type[0] > 0):
            seed_str = {
                ikreq.SEED_USER: 'User Provided Seed',
                ikreq.SEED_CURRENT: 'Current Joint Angles',
                ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
            }.get(resp.result_type[0], 'None')
            rospy.logdebug(
                "SUCCESS - Valid Joint Solution Found from Seed Type: %s" % (seed_str,))
            # Format solution into Limb API-compatible dictionary
            rospy.logdebug("\nIK Joint Solution:\n%s", limb_joints)
            return InverseKinematicsResponse(True, resp.joints[0])
        else:
            return InverseKinematicsResponse(False, None)


class MoveitForwardKinematicsClient(AbstractROSClient):
    """
    Class that creates a ROS client in order to make service calls to calculate
    forward kinematics  give a set of joint positions. This client connects to
    MoveIt's '/compute_fk' service.

    Attributes
    ----------
    service : ServiceProxy
        The ROS Service proxy object
    """
    def __init__(self):
        self.service = rospy.ServiceProxy("/compute_fk", GetPositionFK, persistent=True)
        rospy.loginfo("Connecting to Forward Kinematics service.")
        try:
            self.service.wait_for_service()
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % (e,))
        except rospy.ROSException as e:
            rospy.logerr("General ROS Exception: %s" % (e,))

    def close(self):
        """
        Closes the connection to the service
        """
        self.service.close()

    def call(self,
             positions,
             joint_names=['right_j0', 'right_j1', 'right_j2',
                          'right_j3', 'right_j4', 'right_j5', 'right_j6'],
             links=["right_gripper_tip"],
             frame_id="/base"):
        """
        Call the forward kinematics service "/compute_fk" to get FK of a joint configuration.

        Parameters
        ----------
        links : list
            list of links that we want to get the forward kinematics from.
        joint_names : list
            List of strings with the joint names.
        positions : list
            List of doubles representing the the position of the joints.
        frame_id : string
            Reference frame.
        Returns
        -------
        : ForwardKinematicsResponse
            The ForwardKinematicsResponse response built from from the /compute_fk service response.
        """
        request = GetPositionFKRequest()
        request.fk_link_names = links
        request.robot_state.joint_state.name = joint_names
        request.robot_state.joint_state.position = positions
        request.header.frame_id = frame_id
        response = self.service.call(request)

        # check if there is a moveit failure.
        if response.error_code == 99999:
            return ForwardKinematicsResponse(False, None)
        else:
            return ForwardKinematicsResponse(True, response.pose_stamped[0].pose)


class MoveitInverseKinematicsClient(AbstractROSClient):
    """
    Class that creates a ROS client in order to make service calls to calculate
    inverse kinematics  give a set of joint positions. This client connects to
    MoveIt's '/compute_ik' service.

    Attributes
    ----------
    service : ServiceProxy
        The ROS Service proxy object
    """
    def __init__(self):
        self.service = rospy.ServiceProxy("/compute_ik", GetPositionIK, persistent=True)
        rospy.loginfo("Connecting to Inverse Kinematics service.")
        try:
            self.service.wait_for_service()
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % (e,))
        except rospy.ROSException as e:
            rospy.logerr("General ROS Exception: %s" % (e,))

    def close(self):
        """
        Closes the connection to the service
        """
        self.ik_srv.close()

    def call(self, pose, group_name="right_arm", link="/right_gripper_tip", avoid_collisions=True, attempts=10):
        """
        Call the inverse kinematics service "/compute_ik" to get IK for a give Pose (must be PoseStamped).

        Parameters
        ----------
        group_name : string
            Name of the group (ex: "right_arm") on which to perform IK.
        link : string
            Name of link that the Pose represents.
        pose  : PoseStamped
            Pose (must include frame_id) to caluclate the IK.
        avoid_collisions : Bool
            True for IK results that avoid collisions.
        attempts : int
            Number of attempts to try before deemed at failure.
        Returns
        -------
        response : InverseKinematicsResponse
            The InverseKinematicsResponse response built from the /compute_ik service response.
        """
        request = GetPositionIKRequest()
        request.ik_request.group_name = group_name
        request.ik_request.avoid_collisions = avoid_collisions
        request.ik_request.ik_link_name = link
        if isinstance(pose, PoseStamped):
            request.ik_request.pose_stamped = pose
        elif isinstance(pose, Pose):
            ps = PoseStamped()
            ps.pose = pose
            request.ik_request.pose_stamped = ps
        else:
            rospy.logerr(
                "Pose must be of type PoseStamped or Pose of geometry_msgs for the InverseKinematics call() method!")
            raise ValueError("Pose must be of type PoseStamped or Pose of geometry_msgs for the InverseKinematics call() method!")
        # request.ik_request.attempts = attempts
        print(request)
        response = self.service.call(request)
        print(response)
        if response.error_code == 99999:
            return InverseKinematicsResponse(False, None)
        else:
            return InverseKinematicsResponse(True, response.solution.joint_state)


class MoveitRobotStateValidityClient(AbstractROSClient):
    """
    Class that creates a ROS client in order to make service calls to check for point validity
    given a robot state. This client connects to MoveIt's '/check_state_validity' service.

    Attributes
    ----------
    service : ServiceProxy
        The ROS Service proxy object
    """
    def __init__(self):
        self.service = rospy.ServiceProxy(
            "/check_state_validity", GetStateValidity, persistent=True)
        rospy.loginfo("Connecting to State Validity service")
        self.service.wait_for_service()

    def close(self):
        """
        Closes the connection to the service
        """
        self.sv_srv.close()

    def call(self, robot_state, group_name="right_arm"):
        """
        Given a robot state and a group name, calculate whether or not the state is valid.
        Parameters
        ----------
        robot_state : RobotState
            RobotState for which to check validity.
        group_name : string
            Name of the group (ex: "right_arm") on which to check for validity.
        Returns
        -------
        response : GetStateValidityResponse
            The GetStateValidityResponse response from the /check_state_validity service.
        """
        request = GetStateValidityRequest()
        request.robot_state = robot_state
        request.group_name = group_name
        try:
            response = self.service.call(request)
            return response.valid
        except ServiceException as e:
            rospy.logwarn(e)
            return None
