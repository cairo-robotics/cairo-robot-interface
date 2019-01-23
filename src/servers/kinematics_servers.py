import rospy
from servers.abstract_servers import AbstractROSServer
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


class SawyerForwardKinematicsServer(AbstractROSServer):
    def __init__(self, limb="right"):
        self.ns = "ExternalTools/" + limb + "/PositionKinematicsNode/FKService"
        self.service = rospy.ServiceProxy(self.ns, SolvePositionFK)
        rospy.loginfo("Connecting to Forward Kinematics service.")
        try:
            self.service.wait_for_service()
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False

    def close(self):
        self.service.close()

    def call(self,
             joint_positions,
             joint_names=['right_j0', 'right_j1', 'right_j2', 'right_j3',
                          'right_j4', 'right_j5', 'right_j6'],
             tip_names=["right_hand"]):

        """

        Call SolvePositionFK servce

        Parameters
        ----------

        Returns
        -------
        response : SolvePositionFKResponse

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
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False

        # Check if result valid
        if (resp.isValid[0]):
            rospy.logdebug("SUCCESS - Valid Cartesian Solution Found")
            rospy.logdebug("\nFK Cartesian Solution:\n")
            rospy.logdebug("------------------")
            rospy.logdebug("Response Message:\n%s", resp)
        else:
            rospy.logerr("INVALID JOINTS - No Cartesian Solution Found.")
            return False
        return resp.pose_stamp[0].pose


class SawyerInverseKinematicsServer(AbstractROSServer):
    def __init__(self, ):
        self.service = rospy.ServiceProxy("/compute_ik", GetPositionIK)
        rospy.loginfo("Connecting to Inverse Kinematics service.")
        self.service.wait_for_service()

    def close(self):
        self.service.close()

    def call(self, pose, tip_name="/right_gripper", limb="left", use_advanced_options=False):

        """

        Call the inverse kinematics service "/compute_ik" to get IK for a give Pose (must be PoseStamped).

        Parameters
        ----------


        Returns
        -------
        response : GetPositionIKResponse
            The GetPositionIKRespponse response from the /compute_ik service
        """

        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        ikreq = SolvePositionIKRequest()
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        poses = pose
        poses.hdr = hdr
        # Add desired pose for inverse kinematics
        ikreq.pose_stamp.append(poses[limb])
        # Request inverse kinematics from base to "right_hand" link
        ikreq.tip_names.append(tip_name)

        if (use_advanced_options):
            # Optional Advanced IK parameters
            rospy.loginfo("Running Advanced IK Service Client example.")
            # The joint seed is where the IK position solver starts its optimization
            ikreq.seed_mode = ikreq.SEED_USER
            seed = JointState()
            seed.name = ['right_j0', 'right_j1', 'right_j2', 'right_j3',
                         'right_j4', 'right_j5', 'right_j6']
            seed.position = [0.7, 0.4, -1.7, 1.4, -1.1, -1.6, -0.4]
            ikreq.seed_angles.append(seed)

            # Once the primary IK task is solved, the solver will then try to bias the
            # the joint angles toward the goal joint configuration. The null space is 
            # the extra degrees of freedom the joints can move without affecting the
            # primary IK task.
            ikreq.use_nullspace_goal.append(True)
            # The nullspace goal can either be the full set or subset of joint angles
            goal = JointState()
            goal.name = ['right_j1', 'right_j2', 'right_j3']
            goal.position = [0.1, -0.3, 0.5]
            ikreq.nullspace_goal.append(goal)
            # The gain used to bias toward the nullspace goal. Must be [0.0, 1.0]
            # If empty, the default gain of 0.4 will be used
            ikreq.nullspace_gain.append(0.4)
        else:
            rospy.loginfo("Running Simple IK Service Client example.")

        try:
            rospy.wait_for_service(ns, 5.0)
            resp = iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False

        # Check if result valid, and type of seed ultimately used to get solution
        if (resp.result_type[0] > 0):
            seed_str = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                       }.get(resp.result_type[0], 'None')
            rospy.loginfo("SUCCESS - Valid Joint Solution Found from Seed Type: %s" % (seed_str,))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            rospy.loginfo("\nIK Joint Solution:\n%s", limb_joints)
            rospy.loginfo("------------------")
            rospy.loginfo("Response Message:\n%s", resp)
            return resp


class MoveItForwardKinematicsServer():
    def __init__(self):
        self.service = rospy.ServiceProxy("/compute_fk", GetPositionFK)
        rospy.loginfo("Connecting to Forward Kinematics service.")
        self.service.wait_for_service()

    def close(self):
        self.service.close()

    def call(self,
             positions,
             joint_names=['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6'],
             links=["right_hand"],
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
        response : GetPositionFKResponse
            The GetPositionFKResponse response from the /compute_fk service
        """
        request = GetPositionFKRequest()
        request.fk_link_names = links
        request.robot_state.joint_state.name = joint_names
        request.robot_state.joint_state.position = positions
        request.header.frame_id = frame_id
        response = self.service.call(request)
        return response.pose_stamped[0].pose


class MoveItInverseKinematicsServer():
    def __init__(self):
        self.service = rospy.ServiceProxy("/compute_ik", GetPositionIK)
        rospy.loginfo("Connecting to Inverse Kinematics service.")
        self.service.wait_for_service()

    def close(self):
        self.ik_srv.close()

    def call(self, pose, group_name="right_arm", link="/right_hand", avoid_collisions=True, attempts=10):

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
        response : GetPositionIKResponse
            The GetPositionIKRespponse response from the /compute_ik service
        """

        request = GetPositionIKRequest()
        request.ik_request.group_name = group_name
        request.ik_request.avoid_collisions = avoid_collisions
        request.ik_request.ik_link_name = link
        if isinstance(pose, PoseStamped):
            request.ik_request.pose_stamped = pose
        else:
            rospy.logerr("Pose must be of type PoseStamped for the InverseKinematics call() method!")
            return
        request.ik_request.attempts = attempts
        response = self.service.call(request)
        rospy.logwarn("Sent: " + str(request))
        return response

class RobotStateValidityServer(AbstractROSServer):
    def __init__(self):
        self.service = rospy.ServiceProxy("/check_state_validity", GetStateValidity)
        rospy.loginfo("Connecting to State Validity service")
        self.service.wait_for_service()

    def close(self):
        self.sv_srv.close()

    def call(self, robot_state, group_name="right_arm"):
        """
        Given a robot state and a group name, caluclate whether or not the state is valid.
        Parameters
        ----------
        robot_state : RobotState
            RobotState msg for which to check validity.
        group_name : string
            Name of the group (ex: "right_arm") on which to check for validity.
        Returns
        -------
        response : GetStateValidityResponse
            The GetStateValidityResponse response from the /check_state_validity service
        """
        request = GetStateValidityRequest()
        request.robot_state = robot_state
        request.group_name = group_name
        try:
            response = self.service.call(request)
            return response
        except ServiceException as e:
            rospy.logwarn(e)
            return None