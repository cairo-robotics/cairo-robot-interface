import rospy
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest, GetPositionIK, \
                            GetPositionIKRequest, GetStateValidity, GetStateValidityRequest
from geometry_msgs.msg import PoseStamped


class ForwardKinematicsServer():
    def __init__(self):
        self.service = rospy.ServiceProxy("/compute_fk", GetPositionFK)
        rospy.loginfo("Connecting to Forward Kinematics service.")
        self.service.wait_for_service()

    def close(self):
        self.service.close()

    def call(self,
             positions,
             joint_names=['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6'],
             links=["right_gripper"],
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
        return response


class InverseKinematicsServer():
    def __init__(self):
        self.service = rospy.ServiceProxy("/compute_ik", GetPositionIK)
        rospy.loginfo("Connecting to Inverse Kinematics service.")
        self.service.wait_for_service()

    def close(self):
        self.ik_srv.close()

    def call(self, pose, group_name="right_arm", link="/right_gripper", avoid_collisions=True, attempts=10):

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


class RobotStateValidityServer():
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
        response = self.service.call(request)
        return response
