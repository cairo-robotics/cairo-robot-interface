import rospy, tf2_ros
from servers.abstract_servers import AbstractROSServer


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
