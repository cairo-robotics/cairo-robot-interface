import rospy
import intera_interface


class SawyerRobot(object):

    def __init__(self, robot_id, upright_position):
        self.id = robot_id
        self.upright_position = upright_position
        self._limb = intera_interface.Limb("right")
        self._cuff = intera_interface.Cuff("right")
        self._navigator = intera_interface.Navigator()
        try:
            self._gripper = intera_interface.Gripper("right")
            rospy.loginfo("Electric gripper detected.")
            if self._gripper.has_error():
                rospy.loginfo("Gripper error...rebooting.")
                self._gripper.reboot()
            if not self._gripper.is_calibrated():
                rospy.loginfo("Calibrating gripper.")
                self._gripper.calibrate()
        except Exception as e:
            self._gripper = None
            rospy.loginfo("No electric gripper detected.")

    def get_state(self):
        state = {}
        joints_right = self._limb.joint_names()
        pose_right = self._limb.endpoint_pose()
        state['PoseX'] = pose_right['position'].x
        state['PoseY'] = pose_right['position'].y
        state['PoseZ'] = pose_right['position'].z
        state['OrienX'] = pose_right['orientation'].x
        state['OrienY'] = pose_right['orientation'].y
        state['OrienZ'] = pose_right['orientation'].z
        state['OrienW'] = pose_right['orientation'].w
        state['gripper'] = self._gripper.get_position()
        for j in joints_right:
            state[j] = self._limb.joint_angle(j)
        return state
