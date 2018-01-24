import rospy
import intera_interface
import abc


class Environment(object):

    def __init__(self):
        self.items
        self.robot
        self.constraints

    def get_item_states(self):
        items = []
        for item in self.items:
            item.append(item.get_state())
        return items

    def get_item_definitions(self):
        pass


class LFDItem(abc.ABC):

    @abc.abstractmethod
    def get_definition(self):
        pass

    @abc.abstractmethod
    def get_state(self):
        pass


class ConstraintInterface(object):

    def __init__(self, constraints):
        self._navigator = intera_interface.Navigator()
        self.constraints = constraints

    def get_state(self):
        state = {}
        for constraint in self.constraints:
            button_state = self._navigator.get_button_state(constraint["button"])
            state[constraint["name"]] = 1 if button_state != 0 else 0
        return state


class SawyerRobot(object):

    def __init__(self, side="right"):
        self._limb = intera_interface.Limb(side)
        self._cuff = intera_interface.Cuff(side)
        self._navigator = intera_interface.Navigator()
        try:
            self._gripper = intera_interface.Gripper(side)
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
