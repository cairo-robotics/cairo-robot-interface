import rospy
import json
import intera_interface


class ConstraintState(object):

    def __init__(self, constraints):
        self._navigator = intera_interface.Navigator()
        self.constraints = constraints

    def get_state_dict(self):
        state = {}
        for constraint in self.constraints:
            button_state = self._navigator.get_button_state(constraint["button"])
            state[constraint["name"]] = 1 if button_state != 0 else 0
        return state


class RobotState(object):

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

    def get_state_dict(self):
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


class Recorder(object):

    def __init__(self, filename, rate, side="right"):
        """
        Records joint data to a file at a specified rate.
        """
        self._filename = filename
        self._raw_rate = rate
        self._rate = rospy.Rate(rate)
        self._start_time = rospy.get_time()
        self._done = False
        self.constraints = [
                {
                    "name": "height",
                    "button": "right_button_circle"
                },
                {
                    "name": "is_upright",
                    "button": "right_button_square"
                }
            ]
        self.constraint_state = ConstraintState(self.constraints)
        self.robot_state = RobotState(side=side)

    def _time_stamp(self):
        return rospy.get_time() - self._start_time

    def stop(self):
        """
        Stop recording.
        """
        self._done = True

    def done(self):
        """
        Return whether or not recording is done.
        """
        if rospy.is_shutdown():
            self.stop()
        return self._done

    def record_demonstration(self):
        """
        Records the current joint positions to a csv file if outputFilename was
        provided at construction this function will record the latest set of
        joint angles in a csv format.

        If a file exists, the function will overwrite existing file.
        """

        if self._filename:
            with open(self._filename, 'w') as f:
                demonstration = {
                    "observations": []
                }
                while not self.done():
                    if self.robot_state._gripper:
                        if self.robot_state._cuff.upper_button():
                            self.robot_state._gripper.open()
                        elif self.robot_state._cuff.lower_button():
                            self.robot_state._gripper.close()
                    observation = {
                        "time": self._time_stamp(),
                        "robot_state": self.robot_state.get_state_dict(),
                        "constraint_state": self.constraint_state.get_state_dict()
                    }
                    demonstration["observations"].append(observation)
                    self._rate.sleep()
                json.dump(demonstration, f,  indent=4, sort_keys=True)
