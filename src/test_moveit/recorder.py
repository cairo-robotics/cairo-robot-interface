import rospy
import json
from lfd_environment.interfaces import SawyerRobot, ConstraintInterface


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
        self.constraint_state = ConstraintInterface(self.constraints)
        self.robot_state = SawyerRobot(side=side)

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
