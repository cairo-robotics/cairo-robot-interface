import rospy
from lfd_processor.environment import Observation, Demonstration


class Recorder(object):

    def __init__(self, rate, side="right"):
        """
        Records joint data to a file at a specified rate.
        """
        self._raw_rate = rate
        self._rate = rospy.Rate(rate)
        self._start_time = rospy.get_time()
        self._done = False

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

    def record_demonstrations(self, environment):
        """
        Records the current joint positions to a csv file if outputFilename was
        provided at construction this function will record the latest set of
        joint angles in a csv format.

        If a file exists, the function will overwrite existing file.
        """
        robot = environment.robot

        demonstrations = []

        while not self.done():
            observations = []
            if environment.robot._navigator.get_button_state("right_button_ok") == 1:
                rospy.loginfo("Caputing new demonstration.")
                while True:
                    if robot._gripper:
                        if robot._cuff.upper_button():
                            robot._gripper.open()
                        elif robot._cuff.lower_button():
                            robot._gripper.close()
                    data = {
                        "time": self._time_stamp(),
                        "robot": environment.get_robot_state(),
                        "items": environment.get_item_states(),
                        "triggered_constraints": environment.check_constraint_triggers()
                    } 
                    observation = Observation(data)
                    observations.append(observation)
                    self._rate.sleep()
                    if environment.robot._navigator.get_button_state("right_button_ok") == 3:
                        demonstrations.append(Demonstration(observations))
                        rospy.loginfo("Demonstration captured!")
                        break
            if environment.robot._navigator.get_button_state("right_button_ok") == 2:
                self.stop()
        return demonstrations
