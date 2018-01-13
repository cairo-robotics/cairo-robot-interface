#!/usr/bin/env python2
'''record pose information into csv file'''

import argparse
import rospy

import intera_interface
from intera_interface import CHECK_VERSION

class FullRecorder(object):
    def __init__(self, filename, rate, side="right"):
        """
        Records joint data to a file at a specified rate.
        """
        self.gripper_name = '_'.join([side, 'gripper'])
        self._filename = filename
        self._raw_rate = rate
        self._rate = rospy.Rate(rate)
        self._start_time = rospy.get_time()
        self._done = False

        self._limb_right = intera_interface.Limb(side)
        try:
            self._gripper = intera_interface.Gripper(side)
            rospy.loginfo("Electric gripper detected.")
        except Exception as e:
            self._gripper = None
            rospy.loginfo("No electric gripper detected.")

        if self._gripper:
            if self._gripper.has_error():
                self._gripper.reboot()
            if not self._gripper.is_calibrated():
                self._gripper.calibrate()

        self._cuff = intera_interface.Cuff(side)

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

    def record(self):
        """
        Records the current joint positions to a csv file if outputFilename was
        provided at construction this function will record the latest set of
        joint angles in a csv format.

        If a file exists, the function will overwrite existing file.
        """
        if self._filename:
            pose_right = ['PoseX', 'PoseY', 'PoseZ', 'OrienX', 'OrienY', 'OrienZ', 'OrienW']
            joints_right = self._limb_right.joint_names()
            with open(self._filename, 'w') as f:
                f.write('time,')
                f.write(','.join([p for p in pose_right])+ ',')
                f.write('right_gripper,')
                f.write(','.join([j for j in joints_right]) + '\n')
                while not self.done():
                    if self._gripper:
                        if self._cuff.upper_button():
                            self._gripper.open()
                        elif self._cuff.lower_button():
                            self._gripper.close()
                    if self._cuff.cuff_button():
                        pose_right = self._limb_right.endpoint_pose()
                        angles_right = [self._limb_right.joint_angle(j)
                                        for j in joints_right]
                        pose_array = [0] *7
                        pose_array[0] = pose_right['position'].x
                        pose_array[1] = pose_right['position'].y
                        pose_array[2] = pose_right['position'].z
                        pose_array[3] = pose_right['orientation'].x
                        pose_array[4] = pose_right['orientation'].y
                        pose_array[5] = pose_right['orientation'].z
                        pose_array[6] = pose_right['orientation'].w
                        f.write("%f, " % (self._time_stamp(),))
                        f.write(', '.join([str(x) for x in pose_array]) + ', ')
                        if self._gripper:
                            f.write(str(self._gripper.get_position()) + ', ')
                        else:
                            f.write('0.0, ')
                        f.write(', '.join([str(x) for x in angles_right]) + '\n')

                    self._rate.sleep()

def main():
    """Pose Recorder

    Record timestamped pose to a file for
    later play back.

    Run this example while moving the robot's arm to record a
    time series of end effector pose positions to a
    new csv file with the provided *filename*. This example can
    be run in parallel with any other example or standalone
    (moving the arms in zero-g mode).

    You can later play the movements back using one of the
    *_file_playback examples.
    """
    epilog = """
    Related examples:
    joint_position_file_playback.py; joint_trajectory_file_playback.py.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-f', '--file', dest='filename', required=True,
        help='the file name to record to'
    )
    parser.add_argument(
        '-r', '--record-rate', type=int, default=100, metavar='RECORDRATE',
        help='rate at which to record (default: 100)'
    )
    args = parser.parse_args(rospy.myargv()[1:])


    print("Initializing node... ")
    rospy.init_node("sdk_joint_recorder")
    print("Getting robot state... ")
    rs = intera_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()

    recorder = FullRecorder(args.filename, args.record_rate)
    rospy.on_shutdown(recorder.stop)



    print("Recording. Press Ctrl-C to stop.")
    recorder.record()

    print("\nDone.")

if __name__ == '__main__':
    main()
