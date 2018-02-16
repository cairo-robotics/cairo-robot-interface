#!/usr/bin/env python2

import rospy
import argparse
import intera_interface
from intera_interface import CHECK_VERSION
from sawyer_interface.recorder import Recorder
from lfd_processor.environment import Environment, import_configuration
from lfd_processor.items import RobotFactory, ConstraintFactory
from lfd_processor.analyzer import ConstraintAnalyzer
from lfd_processor.data_io import DataExporter
from lfd_processor.alignment import DemonstrationAligner, vectorize_demonstration
from lfd_processor.analyzer import DemonstrationKeyframeLabeler


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
        '-d', '--directory', dest='directory', required=True,
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

    recorder = Recorder(args.record_rate)
    rospy.on_shutdown(recorder.stop)

    config_filepath = "./src/cairo_sawyer_interface/scripts/config.json"
    configs = import_configuration(config_filepath)

    robot_factory = RobotFactory(configs["robots"])
    constraint_factory = ConstraintFactory(configs["constraints"])

    robot = robot_factory.generate_robots()[0]
    constraints = constraint_factory.generate_constraints()

    # We only have just the one robot...for now.......
    environment = Environment(items=None, robot=robot, constraints=constraints)

    exp = DataExporter()

    print("Recording. Press Ctrl-C to stop.")
    demos = recorder.record_demonstrations(environment)

    constraint_analyzer = ConstraintAnalyzer(environment)
    for demo in demos:
        constraint_analyzer.applied_constraint_evaluator(demo.observations)

    exp = DataExporter()
    for idx, demo in enumerate(demos):
        raw_data = [obs.data for obs in demo.observations]
        exp.export_to_json(args.directory + "/raw_demonstration{}.json".format(idx), raw_data)

    # For more details about alignment, see the alignment_example.
    aligner = DemonstrationAligner(demos, vectorize_demonstration)
    aligned_demos, constraint_transitions = aligner.align()

    # Create DemosntrationkeyframeLabeler passing in the aligned demonstrations and the constraint transition
    # ordering, both of which are returned from the DemonstratinAligner object.
    keyframe_labeler = DemonstrationKeyframeLabeler(aligned_demos, constraint_transitions)

    # Call label_demontrations. The first paremeter is the average group length divisor which determines
    # the number of keyframes for that group. So if the first grouping of data before the first constraint transtion
    # has an average length of 100, then that first grouping will generate 5 keyframes (100/20 = 5). The second parameter
    # is the window size i.e. how big each keyframe size should be (+/- one depending on if odd number of elements in 
    # the grouping list per demonstration)
    labeled_demonstrations = keyframe_labeler.label_demonstrations(20, 10)

    # Export the dictionary data representation of the observations of the labeled demos.
    # Notice that the demonstrations now have a populated instance member 'labeled_observations'.
    exp = DataExporter()
    for idx, demo in enumerate(labeled_demonstrations):
        raw_data = [obs.data for obs in demo.labeled_observations]
        exp.export_to_json(args.directory + "/labeled_demonstration{}.json".format(idx), raw_data)
    print("\nDone.")


if __name__ == '__main__':
    main()
