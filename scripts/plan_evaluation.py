#!/usr/bin/env python2

import rospy
import intera_interface
from intera_interface import CHECK_VERSION
from lfd_processor.environment import Environment, Observation, import_configuration
from lfd_processor.items import RobotFactory, ConstraintFactory
from lfd_processor.analyzer import MotionPlanAnalyzer
from sawyer_interface.moveit_interface import SawyerMoveitInterface


def main():

    print("Initializing node... ")
    rospy.init_node("plan_evaluation")
    print("Getting robot state... ")
    rs = intera_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()

    config_filepath = "./src/cairo_sawyer_interface/scripts/config.json"
    configs = import_configuration(config_filepath)

    robot_factory = RobotFactory(configs["robots"])
    constraint_factory = ConstraintFactory(configs["constraints"])

    robot = robot_factory.generate_robots()[0]
    constraints = constraint_factory.generate_constraints()

    # We only have just the one robot...for now.......
    environment = Environment(items=None, robot=robot, constraints=constraints)

    sawyer = SawyerMoveitInterface()
    mp_analyzer = MotionPlanAnalyzer(environment)

    start_joint_positions = [0.1799765625, 0.327591796875, -1.8281484375, -1.6291728515625, 2.9697978515625, -1.9278564453125, 3.608435546875]
    end_position = [-2.48540625, 0.5464658203125, -2.8478037109375, -2.1630263671875, 1.3021298828125, -0.5500009765625, 3.9815615234375]
    end_position2 = [3.012658203125, -3.6015283203125, 1.9439833984375, 2.7545927734375, -1.379212890625, -0.1674365234375, 1.9711083984375]

    end_pose = [
        0.6436699526528915,
        0.12474898592092336,
        0.3111318016210859,
        0.9993248398773156,
        0.00280865335123008,
        -0.03621439175601021,
        -0.0055221101286452015]

    sawyer.move_to_joint_target(start_joint_positions)
    print sawyer.get_end_effector_pose(start_joint_positions)
    # sawyer.set_joint_start(start_joint_positions)
    sawyer.set_joint_target(end_position2)
    # # sawyer.set_pose_target(end_pose)
    plan = sawyer.plan()
    plan_observations = []
    for pose in sawyer.convert_plan_to_dict(plan):
        data = {
            "robot": pose
        }
        data["robot"]["id"] = 1
        plan_observations.append(Observation(data))
    if mp_analyzer.evaluate_plan([1], plan_observations) is True:
        sawyer.execute(plan)

if __name__ == '__main__':
    main()
