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

    end_pose = [
        0.6348590496413895,
        0.12485524636261497,
        0.19661005101541001,
        0.9993962837780159,
        0.004024747130915506,
        -0.034228490820403086,
        -0.004390876607434143]
    end_joints = sawyer.get_pose_IK_joints(end_pose)
    print end_joints
    print sawyer.checkPointValidity(end_joints)

if __name__ == '__main__':
    main()
