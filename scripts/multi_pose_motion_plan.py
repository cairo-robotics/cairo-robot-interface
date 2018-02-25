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

    pose1 = [
        0.6949347211914496,
        -0.5413202179320513,
        0.3014203901950048,
        0.9416077155400848,
        -0.335109159155727,
        0.022714612387440658,
        0.02368138233746421]
    pose2 = [
        0.737768633028777,
        -0.20838165935163705,
        0.08693570083579126,
        0.942718170265355,
        -0.3322233875202301,
        0.02316207641159948,
        0.019328488105065017]
    pose3 = [
        0.7032578539410487,
        0.25775339686828397,
        0.09149120266495979,
        0.9436094934555694,
        -0.32990025449253385,
        0.022422331235321512,
        0.01625376904779616]
    poses = [pose1, pose2, pose3, pose2, pose1]
    
    plans = []
    for pose in poses:
        sawyer.set_pose_target(pose)
        plans.append(sawyer.plan())

    # for plan in plans:

    #     print plan.joint_trajectory.points

    combine_plans(plans[0], plans[1])

def combine_plans(plan1, plan2):
    points_1 = plan1.joint_trajectory.points
    points_2 = plan1.joint_trajectory.points
    end_1 = points_1[-1]
    start_2 = points_2[1]
    end = (end_1.time_from_start.secs, end_1.time_from_start.nsecs)
    start = (start_2.time_from_start.secs, start_2.time_from_start.nsecs)
    print end
    print 
    print start
    print
    seconds = end[0] + start[0]
    end = end[1] + start[1]

    nano = 1000000000

if __name__ == '__main__':
    main()
