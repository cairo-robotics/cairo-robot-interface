#!/usr/bin/env python2

import rospy
import intera_interface
from intera_interface import CHECK_VERSION
from lfd_processor.environment import Environment, Observation, import_configuration
from lfd_processor.items import RobotFactory, ConstraintFactory
from lfd_processor.analyzer import MotionPlanAnalyzer
from sawyer_interface.moveit_interface import SawyerMoveitInterface

from moveit_msgs.msg import Constraints, OrientationConstraint, PositionConstraint
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header

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

    start_pose = [0.698798449415, 
                  -0.589663719382, 
                  0.21863680518,  
                  0.731252055018, 
                  -0.0600575030253, 
                  0.67209547727, 
                  0.0997556905436]

    end_pose = [0.671136779022,
                0.474330130145,
                0.210298146562,
                0.670733323382,
                -0.168075208186,
                0.704857726658,
                0.158250176837]

    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = '/base'

    quat = Quaternion()

    quat.x = 0.728688159145
    quat.y = -0.0585758400832
    quat.z = 0.675117364291
    quat.w = 0.0989898081307

    o_constraint = OrientationConstraint()
    o_constraint.orientation = quat
    o_constraint.header = header
    o_constraint.link_name = "right_gripper"
    o_constraint.absolute_z_axis_tolerance = .34
    o_constraint.absolute_x_axis_tolerance = .34
    o_constraint.absolute_y_axis_tolerance = 0.05
    o_constraint.weight = .25

    constraints = Constraints()
    constraints.orientation_constraints = [o_constraint]

    sawyer.move_to_pose_target(start_pose)
    sawyer.group.set_path_constraints(constraints)
    sawyer.set_pose_target(end_pose)
    plan = sawyer.plan()
    print plan
    sawyer.execute(plan)
    print sawyer.robot.get_root_link()
    print sawyer.robot.get_link_names()
if __name__ == '__main__':
    main()
