



#include  <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_visual_tools/moveit_visual_tools.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "path_constraints");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  //name of the planing group
  static const std::string PLANNING_GROUP = "right_arm";
  //movegroup interface "move group"
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);



  //move_group.startStateMonitor();

  //const robot_state::JointModelGroup *joint_model_group =
  //  move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.z = 1.0;
  target_pose1.position.x = 0.28;
  target_pose1.position.y = 0.7;
  target_pose1.position.z = 0.0;
  move_group.setPoseTarget(target_pose1);

  geometry_msgs::Pose target_pose2;
  target_pose1.orientation.z = 1.0;
  target_pose1.position.x = 0.28;
  target_pose1.position.y = 0.7;
  target_pose1.position.z = 0.1;


  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  move_group.move();

  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "right_gripper";
  ocm.header.frame_id = "base";
  ocm.orientation.z = 1.0;
  ocm.absolute_x_axis_tolerance = 0.3;
  ocm.absolute_y_axis_tolerance = 0.3;
  ocm.absolute_z_axis_tolerance = 0.3;
  ocm.weight = 0.5;


  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  move_group.setPathConstraints(test_constraints);

  move_group.setPoseTarget(target_pose2);
  move_group.setPlanningTime(10.0);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");

/*
  //the scene interface
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  //moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

  // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
  // and trajectories in Rviz as well as debugging tools such as step-by-step introspection of a script
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("odom_combined");
  visual_tools.deleteAllMarkers();

  visual_tools.loadRemoteControl();

  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 1.75; // above head of PR2
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  visual_tools.trigger();


  ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());


  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.28;
  target_pose1.position.y = -0.7;
  target_pose1.position.z = 1.0;
  move_group.setPoseTarget(target_pose1);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");




  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("next step");


*/





















  ros::shutdown();
  return 0;
}
