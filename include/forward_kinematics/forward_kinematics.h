#include <ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <cairo_sawyer_interface/ForwardKinematics.h>
#include <Eigen/Core>
#include <Eigen/Geometry>



class SawyerFK{
/* a class that creats an object with a callback function to transform from
joint space to pose space using moveit and the swayer urdf */
public:
  robot_model::RobotModelPtr kinematic_model;
  robot_state::RobotStatePtr kinematic_state2;
  std::vector<double> joint_values;
  robot_state::JointModelGroup *joint_model_group;
  SawyerFK();
  bool callback(cairo_sawyer_interface::ForwardKinematics::Request &req, cairo_sawyer_interface::ForwardKinematics::Response &res);
};

