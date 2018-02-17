#include <forward_kinematics/forward_kinematics.h>

SawyerFK::SawyerFK(){
  //TODO constructor and initializations should be seperated

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  //add the kinematic model
  kinematic_model = robot_model_loader.getModel();
  //create kinematic stat model TODO find better method of this
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  // reassigning this? amatuer hour
  kinematic_state2 = kinematic_state;
  //joint model group for assiging to incoming joint states
  joint_model_group = kinematic_model->getJointModelGroup("right_arm");
}


bool SawyerFK::callback(cairo_sawyer_interface::ForwardKinematics::Request  &req, cairo_sawyer_interface::ForwardKinematics::Response &res) {
  //TODO remove from callback method and create jointToFK method
  //creat joint vector for incoming joint positions
  std::vector<double> position;
  //joint 0
  position.push_back(req.joint_names.data[0]);
  //head joint
  position.push_back(.99);
  //joint 1-6
  for(std::size_t i = 1; i < req.joint_names.data.size(); i++){
    position.push_back(req.joint_names.data[i]);
  }
  //set variable joint positions
  kinematic_state2->setVariablePositions(position);
  //perform forward kinematics using kinematic state model
  const Eigen::Affine3d &end_effector_state = kinematic_state2->getGlobalLinkTransform("right_gripper_base");

  //convert to pose for pusblishing
  geometry_msgs::Pose pose;

  pose.position.x = end_effector_state.translation()[0];
  pose.position.y = end_effector_state.translation()[1];
  pose.position.z = end_effector_state.translation()[2];

  pose.orientation.x = Eigen::Quaterniond(end_effector_state.rotation()).x();
  pose.orientation.y = Eigen::Quaterniond(end_effector_state.rotation()).y();
  pose.orientation.z = Eigen::Quaterniond(end_effector_state.rotation()).z();
  pose.orientation.w = Eigen::Quaterniond(end_effector_state.rotation()).w();

  res.end_effector_pose = pose;

  return true;
}