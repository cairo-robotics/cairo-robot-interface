#include <ros/ros.h>
#include <forward_kinematics/forward_kinematics.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "forward_kinematic_service");
  ros::NodeHandle n;

  SawyerFK sawyer;

  ros::ServiceServer service = n.advertiseService("forward_kinematic_service", &SawyerFK::callback, &sawyer);
  ROS_INFO("Ready to perform forward kinematics on trajectory path joint positions.");
  ros::spin();

  return 0;
}