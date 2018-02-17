/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta */

#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include "std_msgs/Float32MultiArray.h"

#include <Eigen/Core>
#include <Eigen/Geometry>


class SawyerFK{
/* a class that creats an object with a callback function to transform from
joint space to pose space using moveit and the swayer urdf */
public:
  robot_model::RobotModelPtr kinematic_model;
  robot_state::RobotStatePtr kinematic_state2;
  ros::Publisher pub;
  std::vector<double> joint_values;
  robot_state::JointModelGroup *joint_model_group;
  SawyerFK(ros::NodeHandle);
  void callback(const std_msgs::Float32MultiArray::ConstPtr&);

};

SawyerFK::SawyerFK(ros::NodeHandle n){
  //TODO constructor and initializations should be seperated
  //create pose publisher
  pub = n.advertise<geometry_msgs::Pose>("fk_pose", 1);
  //load the swayer model
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  //add the kinematic model
  kinematic_model = robot_model_loader.getModel();
  //create kinematic stat model TODO find better method of this
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  //reassigning this? amatuer hour
  kinematic_state2 = kinematic_state;
  //joint model group for assiging to incoming joint states
  joint_model_group = kinematic_model->getJointModelGroup("right_arm");
}

void SawyerFK::callback(const std_msgs::Float32MultiArray::ConstPtr& msg){
  //TODO remove from callback method and create jointToFK method
  //creat joint vector for incoming joint positions
  std::vector<double> position;
  //joint 0
  position.push_back(msg->data[0]);
  //head joint
  position.push_back(.99);
  //joint 1-6
  for(std::size_t i = 1; i < msg->data.size(); i++){
    position.push_back(msg->data[i]);
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

  pub.publish(pose);
}


int main(int argc, char **argv)
{
  //TODO remove SawyerFK class from node file
  ros::init(argc, argv, "forward_kinematics_server");
  ros::NodeHandle n;

  SawyerFK sawyer = SawyerFK(n);
  ros::Subscriber sub = n.subscribe("fk_joints", 10, &SawyerFK::callback, &sawyer);

  ros::spin();
  return 0;
}
