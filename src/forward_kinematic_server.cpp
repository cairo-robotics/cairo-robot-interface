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

//temporary for debugging
#include <sstream>



class SawyerFK{
  //robot_model_loader::RobotModelLoader robot_model_loader("robot_description");


public:
  robot_model::RobotModelPtr kinematic_model;
  robot_state::RobotStatePtr kinematic_state2;
  std::vector<double> joint_values;
  robot_state::JointModelGroup *joint_model_group;
  SawyerFK();
  void callback(const std_msgs::Float32MultiArray::ConstPtr&);

};

SawyerFK::SawyerFK(){
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  kinematic_model = robot_model_loader.getModel();
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  kinematic_state2 = kinematic_state;
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
  joint_model_group = kinematic_model->getJointModelGroup("right_arm");
}


void SawyerFK::callback(const std_msgs::Float32MultiArray::ConstPtr& msg){

  std::size_t count = kinematic_state2->getVariableCount();
  ROS_INFO("state count: %zu", count);
  const std::vector<std::string> &joint_names = kinematic_state2->getVariableNames();
  for(std::size_t i = 0; i < joint_names.size(); i++){
    ROS_INFO("joint names: %s", joint_names[i].c_str());
  }

  for(std::size_t i = 0; i < msg->data.size(); i++){
    ROS_INFO("send values: %f", msg->data[i]);
  }
  std::vector<double> position;
  position.push_back(msg->data[0]);
  position.push_back(.99);
  for(std::size_t i = 1; i < msg->data.size(); i++){
    position.push_back(msg->data[i]);
  }
  for(std::size_t i = 0; i < position.size(); i++){
    ROS_INFO("made position: %f", position[i]);
  }
  kinematic_state2->setVariablePositions(position);
  const Eigen::Affine3d &end_effector_state = kinematic_state2->getGlobalLinkTransform("right_gripper_base");

  ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
  ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());



  //std::size_t count =  kinematic_model->getJointModelCount();
  //kinematic_model->copyJointGroupPositions(joint_model_group, joint_values);
  //for( std::size_t i = 0; i < joint_values.size(); i++){
  //  ROS_INFO("joint value: %zu", joint_values[i]);
  //}

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "right_arm_kinematics");
  ros::NodeHandle n;

  SawyerFK sawyer = SawyerFK();
  ros::Subscriber sub = n.subscribe("fk_joints", 10, &SawyerFK::callback, &sawyer);

  ros::spin();


/*
  kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));

  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  const robot_state::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup("right_arm");

  const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();

  std::vector<double> joint_values;
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }



  kinematic_state->setToRandomPositions(joint_model_group);
  const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("right_gripper_base");

  ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
  ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());

  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }

  /*
  joint_values[0] = 1.57;
  kinematic_state->setJointGroupPositions(joint_model_group, joint_values);

  ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

  kinematic_state->enforceBounds();
  ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));


  kinematic_state->setToRandomPositions(joint_model_group);
  const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("r_wrist_roll_link");

  ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
  ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());
  */
  //ros::shutdown();
  return 0;
}
