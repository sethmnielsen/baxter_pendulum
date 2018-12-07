/*
 * Copyright 2017 James Jackson Brigham Young University
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include "pose_plugin.h"
#include "gazebo_compat.h"

namespace baxter_description
{

PosePlugin::~PosePlugin() {
#if GAZEBO_MAJOR_VERSION >=8
  updateConnection_.reset();
#else
  gazebo::event::Events::DisconnectWorldUpdateBegin(updateConnection_);
#endif
  nh_.shutdown();
}


void PosePlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  if (!ros::isInitialized())
  {
    ROS_FATAL("A ROS node for Gazebo has not been initialized, unable to load pose plugin");
    return;
  }
  ROS_INFO("Loaded the pose plugin");

  //
  // Configure Gazebo Integration
  //

  model_ = _model;
  world_ = model_->GetWorld();

  namespace_.clear();

  //
  // Get elements from the robot urdf/sdf file
  //

  if (_sdf->HasElement("namespace"))
    namespace_ = _sdf->GetElement("namespace")->Get<std::string>();
  else
    ROS_ERROR("[pose_plugin] Please specify a namespace.");

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    ROS_ERROR("[pose_plugin] Please specify a linkName.");

  link_ = model_->GetLink(link_name_);
  ROS_INFO("Loaded the pose plugin");
  if (link_ == nullptr)
  {
    gzthrow("[pose_plugin] Couldn't find specified link \"" << link_name_ << "\".");
  }
  

  //
  // ROS Node Setup
  //

  nh_ = ros::NodeHandle(namespace_);
  nh_private_ = ros::NodeHandle(namespace_ + "/pose");

  // load params from rosparam server
  transform_pub_topic_ = nh_private_.param<std::string>("transform_topic", "transform");
  pose_pub_topic_ = nh_private_.param<std::string>("pose_topic", "pose");

  // ROS Publishers
  pose_NWU_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(pose_pub_topic_+ "/NWU", 10);

  // Listen to the update event. This event is broadcast every simulation iteration.
  this->updateConnection_ = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&PosePlugin::OnUpdate, this, std::placeholders::_1));
}


void PosePlugin::OnUpdate(const gazebo::common::UpdateInfo& _info) {
  // C denotes child frame, P parent frame, and W world frame.
  // Further C_pose_W_P denotes pose of P wrt. W expressed in C.
  GazeboPose inertial_pose = GET_WORLD_POSE(link_);

  geometry_msgs::PoseStamped pose_NWU;
  pose_NWU.header.stamp.sec = (GET_SIM_TIME(world_)).sec;
  pose_NWU.header.stamp.nsec = (GET_SIM_TIME(world_)).nsec;

  // Set the NWU pose and transform messages
  pose_NWU.pose.position.x = GET_X(GET_POS(inertial_pose));
  pose_NWU.pose.position.y = GET_Y(GET_POS(inertial_pose));
  pose_NWU.pose.position.z = GET_Z(GET_POS(inertial_pose));
  pose_NWU.pose.orientation.w = GET_W(GET_ROT(inertial_pose));
  pose_NWU.pose.orientation.x = GET_X(GET_ROT(inertial_pose));
  pose_NWU.pose.orientation.y = GET_Y(GET_ROT(inertial_pose));
  pose_NWU.pose.orientation.z = GET_Z(GET_ROT(inertial_pose));
  pose_NWU_pub_.publish(pose_NWU);

}

GZ_REGISTER_MODEL_PLUGIN(PosePlugin);
}
