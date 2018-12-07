/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
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


#ifndef BAXTER_DESCRIPTION_GAZEBO_POSE_PLUGIN_H
#define BAXTER_DESCRIPTION_GAZEBO_POSE_PLUGIN_H

#include <random>
#include <chrono>
#include <cmath>
#include <iostream>

#include <ros/ros.h>

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

namespace baxter_description
{

  class PosePlugin : public gazebo::ModelPlugin {
   public:
    PosePlugin() : gazebo::ModelPlugin() { }
    ~PosePlugin();

   protected:
    void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf);
    void OnUpdate(const gazebo::common::UpdateInfo&);

   private:
    // ROS Stuff
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Publisher transform_NWU_pub_;
    ros::Publisher transform_NED_pub_;
    ros::Publisher pose_NWU_pub_;
    ros::Publisher pose_NED_pub_;
    ros::Publisher euler_pub_;

    std::string namespace_;
    std::string transform_pub_topic_;
    std::string pose_pub_topic_;
    std::string parent_frame_id_;
    std::string link_name_;

    int gazebo_sequence_;
    int pose_sequence_;

    // Gazebo Information
    gazebo::physics::WorldPtr world_;
    gazebo::physics::ModelPtr model_;
    gazebo::physics::LinkPtr link_;
    gazebo::physics::EntityPtr parent_link_;
    gazebo::event::ConnectionPtr updateConnection_;
  };
}

#endif // ROSFLIGHT_PLUGINS_GAZEBO_POSE_PLUGIN_H
