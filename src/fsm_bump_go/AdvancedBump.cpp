// Copyright 2022 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "fsm_bump_go/AdvancedBump.h"

#include "kobuki_msgs/BumperEvent.h"
#include "geometry_msgs/Twist.h"

#include "ros/ros.h"

namespace fsm_bump_go
{

AdvancedBump::AdvancedBump()
{
  state_ = GOING_FORWARD;
  detected_ = false;
  side_ = 0;
  sub_ = n_.subscribe("/mobile_base/events/bumper", 1, &AdvancedBump::detectionCallBack, this);
  pub_vel_ = n_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
}

void
AdvancedBump::detectionCallBack(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
  detected_ = msg->state == kobuki_msgs::BumperEvent::PRESSED;
  side_ = msg->bumper;
  ROS_INFO("Data: [%d]", msg->bumper);
}

}  // namespace fsm_bump_go
