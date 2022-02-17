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

#include "fsm_bump_go/MegaPro.h"

#include "kobuki_msgs/BumperEvent.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

#include "ros/ros.h"

namespace fsm_bump_go
{

MegaPro::MegaPro()
: n_("~")
{
  linspeed_ = n_.param("linspeed", 1.0);
  angspeed_ = n_.param("angspeed", 0.5);
  state_ = GOING_FORWARD;
  detected_ = false;
  sub_ = n_.subscribe("/scan_filtered", 1, &MegaPro::detectionCallBack, this);
  pub_vel_ = n_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
}

void
MegaPro::detectionCallBack(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  // Returns array index of an inrange detection
  if (state_ == GOING_FORWARD)
  {
    indexlim_ = anglelim_ / msg->angle_increment;
    indexnear_ = detectInRange(msg);

    if (indexnear_ != -1)
      {
        detected_ = true;

      }
  } else if (state_ == READ)
  {
    indexfar_=detectBetterOption(msg);
    TURNING_TIME = (indexfar_ * msg->angle_increment) / 0.5;
  }  
   else
  {
    detected_ = false;
  }
}

void
MegaPro::step()
{
  geometry_msgs::Twist cmd;

  cmd.linear.y = 0.0;
  cmd.linear.z = 0.0;
  cmd.angular.x = 0.0;
  cmd.angular.y = 0.0;

  switch (state_)
  {
    case GOING_FORWARD:
      cmd.linear.x = linspeed_;
      cmd.angular.z = 0.0;

      if (detected_)
      {
        state_ = READ;
        ROS_INFO("GOING_FORWARD -> READ");
      }

      break;
    case READ:
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      
      detected_ts_ = ros::Time::now();
      state_=TURNING;
      ROS_INFO("READ -> TURNING");

      break;
    case TURNING:
      cmd.linear.x = 0.0;
      cmd.angular.z = angspeed_;

      if ((ros::Time::now()-turn_ts_).toSec() > TURNING_TIME )
      {
        state_ = GOING_FORWARD;
        ROS_INFO("TURNING -> GOING_FORWARD");
      }
      break;
    }

}

int
MegaPro::detectInRange(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  int pos = 0;
  while (pos < msg->ranges.size())
  {
    if (msg->ranges[pos] <= dist_) return pos;
    if (pos == indexlim_) pos = int(msg->ranges.size() - indexlim_);
    pos = pos + 1;
  }
  return -1;
}

int
MegaPro::detectBetterOption(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  int farther=0;
  for (int pos=0; pos < msg->ranges.size(); pos++)
  {
    if (msg->ranges[pos] > msg->ranges[farther]) farther=pos;
  }
  return farther;
}

}  // namespace fsm_bump_go
