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

#include "fsm_bump_go/ProBump.h"

#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

#include "ros/ros.h"

namespace fsm_bump_go
{

ProBump::ProBump()
: Base()
{
  state_ = GOING_FORWARD;
  detected_ = false;
  side_ = LEFT;
  dist_ = n_.param("dist", 1.0);
  anglelim_ = static_cast<float>(atan2(0.17, dist_));
  sub_ = n_.subscribe("/scan_filtered", 1, &ProBump::detectionCallBack, this);
  pub_vel_ = n_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
}

void
ProBump::detectionCallBack(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  // Returns array index of an inrange detection
  if (state_ == GOING_FORWARD)
  {
    index_ = detectInRange(msg);
    indexlim_ = anglelim_ / msg->angle_increment;

    if (index_ < indexlim_ && index_ >= 0)
    {
      detected_ = true;
      side_ = RIGHT;
    }
    else if (index_ > static_cast<int>(msg->ranges.size() - indexlim_))
    {
      detected_ = true;
      side_ = LEFT;
    }
    else
    {
      detected_ = false;
    }
  }

  else
  {
    detected_ = false;
  }
}

int
ProBump::detectInRange(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  int pos = 0;
  while (pos < msg->ranges.size())
  {
    if (msg->ranges[pos] <= dist_) return pos;
    if (pos == indexlim_) pos = static_cast<int>(msg->ranges.size() - indexlim_);
    pos = pos + 1;
  }
  return -1;
}

}  // namespace fsm_bump_go
