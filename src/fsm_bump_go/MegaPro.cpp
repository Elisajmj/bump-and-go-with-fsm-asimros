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
{
  state_ = GOING_FORWARD;
  detected_ = false;
  side_ = LEFT;
  sub_ = n_.subscribe("/scan_filtered", 1, &MegaPro::detectionCallBack, this);
  pub_vel_ = n_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
}

void
MegaPro::detectionCallBack(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  // Returns array index of an inrange detection
  indexlim_ = anglelim_ / msg->angle_increment;
  indexnear_ = detectInRange(msg);
  indexfar_ = detectBetterOption(msg);

  if (indexnear_ != -1)
  {
    detected_ = true;
    side_ = LEFT;
    TURNING_TIME = (indexfar_ * msg->angle_increment) / 0.5;

  } else
  {
    detected_ = false;
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
detectBetterOption(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  int farther=0;
  for (int pos=0; pos < msg->ranges.size(); pos++)
  {
    if (msg->ranges[pos] > msg->ranges[farther]) farther=pos;
  }
  return farther;
}

}  // namespace fsm_bump_go
