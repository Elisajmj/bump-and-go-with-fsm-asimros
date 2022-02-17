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

#ifndef FSM_BUMP_GO_MegaPro_H
#define FSM_BUMP_GO_MegaPro_H

#include "ros/ros.h"

#include "geometry_msgs/Twist.h"
#include "fsm_bump_go/Base.h"
#include "sensor_msgs/LaserScan.h"

namespace fsm_bump_go
{

class MegaPro
{
public:
  MegaPro();
  void detectionCallBack(const sensor_msgs::LaserScan::ConstPtr& msg);
  int detectInRange(const sensor_msgs::LaserScan::ConstPtr& msg);
  int detectBetterOption(const sensor_msgs::LaserScan::ConstPtr& msg);
  void step();

private:
  ros::NodeHandle n_;

  static const int GOING_FORWARD   = 0;
  static const int READ = 1;
  static const int TURNING = 2;

  double TURNING_TIME;

  int state_;
  float linspeed_;
  float angspeed_;

  bool detected_;


  int dist_ = 1;
  int indexnear_;
  int indexfar_;
  int indexlim_;
  float turnangle_;
  float anglelim_ = 0.16;

  ros::Time detected_ts_;
  ros::Time turn_ts_;

  ros::Subscriber sub_;
  ros::Publisher pub_vel_;

};

}  // namespace fsm_bump_go

#endif  // FSM_BUMP_GO_MegaPro_H
