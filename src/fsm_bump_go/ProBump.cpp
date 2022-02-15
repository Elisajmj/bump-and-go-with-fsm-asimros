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

#include "kobuki_msgs/BumperEvent.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

#include "ros/ros.h"

namespace fsm_bump_go
{

ProBump::ProBump()
{
  state_ = GOING_FORWARD;
  detected_ = false;
  side_ = LEFT;
  sub_ = n_.subscribe("/scan_filtered", 1, &ProBump::detectionCallBack, this);
  pub_vel_ = n_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
}

//pending stile review
int
ProBump::which_side(float sensor_msgs::LaserScan::ranges[])
{
  //en realidad -pi/2 y pi/2
  float derecha = ((sensor_msgs::LaserScan::range_min/2)-sensor_msgs::LaserScan::range_min)/sensor_msgs::LaserScan::angle_increment;
  //podría ser izquierda = -derecha;
  float izquierda = ((sensor_msgs::LaserScan::range_max/2)-sensor_msgs::LaserScan::range_min)/sensor_msgs::LaserScan::angle_increment;;

  float centro_drch = - 0,523599;
  //podría ser centro_izqd = -centro_drch;
  float centro_izqd = 0,523599;

  //si =< 0.5 entre sensor_msgs::LaserScan::ranges[izquierda] y sensor_msgs::LaserScan::ranges[centro_izqd]
  //girar dcha
  //si =< 0.5 en cualquiera de los otros
  //girar izquierda

}

void
ProBump::detectionCallBack(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  float posicion_0 =0-sensor_msgs::LaserScan::range_min / sensor_msgs::LaserScan::angle_increment;
  if(sensor_msgs::LaserScan::ranges[posicion_0] =< 0.5)
  {
    detected_ = true;
  }
  side_ = 1; // por ahora solo trabaja asumiendo que va de frente 0 1 2
  ROS_INFO("Data: [%d]", msg->bumper);
}


}  // namespace fsm_bump_go
