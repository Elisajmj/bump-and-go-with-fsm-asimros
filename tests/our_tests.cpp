// Copyright 2022 AsimRos team
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

#include <gtest/gtest.h>
#include "ros/ros.h"
#include "fsm_bump_go/BasicBump.h"
#include "fsm_bump_go/AdvancedBump.h"
#include "fsm_bump_go/ProBump.h"

class TestBasicBump: public fsm_bump_go::BasicBump
{
public:
  int get_state() const { return state_; }
  bool get_pressed() const { return pressed_; }
  void set_pressed(const bool pressed) { pressed_ = pressed; }
  double get_backing_time() const { return BACKING_TIME; }
  double get_turning_time() const { return TURNING_TIME; }
};

class TestAdvancedBump: public fsm_bump_go::AdvancedBump
{
public:
  int get_state() { return state_; }
  int get_side() { return side_; }
  bool get_detected() {return detected_; }
};

class TestProBump: public fsm_bump_go::ProBump
{
public:
  int get_state() { return state_; }
  int get_side() { return side_; }
  bool get_detected() {return detected_; }
};

TEST(BasicBump, test_set_reading)
{
  TestBasicBump basic_bump_test;
  ASSERT_EQ(basic_bump_test.get_state(), 0);  // GOING_FORWARD
  ASSERT_EQ(basic_bump_test.get_pressed(), false);

  basic_bump_test.set_pressed(true);
  basic_bump_test.step();
  ASSERT_EQ(basic_bump_test.get_state(), 1);  // GOING_BACK

  usleep(basic_bump_test.get_backing_time() * 1000000);
  basic_bump_test.step();
  ASSERT_EQ(basic_bump_test.get_state(), 2);  // TURNING

  usleep(basic_bump_test.get_turning_time() * 1000000);
  basic_bump_test.step();
  ASSERT_EQ(basic_bump_test.get_state(), 0);  // GOING_FORWARD
}

TEST(AdvancedBump, test_set_reading)
{
  TestAdvancedBump advanced_bump_test;

  ASSERT_EQ(advanced_bump_test.get_state(), 0);  // GOING_FORWARD
  ASSERT_EQ(advanced_bump_test.get_detected(), false);
  ASSERT_EQ(advanced_bump_test.get_side(), 0);  // LEFT
}

TEST(ProBump, test_set_reading)
{
  TestProBump pro_bump_test;

  ASSERT_EQ(pro_bump_test.get_state(), 0);  // GOING_FORWARD
  ASSERT_EQ(pro_bump_test.get_detected(), false);
  ASSERT_EQ(pro_bump_test.get_side(), 0);  // LEFT
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "basic_node");

  return RUN_ALL_TESTS();
}
