cmake_minimum_required(VERSION 2.8.3)
project(fsm_bump_go)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  roslint
)

catkin_package(
CATKIN_DEPENDS roscpp std_msgs kobuki_msgs sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_library(${PROJECT_NAME}
  src/fsm_bump_go/BasicBump.cpp
)

add_executable(basic_node src/basic_node.cpp)
target_link_libraries(basic_node ${catkin_LIBRARIES} ${PROJECT_NAME})

roslint_cpp(
 src/basic_node.cpp
 src/fsm_bump_go/BasicBump.cpp
 include/fsm_bump_go/BasicBump.h
)
