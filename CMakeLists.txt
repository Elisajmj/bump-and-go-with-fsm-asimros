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
  src/fsm_bump_go/Base.cpp
  src/fsm_bump_go/BasicBump.cpp
  src/fsm_bump_go/AdvancedBump.cpp
  src/fsm_bump_go/ProBump.cpp
  src/fsm_bump_go/MegaPro.cpp
)

add_executable(basic_node src/basic_node.cpp)
add_executable(advanced_node src/advanced_node.cpp)
add_executable(pro_node src/pro_node.cpp)
add_executable(mega_node src/mega_node.cpp)

target_link_libraries(basic_node ${catkin_LIBRARIES} ${PROJECT_NAME})
target_link_libraries(advanced_node ${catkin_LIBRARIES} ${PROJECT_NAME})
target_link_libraries(pro_node ${catkin_LIBRARIES} ${PROJECT_NAME})
target_link_libraries(mega_node ${catkin_LIBRARIES} ${PROJECT_NAME})

roslint_cpp(
 src/basic_node.cpp
 include/fsm_bump_go/BasicBump.h
 src/fsm_bump_go/BasicBump.cpp
 include/fsm_bump_go/Base.h
 src/fsm_bump_go/Base.cpp
 src/advanced_node.cpp
 include/fsm_bump_go/AdvancedBump.h
 src/fsm_bump_go/AdvancedBump.cpp
 src/pro_node.cpp
 include/fsm_bump_go/ProBump.h
 src/fsm_bump_go/ProBump.cpp
 include/fsm_bump_go/MegaPro.h
 src/fsm_bump_go/MegaPro.cpp
 src/mega_node.cpp
)