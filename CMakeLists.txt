cmake_minimum_required(VERSION 2.8.3)
project(fsm_bump_go)

set(CMAKE_CXX_STANDARD 17)

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
  
)

add_executable(basic_node src/basic_node.cpp)
add_executable(advanced_node src/advanced_node.cpp)
add_executable(pro_node src/pro_node.cpp)


target_link_libraries(basic_node ${catkin_LIBRARIES} ${PROJECT_NAME})
target_link_libraries(advanced_node ${catkin_LIBRARIES} ${PROJECT_NAME})
target_link_libraries(pro_node ${catkin_LIBRARIES} ${PROJECT_NAME})


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
 tests/our_tests.cpp
)

catkin_add_gtest(our_tests tests/our_tests.cpp)
add_dependencies(our_tests ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
if(TARGET our_tests)
  target_link_libraries(our_tests ${catkin_LIBRARIES} ${PROJECT_NAME})
endif()

