cmake_minimum_required(VERSION 3.0.2)
project(evaluation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   tf
   message_filters
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES petra
  CATKIN_DEPENDS roscpp rospy std_msgs message_filters message_runtime
)

include_directories(
  include
  include ${catkin_INCLUDE_DIRS}
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

add_executable(evaluation src/evaluation.cpp)
add_dependencies(evaluation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(evaluation ${catkin_LIBRARIES})
