cmake_minimum_required(VERSION 2.8.3)
project(petra)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  roslib
  tensorflow_ros_cpp
)
find_package(OpenCV REQUIRED)

add_message_files(
 FILES
 Leg.msg
 Person.msg
 People.msg
)

generate_messages(
 DEPENDENCIES
 std_msgs
 geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS message_runtime
)

include_directories(
  include
  ${tensorflow_ros_cpp_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
add_executable(petra src/petra.cpp
                     src/networkPrediction.cpp
                     src/processImage.cpp
                     src/correlation.cpp
                     src/kalmanFilterCorrelation.cpp)

add_dependencies(petra ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(petra ${tensorflow_ros_cpp_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBS})
