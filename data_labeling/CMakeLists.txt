cmake_minimum_required(VERSION 2.8.3)
project(data_labeling)
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   tf
   message_generation
   message_filters
)
find_package(OpenCV REQUIRED)

add_message_files(
   FILES
   DataLabelingBW.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
	CATKIN_DEPENDS message_runtime message_filters
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
  include ${OpenCV_INCLUDE_DIRS}
)

add_executable(data_labeling_one_person src/data_labeling_one_person.cpp)
target_link_libraries(data_labeling_one_person ${catkin_LIBRARIES})
target_link_libraries(data_labeling_one_person ${OpenCV_LIBS})
add_dependencies(data_labeling_one_person data_labeling_generate_messages_cpp)

add_executable(data_labeling_with_petra src/data_labeling_with_petra.cpp)
target_link_libraries(data_labeling_with_petra ${catkin_LIBRARIES})
target_link_libraries(data_labeling_with_petra ${OpenCV_LIBS})
add_dependencies(data_labeling_with_petra data_labeling_generate_messages_cpp)
