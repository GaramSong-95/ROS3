cmake_minimum_required(VERSION 3.0.2)
project(cmd_vel_topic)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  geometry_msgs
)

add_message_files(FILES msgCmdVel.msg)
generate_messages(
	DEPENDENCIES 
	std_msgs
	geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES cmd_vel_topic
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(cmd_vel_pub src/cmd_vel_pub.cpp)
target_link_libraries(cmd_vel_pub ${catkin_LIBRARIES})
add_dependencies(cmd_vel_pub cmd_vel_topic_generate_messages_cpp)

add_executable(cmd_vel_sub src/cmd_vel_sub.cpp)
target_link_libraries(cmd_vel_sub ${catkin_LIBRARIES})
add_dependencies(cmd_vel_sub cmd_vel_topic_generate_messages_cpp)
