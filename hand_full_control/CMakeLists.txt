cmake_minimum_required(VERSION 2.8.3)
project(hand_full_control)

find_package(catkin REQUIRED
  message_generation
  rospy
  std_msgs
)

add_service_files(
  FILES
  TargetAngles.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()
