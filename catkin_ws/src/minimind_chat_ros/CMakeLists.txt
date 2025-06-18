cmake_minimum_required(VERSION 3.0.2)
project(minimind_chat_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

## Declare message files
add_message_files(
  FILES
  ChatIntent.msg
)

## Generate message and service code
generate_messages(
  DEPENDENCIES
  std_msgs
)

## Declare catkin package
catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime
)

## Install Python scripts
catkin_install_python(
  PROGRAMS
    scripts/eval_model.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install model and config folders if needed
install(DIRECTORY
  src/minimind_chat_ros/model
  src/minimind_chat_ros/out
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
