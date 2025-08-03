cmake_minimum_required(VERSION 3.1.3)
project(pick)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  tf2_ros
  tf2_geometry_msgs
  message_generation
  project_msgs
  std_srvs
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
    roscpp 
    std_msgs
    moveit_core
    moveit_ros_planning_interface
    tf2_geometry_msgs
    message_runtime
    geometry_msgs
  DEPENDS
    EIGEN3
    std_srvs
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR} 
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(pick_node src/applications/pick_node.cpp)


add_library(pick src/pick.cpp)



target_link_libraries(pick_node pick)


target_link_libraries(pick ${catkin_LIBRARIES} ${Boost_LIBRARIES})
