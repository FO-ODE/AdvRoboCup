cmake_minimum_required(VERSION 3.0.2)
project(object_grasp)

find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning_interface
  roscpp
  geometry_msgs
  actionlib
  move_base_msgs
)

catkin_package(
)

include_directories(
  SYSTEM
  ${catkin_INCLUDE_DIRS}
)

add_executable(item_grasp src/item_grasp.cpp)
add_dependencies(item_grasp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(item_grasp
   ${catkin_LIBRARIES}
 )

 install(TARGETS item_grasp
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_executable(drop_item src/drop_item.cpp)
target_link_libraries(drop_item ${catkin_LIBRARIES})


add_executable(move_to_position src/move_to_position.cpp)
target_link_libraries(move_to_position ${catkin_LIBRARIES})
