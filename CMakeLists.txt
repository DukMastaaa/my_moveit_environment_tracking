cmake_minimum_required(VERSION 3.0.2)
project(my_moveit_environment_tracking)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  moveit_msgs
  message_generation
  gazebo_ros
  # interactive_markers
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_perception
  rviz_visual_tools
  moveit_visual_tools
)

find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})

add_message_files(
  FILES
  VisibleModels.msg
)

add_service_files(
  FILES
  CollisionObjectFromModel.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  moveit_msgs  
)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs std_msgs moveit_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${GAZEBO_INCLUDE_DIRS}
)

add_executable(visibility_from_logical_camera src/visibility_from_logical_camera.cpp src/default_params.cpp)
target_link_libraries(visibility_from_logical_camera ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(visibility_from_whole_gazebo src/visibility_from_whole_gazebo.cpp src/default_params.cpp)
target_link_libraries(visibility_from_whole_gazebo ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(environment_tracker src/environment_tracker.cpp src/default_params.cpp)
target_link_libraries(environment_tracker ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
