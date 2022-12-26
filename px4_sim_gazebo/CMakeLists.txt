cmake_minimum_required(VERSION 3.0.2)
project(px4_sim_gazebo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)
catkin_package(
  CATKIN_DEPENDS roscpp rospy
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_ref_offb_node src/ref_offboard_control.cpp)

add_dependencies(${PROJECT_NAME}_ref_offb_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_ref_offb_node
   ${catkin_LIBRARIES}
 )
