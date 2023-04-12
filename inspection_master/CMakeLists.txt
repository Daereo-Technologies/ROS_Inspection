cmake_minimum_required(VERSION 3.0.2)
project(inspection_master)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  tf
  tf2
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy geometry_msgs 
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(ref_offb_node src/ref_offboard_control.cpp)
add_executable(transforms src/transforms.cpp)

add_dependencies(ref_offb_node  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(transforms ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(ref_offb_node  ${catkin_LIBRARIES} )
target_link_libraries(transforms ${catkin_LIBRARIES} )
