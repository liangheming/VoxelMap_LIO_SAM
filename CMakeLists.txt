cmake_minimum_required(VERSION 3.0.2)
project(voxel_lio_sam)

add_compile_options(-std=c++17)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  pcl_ros
  livox_ros_driver2
)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)



## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES voxel_lio_sam
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

set(SRC_LIST src/utils.cpp
             src/map_builder/commons.cpp
             src/map_builder/ieskf.cpp)

add_executable(mapping_node src/mapping_node.cpp ${SRC_LIST})

target_link_libraries(mapping_node
  ${catkin_LIBRARIES}
)
