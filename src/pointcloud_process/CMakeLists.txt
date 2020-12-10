cmake_minimum_required(VERSION 3.0.2)
project(pointcloud_process)
set( CMAKE_BUILD_TYPE "release" )  #release debug 模式
find_package(PCL 1.8 REQUIRED )
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )
list(APPEND ALL_LIBS ${PCL_LIBRARIES})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  pcl_ros
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pointcloud_process
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
list(APPEND ALL_LIBS ${catkin_LIBRARIES})
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/pointcloud_process.cpp
# )

# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_executable(PointCloudProcess src/pointcloudprocess.cpp src/ProcessNode.cpp)
target_link_libraries(PointCloudProcess ${ALL_LIBS})


# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
