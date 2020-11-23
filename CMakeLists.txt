cmake_minimum_required(VERSION 2.8.3)
project(stefmap_ros)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  message_generation
  std_msgs
  dynamic_reconfigure
  nav_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   STeFMapMsg.msg
   STeFMapCellMsg.msg
 )
 

## Generate services in the 'srv' folder
add_service_files(
   FILES
   GetSTeFMap.srv
   UpdateSTeFMap.srv
   GetVisibilityMap.srv
 )


## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs nav_msgs)

 generate_dynamic_reconfigure_options(
  cfg/stefmapclient.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES stefmap_ros
  CATKIN_DEPENDS message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(stefmap_ros src/stefmap.cpp)
add_dependencies(stefmap_ros stefmap_ros_generate_messages_cpp)
target_link_libraries(stefmap_ros ${catkin_LIBRARIES})

install(TARGETS stefmap_ros
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)


