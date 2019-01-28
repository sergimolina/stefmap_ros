cmake_minimum_required(VERSION 2.8.3)
project(stefmap_ros)


find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
  std_msgs
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
 )


## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
  CATKIN_DEPENDS message_runtime
)