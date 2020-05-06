cmake_minimum_required(VERSION 2.8.3)
project(qt_nuitrack_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  FaceInfo.msg
  Faces.msg
  GestureInfo.msg
  Gestures.msg
  HandInfo.msg
  Hands.msg
  JointInfo.msg
  SkeletonInfo.msg
  Skeletons.msg
)

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   suspend.srv
 )

## Generate added messages and services with any dependencies listed here
generate_messages()
catkin_package()

