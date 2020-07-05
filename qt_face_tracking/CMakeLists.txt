cmake_minimum_required(VERSION 3.0.2)
project(qt_face_tracking)
find_package(catkin REQUIRED COMPONENTS
  qt_nuitrack_app
  rospy
  std_msgs
)
catkin_python_setup()
catkin_package()
