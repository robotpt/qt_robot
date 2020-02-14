# !/bin/bash

source /home/qtrobot/robot/autostart/qt_robot.inc

SCRIPT_NAME="start_cordial_face.sh"
LOG_FILE=$(prepare_logfile "$SCRIPT_NAME")

{
prepare_ros_environment
wait_for_tcpip_port 1883 60

roslaunch qt_robot_pi qt_robot_pi.launch
roslaunch cordial_sound sound_listener.launch

} &>> ${LOG_FILE}

