#!/usr/bin/env python
from __future__ import print_function
import os
import rospy
from sensor_msgs.msg import JointState
import geometry_msgs.msg
import tf
import math
from std_msgs.msg import Header
from src.qt_simulator.srv import gestures_play
from src.qt_simulator.scripts import parse_xml
dir_path = os.path.dirname(os.path.realpath(__file__))


DEFAULT_POS = {
    'HeadYaw': 0,
    'HeadPitch': 0,
    'RightElbowRoll': 0,
    'RightShoulderPitch': -1.42,
    'RightShoulderRoll': -0.77,
    'LeftElbowRoll': -0.87,
    'LeftShoulderPitch': 1.14,
    'LeftShoulderRoll': -0.95
}


def get_joint_position(data, state, joint_name):
    if len(data[joint_name]) is not 0:
        return math.radians(data[joint_name][state])
    else:
        return DEFAULT_POS[joint_name]


def get_current_state(data, state):
    head_yaw = get_joint_position(data, state, 'HeadYaw')
    head_pitch = get_joint_position(data, state, 'HeadPitch')
    right_shoulder_pitch = get_joint_position(data, state, 'RightShoulderPitch')
    right_shoulder_roll = get_joint_position(data, state, 'RightShoulderRoll')
    right_elbow_roll = get_joint_position(data, state, 'RightElbowRoll')
    left_shoulder_pitch = get_joint_position(data, state, 'LeftShoulderPitch')
    left_shoulder_roll = get_joint_position(data, state, 'LeftShoulderRoll')
    left_elbow_roll = get_joint_position(data, state, 'LeftElbowRoll')

    return [head_yaw, head_pitch, right_shoulder_pitch, right_shoulder_roll, right_elbow_roll,
                            left_shoulder_pitch, left_shoulder_roll, left_elbow_roll]


def publish_joint_position(joint_state, odom_trans, joint_pub, data, broadcaster, state):
        joint_state.header = Header()
        joint_state.header.stamp = rospy.Time.now()

        joint_state.name = ['HeadYaw', 'HeadPitch', 'RightShoulderPitch', 'RightShoulderRoll', 'RightElbowRoll',
                            'LeftShoulderPitch', 'LeftShoulderRoll', 'LeftElbowRoll']

        joint_state.position = get_current_state(data, state)

        odom_trans.header.stamp = rospy.Time.now()
        odom_trans.transform.translation.x = 0
        odom_trans.transform.translation.y = 0
        odom_trans.transform.translation.z = 0

        odom_trans.transform.rotation.x = 0
        odom_trans.transform.rotation.y = 0
        odom_trans.transform.rotation.z = 0
        odom_trans.transform.rotation.w = 0

        joint_pub.publish(joint_state)
        broadcaster.sendTransformMessage(odom_trans)


def gesture_play(req):
    relative_file_path = '../../../resources/gestures/QT/' + req.a
    FILE_PATH = os.path.join(dir_path, relative_file_path)
    broadcaster = tf.TransformBroadcaster()
    joint_pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.init_node('state_publisher', anonymous=True)

    odom_trans = geometry_msgs.msg.TransformStamped()
    odom_trans.header.frame_id = 'odom'
    odom_trans.child_frame_id = 'base_link'

    joint_state = JointState()
    state = 0
    rospy.sleep(2)
    time_difference, data = parse_xml.parse_xml_file(FILE_PATH)

    while not rospy.is_shutdown() and state < len(time_difference):
        publish_joint_position(joint_state, odom_trans, joint_pub, data, broadcaster, state)
        rospy.sleep(time_difference[state] * 10 ** (-9))
        state = state + 1
        # state = state % len(time_difference)

    return True


def gestures_play_server():
    rospy.init_node('gestures_play_server')
    s = rospy.Service('gestures_play', gestures_play, gesture_play)
    print("Ready to play gestures")
    rospy.spin()


if __name__ == '__main__':
    gestures_play_server()