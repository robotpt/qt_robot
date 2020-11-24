#!/usr/bin/env python
from __future__ import print_function
import os
import rospy
from sensor_msgs.msg import JointState
import geometry_msgs.msg
import tf
import math
from std_msgs.msg import Header
import xml.etree.ElementTree as ET
dir_path = os.path.dirname(os.path.realpath(__file__))
from qt_simulator.srv import gestures_play


def parse_xml_file(file_path):
    tree = ET.parse(file_path)
    root = tree.getroot()

    time = []
    pos_data = {
        'HeadYaw': [],
        'HeadPitch': [],
        'RightElbowRoll': [],
        'RightShoulderPitch': [],
        'RightShoulderRoll': [],
        'LeftElbowRoll': [],
        'LeftShoulderPitch': [],
        'LeftShoulderRoll': []
    }

    for point in root.iter('point'):
        for key, val in pos_data.items():
            if point.find(key) is not None:
                val.append(float(point.find(key).text))
        time.append(point.get('time'))

    time_numeric = [int(numeric_string) for numeric_string in time]
    time_difference = [time_numeric[i + 1] - time_numeric[i] for i in range(len(time_numeric) - 1)]
    time_difference.append(0)

    return time_difference, pos_data


def get_current_state(data, state):
    if len(data['HeadYaw']) is not 0:
        head_yaw = math.radians(data['HeadYaw'][state])
    else:
        head_yaw = 0

    if len(data['HeadPitch']) is not 0:
        head_pitch = math.radians(data['HeadPitch'][state])
    else:
        head_pitch = 0

    if len(data['RightShoulderPitch']) is not 0:
        right_shoulder_pitch = math.radians(data['RightShoulderPitch'][state])
    else:
        right_shoulder_pitch = -1.42

    if len(data['RightShoulderRoll']) is not 0:
        right_shoulder_roll = math.radians(data['RightShoulderRoll'][state])
    else:
        right_shoulder_roll = -0.77

    if len(data['RightElbowRoll']) is not 0:
        right_elbow_roll = math.radians(data['RightElbowRoll'][state])
    else:
        right_elbow_roll = 0

    if len(data['LeftShoulderPitch']) is not 0:
        left_shoulder_pitch = math.radians(data['LeftShoulderPitch'][state])
    else:
        left_shoulder_pitch = 1.14

    if len(data['LeftShoulderRoll']) is not 0:
        left_shoulder_roll = math.radians(data['LeftShoulderRoll'][state])
    else:
        left_shoulder_roll = -0.95

    if len(data['LeftElbowRoll']) is not 0:
        left_elbow_roll = math.radians(data['LeftElbowRoll'][state])
    else:
        left_elbow_roll = -0.87

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
    time_difference, data = parse_xml_file(FILE_PATH)

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