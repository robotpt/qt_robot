#!/usr/bin/env python
import os
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import geometry_msgs.msg
import roslib
import tf
from std_msgs.msg import Header
# from src.qt_simulator.scripts.parse_xml import parse_xml_file
import xml.etree.ElementTree as ET
import sys

dir_path = os.path.dirname(os.path.realpath(__file__))
# sys.path.append(os.path.join(dir_path, 'parse_xml.py'))
FILE_PATH = os.path.join(dir_path, '../../../resources/gestures/QT/breathing_exercise.xml')


def parse_xml_file(file_path):
    tree = ET.parse(file_path)
    root = tree.getroot()

    time = []
    pos_data = {
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

    return time, pos_data


if __name__ == '__main__':
    broadcaster = tf.TransformBroadcaster()
    joint_pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.init_node('state_publisher', anonymous=True)
    rate = rospy.Rate(30)  # 10hz

    odom_trans = geometry_msgs.msg.TransformStamped()
    odom_trans.header.frame_id = 'odom'
    odom_trans.child_frame_id = 'base_link'

    joint_state = JointState()
    state = 0
    rospy.sleep(3)
    num_state, data = parse_xml_file(FILE_PATH)
    time_numeric = [int(numeric_string) for numeric_string in num_state]
    time_difference = [time_numeric[i + 1] - time_numeric[i] for i in range(len(time_numeric) - 1)]

    while not rospy.is_shutdown():
        joint_state.header = Header()
        joint_state.header.stamp = rospy.Time.now()

        joint_state.name = ['HeadYaw', 'HeadPitch', 'RightShoulderPitch', 'RightShoulderRoll', 'RightElbowRoll',
                            'LeftShoulderPitch', 'LeftShoulderRoll', 'LeftElbowRoll']

        if len(data['RightShoulderPitch']) is not 0:
            right_shoulder_pitch = data['RightShoulderPitch'][state] * 0.01
        else:
            right_shoulder_pitch = -1.42

        if len(data['RightShoulderRoll']) is not 0:
            right_shoulder_roll = data['RightShoulderRoll'][state] * 0.01
        else:
            right_shoulder_roll = -0.77

        if len(data['RightElbowRoll']) is not 0:
            right_elbow_roll = data['RightElbowRoll'][state] * 0.01
        else:
            right_elbow_roll = 0

        if len(data['LeftShoulderPitch']) is not 0:
            left_shoulder_pitch = data['LeftShoulderPitch'][state] * 0.01
        else:
            left_shoulder_pitch = 1.14

        if len(data['LeftShoulderRoll']) is not 0:
            left_shoulder_roll = data['LeftShoulderRoll'][state] * 0.01
        else:
            left_shoulder_roll = -0.95

        if len(data['LeftElbowRoll']) is not 0:
            left_elbow_roll = data['LeftElbowRoll'][state] * 0.01
        else:
            left_elbow_roll = -0.87

        joint_state.position = [0, 0, right_shoulder_pitch, right_shoulder_roll, right_elbow_roll,
                                left_shoulder_pitch, left_shoulder_roll, left_elbow_roll]

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

        state = state + 1
        state = state % len(num_state)
        rate.sleep()