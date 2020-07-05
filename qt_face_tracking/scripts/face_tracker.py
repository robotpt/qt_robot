import math
import rospy

from qt_nuitrack_app.msg import Skeletons
from std_msgs.msg import Float64MultiArray


SKELETON_TOPIC = '/qt_nuitrack_app/skeletons'
HEAD_COMMAND_TOPIC = '/qt_robot/head_position/command'
HEAD_JOINT_INDEX = 1


class HeadMover:

    def __init__(self, head_command_topic):
        self._head_pose_publisher = rospy.Publisher(head_command_topic, Float64MultiArray, queue_size=1)

    def __call__(self, yaw, pitch):
        head_command = Float64MultiArray()
        head_command.data = [yaw, pitch]
        self._head_pose_publisher.publish(head_command)


class QtFaceTracker:

    class Joints:
        HEAD_YAW = 'HeadYaw'
        HEAD_PITCH = 'HeadPitch'

    def __init__(self, skeleton_topic, head_command_topic, degree_epsilon=10):
        self._skeleton_subscriber = rospy.Subscriber(
            skeleton_topic, Skeletons, self._skeleton_callback, queue_size=1)
        self._head_mover = HeadMover(head_command_topic)
        self._degree_epsilon = degree_epsilon

    def _skeleton_callback(self, data):
        head = data.skeletons[0].joints[HEAD_JOINT_INDEX]
        head_x, head_y, head_z = head.real
        confidence = head.confidence

        yaw_angle = math.degrees(math.atan2(head_x, head_z))
        pitch_angle = math.degrees(math.atan2(head_y, head_z))

        if math.sqrt(yaw_angle**2 + pitch_angle**2) > self._degree_epsilon\
                and confidence >= 0.75:
            self._head_mover(-yaw_angle, -pitch_angle)


if __name__ == '__main__':
    rospy.init_node('face_tracker', anonymous=False)

    if False:  # head mover
        head_mover = HeadMover(HEAD_COMMAND_TOPIC)
        rospy.sleep(1)
        head_mover(10, 30)
        rospy.sleep(5)
        head_mover(0, 0)
        rospy.sleep(1)

    if True:
        face_tracker = QtFaceTracker(SKELETON_TOPIC, HEAD_COMMAND_TOPIC)
        rospy.spin()
