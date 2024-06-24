#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from src.qt_simulator.srv import gestures_play


def gestures_play_client(file):
    rospy.wait_for_service('gestures_play')
    try:
        play_gesture = rospy.ServiceProxy('gestures_play', gestures_play)
        resp1 = play_gesture(file)
        return resp1.status
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
    if len(sys.argv) >= 2:
        for file in sys.argv[1]:
            gestures_play_client(file)
    else:
        sys.exit(1)

