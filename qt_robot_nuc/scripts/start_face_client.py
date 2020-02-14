#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger

if __name__ == "__main__":
	rospy.sleep(3)
	service_name = rospy.get_param('qt_robot/display/start_browser_on_pi_service')
	rospy.wait_for_service(service_name)
	service_call = rospy.ServiceProxy(service_name, Trigger)
	try:
		response = service_call()
		print(response)
	except rospy.ServiceException, e:
		print("Service call failed: %s" % e)
