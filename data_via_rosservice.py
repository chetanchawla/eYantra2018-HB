#!/usr/bin/env python2
from plutodrone.srv import *
from std_msgs.msg import Float64
import rospy

class request_data():
	"""docstring for request_data"""
	def __init__(self):
		rospy.init_node('drone_board_data')
		data = rospy.Service('PlutoService', PlutoPilot, self.access_data)
		
		#Adding a new publisher to publish the yaw values and tune Yaw PID
		#Published on topic yawyaw
		self.yawval=rospy.Publisher('/yawyaw',Float64,queue_size=1)
		rospy.spin()

	def access_data(self, req):
		 print "accx = " + str(req.accX), "accy = " + str(req.accY), "accz = " + str(req.accZ)
		 print "gyrox = " + str(req.gyroX), "gyroy = " + str(req.gyroY), "gyroz = " + str(req.gyroZ)
		 print "magx = " + str(req.magX), "magy = " + str(req.magY), "magz = " + str(req.magZ)
		 print "roll = " + str(req.roll), "pitch = " + str(req.pitch), "yaw = " + str(req.yaw)
		 print "altitude = " +str(req.alt)
		 #Publishes the yaw value
		 self.yawval.publish(req.yaw)
		 rospy.sleep(.1)
		 return PlutoPilotResponse(rcAUX2 =1500)

test = request_data()
		
