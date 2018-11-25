#!/usr/bin/env python

import rospy
import roslib
import tf

from geometry_msgs.msg import PoseArray
from aruco_msgs.msg import MarkerArray


#Defining a class
class Marker_detect():

	def __init__(self):
		#Initializes the node and does the first step succesfully as listener
		rospy.init_node('marker_detection',anonymous=False) # initializing a ros node with name marker_detection
		#Declares empty dicts to save data and print data from successfully
		self.whycon_marker = {}	# Declaring dictionaries
		self.aruco_marker = {}
		#Initializes subscribers to poses and markers successfully with datatype PoseArray and MarkerArray and 
		#calling of functions whycon_data and aruco_data which are callback functions which must 
		#store and thereby print data as output
		rospy.Subscriber('/whycon/poses',PoseArray, self.whycon_data)	# Subscribing to topic
		rospy.Subscriber('/aruco_marker_publisher/markers', MarkerArray , self.aruco_data)	# Subscribing to topic
		


	# Callback for /whycon/poses
	def whycon_data(self,msg):
		print "\n"
		length=len(msg.poses)
		for i in range(length):	
			self.whycon_marker[i]=[msg.poses[i].position.x,msg.poses[i].position.y,msg.poses[i].position.z]
		

	# Callback for /aruco_marker_publisher/markers
	def aruco_data(self,msg):
		# Printing the detected markers on terminal
		print "\n"
		length=len(msg.markers)
		for i in range(length):	
			self.aruco_marker[i]=[msg.markers[i].pose.pose.orientation.x, msg.markers[i].pose.pose.orientation.y, msg.markers[i].pose.pose.orientation.z]
		
		print "WhyCon_marker",self.whycon_marker
		print "ArUco_marker",self.aruco_marker


if __name__=="__main__":

	marker = Marker_detect()

	
	while not rospy.is_shutdown():
		rospy.spin()