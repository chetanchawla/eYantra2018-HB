#!/usr/bin/env python

'''

This python file runs a ROS-node of name drone_control which holds the position of e-Drone on the given dummy.
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/alt_error				/pid_tuning_altitude
		/pitch_error			/pid_tuning_pitch
		/roll_error				/pid_tuning_roll
		/yaw_error				/pid_tuning_yaw
		/checking				/drone_yaw
								/vrep/waypoints
'''

# Importing the required libraries

from plutodrone.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time
#Time variables initialization for previous and current time stamps
lasttime=0.0
currenttime=0.0

class Edrone():
	"""docstring for Edrone"""
	def __init__(self):
		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z,yaw_value]
		self.drone_position = [0.0,0.0,0.0,0.0]	
		
		
		self.initial_waypoint = [-5.6,-2,24,0] # whycon marker at the position of the initial waypoint in the scene (blue sphere). These are whycon coordinates of the same
		#Here, the yaw desired is taken to be 0 while all the other point values are taken from the Target whycon marker (set renderable) as detected by the vision sensor.
		# [x_setpoint, y_setpoint, z_setpoint, yaw_value_setpoint]
		self.setpoint = self.initial_waypoint
		#Declaring a cmd of message type PlutoMsg and initializing values
		self.cmd = PlutoMsg()
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1500
		# self.cmd.plutoIndex = 0
		self.get=PoseArray()
		#The number of points that we want to get from the Lua code in a path. This is set to be 25.
		self.pointsInPath=20
		#Declaring and empty initialization of a list to store points given in a path by the OMPL library
		w, h = 4, 32
		self.path = [[0.0 for x in range(w)] for y in range(h)] 
		#Counter is used to count the number of points that are reached in the path within the maximum error threshold
		self.counter=0
		#Success has boolean values when the drone reaches the setpoints within maximum error thresholds
		self.success= 0
		#Variable to indicate if new path is being requested by drone from the OMPL library
		self.requested=0
		self.initial=0
		#initial setting of Kp, Kd and ki for [pitch, roll, throttle, yaw]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		
		self.iterations=1 #it is used to save the no. of iterations to know total time
		
		self.pathreceived=0
		#values of Kp, Kd and ki for [pitch, roll, throttle, yaw]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		
		self.Kp=[6,4.5,24,0.3]
		self.Ki=[0,0,0,0]
		self.Kd=[14.6,4.2,4,0.2]
		#As the drone is not rotating itself, we are using yaw parameters as 0.
		#self.Kp = [0,0,0,0]
		#self.Ki = [0,0,0,0]
		#self.Kd = [0,0,0,0]


		#-----------------------Add other required variables for pid here ----------------------------------------------
		self.prev_error=[0.0,0.0,0.0,0.0]#It saves previous error (roor in the last pid function call)
		self.dif_error=[0.0,0.0,0.0,0.0]#It stores the difference between current and the previous error(differential)
		self.sum_values=[0.0,0.0,0.0,0.0]#It stores the addition of error terms for iterms
		self.error=[0.0,0.0,0.0,0.0]#It stores the current computed error between desired point and self point
		self.lasttime=time.time()#Used for sample time calling of the pid function
		self.currenttime=0
		self.max_values = [1575,1575,1800,1800]#The max and min values of pitch, roll, throttle and yaw are specified 
		self.min_values = [1425,1425,1200,1200]
		

		# This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
		self.sample_time = 0.010 # in seconds (60 milliseconds)


		# Making Publishers for /drone_command, /alt_error, /pitch_error, /roll_error, /yaw_error
		self.command_pub = rospy.Publisher('/drone_command', PlutoMsg, queue_size=1)
		#------------------------Add other ROS Publishers here- Errors are piblished-----------------------------------------------------
		self.reqpath=rospy.Publisher('/checking',PoseArray,queue_size=1)
                #-----------------------------------------------------------------------------------------------------------
		# Subscribing to /whycon/poses, /drone_yaw, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('/whycon/poses', PoseArray, self.whycon_callback)
		#For changing the PID parameters using the pid_tuning package of ROS
		#rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		#-------------------------Add other ROS Subscribers here----------------------------------------------------
		#rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
		#rospy.Subscriber('/pid_tuning_roll',PidTune,self.roll_set_pid)
		#rospy.Subscriber('/pid_tuning_yaw',PidTune,self.yaw_set_pid)
		rospy.Subscriber('/yawyaw',Float64,self.setyaw)
		#Subscribes to the PoseArray published by the lua script to take in the points in the given path
		rospy.Subscriber('/vrep/waypoints', PoseArray,self.getnewpath)
		#------------------------------------------------------------------------------------------------------------

		self.arm() # ARMING THE DRONE
	#This function is called whenever the drone requires the next path points to be traversed
	def getnewpath(self,msg):
		print("Getting path")
		#Adds the points in the path list
		for i in range(1,self.pointsInPath+1):	
			self.path[i]=[msg.poses[i].position.x,msg.poses[i].position.y,msg.poses[i].position.z,0.0]
		self.pathreceived=1
		#Reinitializes the success variable and sets the setpoint as the path's first point
		# self.success=0
		# self.initial=1
		# self.requested=0
		# self.counter=0
		# self.setpoint=self.path[0]

	# Disarming condition of the drone
	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)


	# Arming condition of the drone : Best practise is to disarm and then arm the drone.
	def arm(self):

		self.disarm()

		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)	# Publishing /drone_command
		rospy.sleep(1)



	# Whycon callback function
	# The function gets executed each time when /whycon node publishes /whycon/poses 
	def whycon_callback(self,msg):
		self.drone_position[0] = msg.poses[0].position.x

		#--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------
		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z
		#---------------------------------------------------------------------------------------------------------------



	# Callback functions for /pid_tuning_altitude, /pid_tuning_pitch ...
	# This function gets executed each time when /tune_pid publishes /pid_tuning_altitude ...
	# These functions are used to tune the PID using the pid_tuning package using GUI sliders
	# def altitude_set_pid(self,alt):
	# 	self.Kp[2] = alt.Kp * 0.06 #Multiplying factors are used
	# 	self.Ki[2] = alt.Ki * 0.008
	# 	self.Kd[2] = alt.Kd * 0.3
	# def pitch_set_pid(self,pit):
	# 	self.Kp[0] = pit.Kp * 0.06 
	# 	self.Ki[0] = pit.Ki * 0.008
	# 	self.Kd[0] = pit.Kd * 0.3
	# def roll_set_pid(self,rol):
	# 	self.Kp[1] = rol.Kp * 0.06 
	# 	self.Ki[1] = rol.Ki * 0.008
	# 	self.Kd[1] = rol.Kd * 0.3
	# def yaw_set_pid(self,yaw):
	# 	self.Kp[3] = yaw.Kp * 0.06
	# 	self.Ki[3] = yaw.Ki * 0.008
	# 	self.Kd[3] = yaw.Kd * 0.3

	#Function to set yaw of the drone
	def setyaw(self,yaw2):
                self.drone_position[3]=yaw2.data
	#----------------------------------------------------------------------------------------------------------------------


	def pid(self):
		#Sets the PID
                self.currenttime=time.time()#Takes the current time stamp
                self.error=[a1-b1 for a1,b1 in zip(self.drone_position,self.setpoint)]#Calculates the error by subtracting drone position and setpoint positions
                thresh=1 # Threshold value or maximum error acceptable at a point in pitch and roll
                threshZ=2# Threshold value or maximum error acceptable at a point in altitude
                #if the drone is not on the final position (where it needs to just stay in the PID loop until new path points are received)
                if self.success == 0:
                	#Check for the error to be less than the maximum acceptable error in x, y, z from current position and setpoint
                	for i in range(0,4):
	                	if(i==0 or i==1):
	                		if(abs(self.error[i])>thresh):
	                			# If the error is not smaller than the maximum acceptable error, break the for loop and continue the pid to get closer
	                			break
	                	elif(i==2):
	                		if(abs(self.error[i])>threshZ):
	                			# If the error is not smaller than the maximum acceptable error, break the for loop and continue the pid to get closer
	                			break
	                	elif(i == 3):
	                		# If the error is within the maximum acceptable error range
	                		# if(self.initial==0):
	                		# 	#if the goal was the initial waypoint
	                		# 	self.success=1
	                		# 	print("Initial WayPoint Reached")
	                		# 	return
	                		# else:
	                			# If it is any other point in the path
	                		self.counter=self.counter+1# Increment the counter
	                		print("reached point ", self.counter)
	                		if(self.counter == 38):# If all the points in the path are covered
	                			self.success=1
	                			print("reached final goal")
	                			return
	                		# change the setpoint to the next point in the path
	                		self.setpoint= self.path[self.counter]
	                		return
             	# self.pitcherr.publish(self.error[0])#Publishes the respective errors for the plot juggler
            	# self.rollerr.publish(self.error[1])
            	# self.alterr.publish(self.error[2])
            	# self.yawerr.publish(self.error[3])
                self.sum_values=[a1+b1 for a1,b1 in zip(self.sum_values,self.error)]#sum of previous error values are added together for the given sample time
                if(self.currenttime-self.lasttime>=self.sample_time):
                	# If the sample time has elapsed, tune PID
                	self.dif_error= [a1-b1 for a1,b1 in zip(self.error,self.prev_error)]#derivative term- difference between current and previous error
		                #Pitch, Roll, Throttle a2nd Yaw are calculated with PID formulas
	                self.cmd.rcPitch=int(1500 + (self.error[1]*self.Kp[0]+self.dif_error[1]*self.Kd[0]/(self.sample_time)+self.sum_values[1]*self.Ki[0]*self.sample_time*self.iterations))
	                self.cmd.rcRoll=int(1500 - (self.error[0]*self.Kp[1]+self.dif_error[0]*self.Kd[1]/(self.sample_time)+self.sum_values[0]*self.Ki[1]*self.sample_time*self.iterations))
	                self.cmd.rcThrottle=int(1500 + self.error[2]*self.Kp[2]+self.dif_error[2]*self.Kd[2]/(self.sample_time)+self.sum_values[2]*self.Ki[2]*self.sample_time*self.iterations)
	                self.cmd.rcYaw=int(1500-(self.error[3]*self.Kp[3]+self.dif_error[3]*self.Kd[3]/(self.sample_time)+self.sum_values[3]*self.Ki[3]*self.sample_time*self.iterations))
	                self.lasttime=self.currenttime#The last time stamp is updated
	                self.prev_error=self.error#Previous error term is updated
	                #For checking that the pitch, roll, throttle and yaw values are within the max and minimum permissible values
	                if self.cmd.rcPitch > self.max_values[0]:
	                	self.cmd.rcPitch = self.max_values[0]
	                elif self.cmd.rcPitch < self.min_values[0]:
	                	self.cmd.rcPitch = self.min_values[0]
	                if self.cmd.rcRoll > self.max_values[1]:
	                	self.cmd.rcRoll = self.max_values[1]
	                elif self.cmd.rcRoll < self.min_values[1]:
	                	self.cmd.rcRoll = self.min_values[1]
	                if self.cmd.rcThrottle > self.max_values[2]:
	                	self.cmd.rcThrottle = self.max_values[2]
	                elif self.cmd.rcThrottle < self.min_values[2]:
	                	self.cmd.rcThrottle = self.min_values[2]
	                if self.cmd.rcYaw > self.max_values[3]:
	                	self.cmd.rcYaw = self.max_values[3]
	                elif self.cmd.rcYaw < self.min_values[3]:
	                	self.cmd.rcYaw = self.min_values[3]
	#------------------------------------------------------------------------------------------------------------------------
	                self.command_pub.publish(self.cmd)
	                self.iterations=self.iterations+1  #add karna hai
                        

if __name__=="__main__":
	e_drone=Edrone()
	e_drone.path[0]=e_drone.setpoint
	e_drone.reqpath.publish(e_drone.get)
	while(e_drone.pathreceived == 0):
		print("path yet to be received")
	print("half path received")
	y=e_drone.pointsInPath-1
	for i in the range(e_drone.pointsInPath+1,38):
		e_drone.path[i]=e_drone.path[y]
		y=y-1
	while not rospy.is_shutdown():
		e_drone.pid()
		if e_drone.success == 1:
			print("Disarming")
			e_drone.disarm()
			break

# if __name__ == '__main__':
# 	e_drone = Edrone()
# 	while not rospy.is_shutdown():
# 		#start the pid loop
# 		e_drone.pid()
# 		#if any of the set goal is reached
# 		if e_drone.success == 1:
# 			# If the goal is not initial waypoint (first point)
# 			if e_drone.initial != 0:
# 				#checking if the loop to the initial point is complete (drone comes back to initial waypoint after all the goal points)
# 				if e_drone.counter==e_drone.pointsInPath:
# 					errX=e_drone.drone_position[0]-e_drone.initial_waypoint[0]
# 					errY=e_drone.drone_position[1]-e_drone.initial_waypoint[1]
# 					errZ=e_drone.drone_position[2]-e_drone.initial_waypoint[2]
# 					print(e_drone.drone_position)
# 					if abs(errX)<0.5 and abs(errY)<0.5 and abs(errZ)<0.5:
# 						print("Disarm")
# 						e_drone.disarm()
# 						break
# 			# If new path is required)
# 			if e_drone.requested==0:
# 				#Get the new path points
# 				e_drone.reqpath.publish(e_drone.get)
# 				e_drone.requested=1