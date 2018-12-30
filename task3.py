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
								/drone_yaw

Rather than using different variables, use list. eg : self.setpoint = [1,2,3,4], where index corresponds to x,y,z and yaw_value...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.	
'''

# Importing the required libraries

from plutodrone.msg import *
from geometry_msgs.msg import PoseArray
from plutodrone.srv import *
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time
lasttime=0.0
currenttime=0.0

class Edrone():
	"""docstring for Edrone"""
	def __init__(self):
		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control
		#rospy.init_node('drone_board_data')
		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z,yaw_value]
		self.drone_position = [0.0,0.0,0.0,0.0]	
		
		# [x_setpoint, y_setpoint, z_setpoint, yaw_value_setpoint]
		#self.setpoint = [-8.39,4.98,27.2,0.00] # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly
		self.setpoint = [0,0,20,0]
		#Here, the yaw desired is taken to be 0 while all the other point values are taken from the Target whycon marker (set renderable) as detected by the vision sensor.

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
		self.iterations=1

		#initial setting of Kp, Kd and ki for [pitch, roll, throttle, yaw]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters
		# self.Kp = [7.68,7.68,58.32,0]
		# self.Ki = [0,0,0.256,0]
		# self.Kd = [6.3,12.9,227.7,0]
		#As the drone is not rotating itself, we are using yaw parameters as 0.
		# self.Kp = [3.8,3.8,0,0]
		# self.Ki = [0,0,0,0]
		# self.Kd = [30,30,0,0]
		self.Kp = [1,0.5,24,0]
		self.Ki = [0,0,0,0]
		self.Kd = [7,3.5,4,0]


		#-----------------------Add other required variables for pid here ----------------------------------------------
		self.prev_error=[0.0,0.0,0.0,0.0]#It saves previous error (roor in the last pid function call)
		self.dif_error=[0.0,0.0,0.0,0.0]#It stores the difference between current and the previous error(differential)
		self.sum_values=[0.0,0.0,0.0,0.0]#It stores the addition of error terms for iterms
		self.error=[0.0,0.0,0.0,0.0]#It stores the current computed error between desired point and self point
		self.lasttime=time.time()#Used for sample time calling of the pid function
		self.currenttime=0
		self.max_values = [1800,1800,2000,1800]#The max and min values of pitch, roll, throttle and yaw are specified
		self.min_values = [1200,1200,1400,1200]


		# Hint : Add variables for storing previous errors in each axis, like self.prev_values = [0,0,0,0] where corresponds to [pitch, roll, throttle, yaw]
		#		 Add variables for limiting the values like self.max_values = [1800,1800,1800,1800] corresponding to [pitch, roll, throttle, yaw]
		#													self.min_values = [1200,1200,1200,1200] corresponding to [pitch, roll, throttle, yaw]
		#																	You can change the upper limit and lower limit accordingly. 
		#----------------------------------------------------------------------------------------------------------

		# This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
		self.sample_time = 0.060 # in seconds (60 milliseconds)


		# Making Publishers for /drone_command, /alt_error, /pitch_error, /roll_error, /yaw_error
		self.command_pub = rospy.Publisher('/drone_command', PlutoMsg, queue_size=1)
		#------------------------Add other ROS Publishers here- Errors are piblished-----------------------------------------------------
		self.alterr=rospy.Publisher('/alt_error',Float64,queue_size=1)
		self.pitcherr=rospy.Publisher('/pitch_error',Float64,queue_size=1)
		self.rollerr=rospy.Publisher('/roll_error',Float64,queue_size=1)
		self.yawerr=rospy.Publisher('/yaw_error',Float64,queue_size=1)

		#-----------------------------------------------------------------------------------------------------------


		# Subscribing to /whycon/poses, /drone_yaw, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('/whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		rospy.Subscriber('/input_key',Int16,self.getInp)
		#-------------------------Add other ROS Subscribers here----------------------------------------------------
		rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
		rospy.Subscriber('/pid_tuning_roll',PidTune,self.roll_set_pid)
		rospy.Subscriber('/pid_tuning_yaw',PidTune,self.yaw_set_pid)
		#rospy.Subscriber('/drone_yaw',Float64,self.setyaw)
		#------------------------------------------------------------------------------------------------------------
		rospy.Subscriber('/yawyaw',Float64,self.setyaw)
		#data = rospy.Service('PlutoService', PlutoPilot, self.setyaw)
		self.arm() # ARMING THE DRONE


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



	# Callback function for /pid_tuning_altitude
	# This function gets executed each time when /tune_pid publishes /pid_tuning_altitude
	def altitude_set_pid(self,alt):
		self.Kp[2] = alt.Kp #* 0.06 # This is just for an example. You can change the fraction value accordingly
		self.Ki[2] = alt.Ki #* 0.008
		self.Kd[2] = alt.Kd #* 0.3

	#----------------------------Define callback function like altitide_set_pid to tune pitch, roll and yaw as well--------------

	def pitch_set_pid(self,pit):
		self.Kp[0] = pit.Kp #* 0.06 
		self.Ki[0] = pit.Ki #* 0.008
		self.Kd[0] = pit.Kd #* 0.3
	def roll_set_pid(self,rol):
		self.Kp[1] = rol.Kp #* 0.06 
		self.Ki[1] = rol.Ki #* 0.008
		self.Kd[1] = rol.Kd #* 0.3
	def yaw_set_pid(self,yaw):
		self.Kp[3] = yaw.Kp #* 0.06
		self.Ki[3] = yaw.Ki #* 0.008
		self.Kd[3] = yaw.Kd #* 0.3
	def setyaw(self,yaw2):
		self.drone_position[3]=yaw2
		print(yaw2)
	#----------------------------------------------------------------------------------------------------------------------


	def pid(self):
	#-----------------------------Write the PID algorithm here--------------------------------------------------------------

	# Steps:
	# 	1. Compute error in each axis. eg: error[0] = self.drone_position[0] - self.setpoint[0] ,where error[0] corresponds to error in x...
	#	2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer Getting_familiar_with_PID.pdf to understand PID equation.
	#	3. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
	#	4. Reduce or add this computed output value on the avg value ie 1500. For eg: self.cmd.rcRoll = 1500 + self.out_roll. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
	#	5. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
	#	6. Limit the output value and the final command value between the maximum(1800) and minimum(1200)range before publishing. For eg : if self.cmd.rcPitch > self.max_values[1]:
	#																														self.cmd.rcPitch = self.max_values[1]
	#	7. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
	#	8. Add error_sum
                self.currenttime=time.time()#Takes the current time stamp
                self.error=[a1-b1 for a1,b1 in zip(self.drone_position,self.setpoint)]#Calculates the error by subtracting drone position and setpoint positions
                self.pitcherr.publish(self.error[0])#Publishes the respective errors for the plot juggler
                self.rollerr.publish(self.error[1])
                self.alterr.publish(self.error[2])
                self.yawerr.publish(self.error[3])
                self.sum_values=[a1+b1 for a1,b1 in zip(self.sum_values,self.error)]#sum of previous error values are added together for the given sample time
                if(self.currenttime-self.lasttime>=self.iterations*self.sample_time):

                	# If the sample time has elapsed, tune PID
		                self.dif_error= [a1-b1 for a1,b1 in zip(self.error,self.prev_error)]#derivative term- difference between current and previous error
		                #Pitch, Roll, Throttle and Yaw are calculated with PID formulas
		                self.cmd.rcPitch=int(1500+self.error[0]*self.Kp[0]+self.dif_error[0]*self.Kd[0]/(self.iterations*self.sample_time)+self.sum_values[0]*self.Ki[0]*self.sample_time*self.iterations)
		                #self.cmd.rcRoll=int(1500+self.error[1]*self.Kp[1]+self.dif_error[1]*self.Kd[1]/(self.sample_time*self.iterations)+self.sum_values[1]*self.Ki[1]*self.sample_time*self.iterations)
		                self.cmd.rcThrottle=int(1500+self.error[2]*self.Kp[2]+self.dif_error[2]*self.Kd[2]/(self.sample_time*self.iterations)+self.sum_values[2]*self.Ki[2]*self.sample_time*self.iterations)
		                self.cmd.rcYaw=int(1500+self.error[3]*self.Kp[3]+self.dif_error[3]*self.Kd[3]/(self.sample_time*self.iterations)+self.sum_values[3]*self.Ki[3]*self.sample_time*self.iterations)
		                self.lasttime=self.currenttime#The last time stamp is updated
		                self.prev_error=self.error#Previous error term is updated
		                #self.sum_values=[0.0,0.0,0.0,0.0]#Sum of errors is reinitialized after sample time
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
		                self.iterations=self.iterations+1
                        

	def getInp(self,msg):
		if msg==7:
			self.disarm()
		elif msg==8:
			e_drone=Edrone()
			while not rospy.is_shutdown():
				e_drone.pid()
				
if __name__ == '__main__':
	e_drone = Edrone()
	while not rospy.is_shutdown():
		e_drone.pid()
                
