#!/usr/bin/env python2

'''
* Team Id : HB#5374
* Author List : Divyansh Malhotra, Chetan Chawla, Himanshu, Yashvi Gulati
* Filename: 5374_pos_hold.py
* Theme: Hungry Bird
* Classes: Edrone
* *	Functions:_init_,ar,n disarm, whycon_callback,altitude_set_pid, pitch_set_pid. roll_set_pid, yaw_set_pid, setyaw, pid, getInp 
* Global Variables: lasttime, currenttime
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
lasttime=0.0  #to save the last time rc values were updated
currenttime=0.0  #to save the current time

class Edrone():
	'''
	* Function Name: _init_
	* Logic: Initialises an Edrone object and creates the subscribers and publishers. It also arms the drone.
	* Example Call: Called when an Edrone object is made
	'''
	def __init__(self):
		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z,yaw_value]
		self.drone_position = [0.0,0.0,0.0,0.0]	
		
		# [x_setpoint, y_setpoint, z_setpoint, yaw_value_setpoint]
		#self.setpoint = [-8.39,4.98,27.92,0.00] # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly
		self.setpoint = [0,0,20,292] #292 is the desired value of yaw for the drone to be straight
		#Here, the yaw is taken from data_via_tosservice while all the other point values are taken from the Target whycon marker (set renderable) as detected by the vision sensor.


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
		
		self.iterations=1 #it is used to save the no. of iterations to know total time
		
		
		#values of Kp, Kd and ki for [pitch, roll, throttle, yaw]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		
		self.Kp=[6,4.5,24,0.3]
		self.Ki=[0,0,0,0]
		self.Kd=[14.6,4.2,4,0.2]


		#-----------------------Other required variables for pid here ----------------------------------------------


        self.prev_error=[0.0,0.0,0.0,0.0]#It saves previous error (roor in the last pid function call)
        self.dif_error=[0.0,0.0,0.0,0.0]#It stores the difference between current and the previous error(differential)
        self.sum_values=[0.0,0.0,0.0,0.0]#It stores the addition of error terms for iterms
        self.error=[0.0,0.0,0.0,0.0]#It stores the current computed error between desired point and self point
        self.lasttime=time.time()#Used for sample time calling of the pid function
        self.currenttime=0
        self.max_values = [1575,1575,1800,1800]#The max and min values of pitch, roll, throttle and yaw are specified 
        self.min_values = [1425,1425,1200,1200]
	
		#----------------------------------------------------------------------------------------------------------

		# This is the sample time in which we are running pid. 
		self.sample_time = 0.010 # in seconds (10 milliseconds)


		# Making Publishers for /drone_command
		self.command_pub = rospy.Publisher('/drone_command', PlutoMsg, queue_size=1)
		
		# Subscribing to /whycon/poses,  /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll,input_key,yawyaw
		rospy.Subscriber('/whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		rospy.Subscriber('/input_key',Int16,self.getInp) #to get inputs from keyboard for emergency control
        rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
        rospy.Subscriber('/pid_tuning_roll',PidTune,self.roll_set_pid)
        rospy.Subscriber('/pid_tuning_yaw',PidTune,self.yaw_set_pid)
        rospy.Subscriber('/yawyaw',Float64,self.setyaw) #data_via_rosservice is publishing yaw values on yawyaw which we are subscribing
		#------------------------------------------------------------------------------------------------------------

		self.arm() # ARMING THE DRONE


	'''
	* Function Name: disarm
	* Logic: Disarming condition of the drone
	* Example Call: self.disarm()
	'''
	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)

	'''
	* Function Name: arm
	* Logic: Arming condition of the drone
	* Example Call: self.arm()
	'''
	
	def arm(self):

		self.disarm()
		# Best practise is to disarm and then arm the drone.
		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)	# Publishing /drone_command
		rospy.sleep(1)



	'''
	* Function Name: whycon_callback
	* Input: msg of type PoseArray
	* Logic: The function gets executed each time when /whycon node publishes /whycon/poses and updates the x,y and z values of the drone_position 
	'''
	
	def whycon_callback(self,msg):
		self.drone_position[0] = msg.poses[0].position.x
        self.drone_position[1] = msg.poses[0].position.y
        self.drone_position[2] = msg.poses[0].position.z
		


    #These functions have been commented and were used for PID tuning
	'''
	 
	def altitude_set_pid(self,alt):
		self.Kp[2] = alt.Kp #* 0.06 # This is just for an example. You can change the fraction value accordingly
		self.Ki[2] = alt.Ki #* 0.008
		self.Kd[2] = alt.Kd #* 0.3

	def pitch_set_pid(self,pit):
		self.Kp[0] = pit.Kp 
		self.Ki[0] = pit.Ki
		self.Kd[0] = pit.Kd*0.2

	def roll_set_pid(self,rol):
		self.Kp[1] = rol.Kp #* 0.06 
		self.Ki[1] = rol.Ki #* 0.008
		self.Kd[1] = rol.Kd*2 #* 0.3

	def yaw_set_pid(self,yaw):
		self.Kp[3] = yaw.Kp *0.1
		self.Ki[3] = yaw.Ki *0.05
		self.Kd[3] = yaw.Kd *0.1
	'''
	
	'''
	* Function Name: setyaw
	* Input: yaw2 of type Float64
	* Logic: The function gets executed each time when /yawyaw node publishes the yaw value and updates the yaw value of the drone_position 
	'''
	def setyaw(self,yaw2):
		self.drone_position[3]=yaw2.data
	


	'''
	* Function: pid
	* Logic: The function traverses the drone by publishing calculated values of Rcroll, Rcpitch, Rcyaw and Rcthrottle on /drone_command through PID
	* Example Call: self.pid() 
	'''
	def pid(self):
	
	# Steps of PID:
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
                self.sum_values=[a1+b1 for a1,b1 in zip(self.sum_values,self.error)]#sum of previous error values are added together for the given sample time
                if(self.currenttime-self.lasttime>=self.sample_time):

                	# If the sample time has elapsed, tune PID
		                
		                self.dif_error= [a1-b1 for a1,b1 in zip(self.error,self.prev_error)]#derivative term- difference between current and previous error
		                #Pitch, Roll, Throttle and Yaw are calculated with PID formulas
		                self.cmd.rcPitch=1500 + (self.error[1]*self.Kp[0]+self.dif_error[1]*self.Kd[0]/(self.sample_time)+self.sum_values[1]*self.Ki[0]*self.sample_time*self.iterations)
		                self.cmd.rcRoll=1500 - (self.error[0]*self.Kp[1]+self.dif_error[0]*self.Kd[1]/(self.sample_time)+self.sum_values[0]*self.Ki[1]*self.sample_time*self.iterations)
		                self.cmd.rcThrottle=1500 + self.error[2]*self.Kp[2]+self.dif_error[2]*self.Kd[2]/(self.sample_time)+self.sum_values[2]*self.Ki[2]*self.sample_time*self.iterations
		                self.cmd.rcYaw=1500-(self.error[3]*self.Kp[3]+self.dif_error[3]*self.Kd[3]/(self.sample_time)+self.sum_values[3]*self.Ki[3]*self.sample_time*self.iterations)
		                self.lasttime=self.currenttime#The last time stamp is updated
		                self.prev_error=self.error#Previous error term is updated
		               
		                #The calculated values are then checked if they are in the range of allowed values or not. 
		                #If not they are set to the maximum/minimum limit possible

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
		                
		                self.command_pub.publish(self.cmd) #it is used to publish the command to /drone_command to change the rc values of the drone
		                self.iterations=self.iterations+1 #increment the no. of iterations
                        

	'''
	* Function: getInp
	* Input: msg of data type Int64
	* Logic: It is a callback of /input_key.It is used to disarm the drone in case of emergencies to avoid crashes while tuning
	'''
	def getInp(self,msg):
		if msg==7:
			self.disarm()

				
if __name__ == '__main__':
	e_drone = Edrone()  #an drone is made of class Edrone and initialised 
	while not rospy.is_shutdown(): #while the roscore is running
		e_drone.pid() #this continuously runs the pid function to make the drone reach and then maintain its position
                