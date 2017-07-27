#! /usr/bin/python

import matplotlib.pyplot as plt 
import numpy as np
import math
from math import pow,atan2,sqrt,cos,sin,asin,pi
import rosbag
import rospy
from std_msgs.msg import UInt8, String
from geometry_msgs.msg import Twist, Vector3, PoseWithCovarianceStamped
import time
import tf

#custom messages
from three_wheel_robot.msg import robot_info


#custom class for Kalman Filter
from KF_class.kalman import KF
from KF_class.Three_wheel_robot_system_1 import A,B,C,F
from KF_class.kalman_settings_1 import R,Q,K


#listener class (comes from controller node)
class cmd_vel_listener(object):

    

	def __init__(self):

		self.x=0.0
		self.y=0.0
		self.theta=0.0
		self.v_x=0.0
		self.v_y=0.0
		self.omega=0.0
		

	def callback(self,data):

		self.x = data.x
		self.y = data.y
		self.theta = data.theta
		self.v_x = data.v_x
		self.v_y = data.v_y
		self.omega = data.omega


#listener class (comes from camera node)
class camera_listener(object):



	def __init__(self):

		
        #world frame position
		self.x = 0.0
		self.y = 0.0
		self.z = 0.0
		#quaternions
		self.quaternion_x = 0.0
		self.quaternion_y = 0.0
		self.quaternion_z = 0.0
		self.quaternion_w = 0.0
		#world frame orientation
		self.theta_x = 0.0
		self.theta_y = 0.0
		self.theta_z = 0.0
		#covariance matrix
		self.cov = [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
				[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
				[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
				[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
				[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
				[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]


	def callback(self,data):

		

        #free space position 
		self.x = data.pose.pose.position.x
		self.y = data.pose.pose.position.y
		self.z = data.pose.pose.position.z
        #angular orientation in quaternions
		self.quaternion_x = data.pose.pose.orientation.x
		self.quaternion_y = data.pose.pose.orientation.y
		self.quaternion_z = data.pose.pose.orientation.z
		self.quaternion_w = data.pose.pose.orientation.w
        # define an array with quaternions
		quaternion = (self.quaternion_x,
							self.quaternion_y,
							self.quaternion_z,
							self.quaternion_w)

        # define obtain euler angles in radians
		euler = tf.transformations.euler_from_quaternion(quaternion)
		self.theta_x = euler[0]
		self.theta_y = euler[1]
		self.theta_z = euler[2]

        #covarince matrix from camera
		self.cov = [data.pose.covariance[0:6],
				[data.pose.covariance[6:12]],
				[data.pose.covariance[12:18]],
				[data.pose.covariance[18:24]],
				[data.pose.covariance[24:30]],
				[data.pose.covariance[30:36]]]


if __name__ == '__main__':



	

#-----------------Set up for all your fuctions -----------------
#Here you put init fuctions or constant definitions for your own fuctions

    #------------------ ROS set up -----------------------------
    # Start node
	rospy.init_node('Three_wheel_robot_KF', anonymous=True)

	#initialize messages
	pubInfo = robot_info()

	#create object from listener classes
	control_vels = cmd_vel_listener()
	measure_pose = camera_listener()

	#init publisher and subscribers
    	#Publisher of this node (Topic, mesage) 
	pub = rospy.Publisher('Pose_hat', robot_info, queue_size=10)
    	#Subscribe to controller (Topic, message, callback function)
	rospy.Subscriber('cmd_vel',robot_info,control_vels.callback)
    	#Subscribe to camera
    	#rospy.Subscriber('/ram/amcl_pose',PoseWithCovarianceStamped,measure_pose.callback)

    # ------------------- End of ROS set up --------------------



    #-------------------KF set-up ------------------------------
    #create the object from kalman filter class
    
	filter = KF(A,B,C,R,Q,K)
	#-previous state
	mt_1 = 0.0
	mt_2 = 0.0
	mt_3 = 0.0


	mt_ = [[mt_1],
		[mt_2],
		[mt_3]]

	#previous covariance
	St_1 = 0.0
	St_2 = 0.0
	St_3 = 0.0

	St_ = [[St_1,0,0],
		[0,St_2,0],
		[0,0,St_3]]

	#initial condition of kalman filter

	last_pkg = [mt_, St_]

	ut = [[0],
		[0],
		[0]]

    # dt = 0.00006
   	t1 = time.time()
	t2 = time.time()
    #----------------End of KF set up --------------------

#--------------------End of Definitions and Set-up--------



#------------------------- Main Loop --------------------

	while not rospy.is_shutdown():
	
	#for t in range(0,100):

		#-----------------Get measurments---------------------

		#Here you obtain zt  which is a list containing all the elements
		#from the IMU using the callback function and then converted into 
		#euler angles.
		#be sure to encapsulate in the same order as shown below

		# zt = [[m_pos_x],       # x position
		#       [m_pos_y],       # y position
		#       [m_theta],       # theta angular orientation

		#	zt = [[measure_pose.x],
		#      [measure_pose.y],
		#      [measure_pose.theta_z]]

		zt = [[0],[0],[0]]	
		#----------------- Get the sensor Covariance -------------
		# This is the covariance matrix Q wich comes from the measurments

		# Q = covariance_zt

			#Q = measure_pose.cov #revisar que covarianza es
		#----------------- Get the system input -------------
		# From the topic you get the input apply to system in this case
		# World frame velocities as shown below

		# ut = [[Vx],           #linear x velocity
		#       [Vy],           #linear y velocity
		#       [omega]]        #angular omega velocity

		ut = [[control_vels.v_x],
		      [control_vels.v_y],
		      [control_vels.omega]]
		# ut = [[3],[2],[1]]
		#print(ut)

		vel_x = control_vels.v_x
		vel_y = control_vels.v_y
		omega = control_vels.omega

		#----------- Beginnign of Kalman Filter -----------
		t2 = time.time()
		dt = t2-t1
		# main function to compute the Kalman Filter
		pkg = filter.KF_compute(last_pkg[0], last_pkg[1], ut, zt, Q, dt)
		#pkg = filter.compute(last_pkg[0],ut,dt)
		# separate state vector into individual arrays

		t1 = time.time()

		last_pkg = pkg 
		mt = pkg[0]
		#output feedback controller
		#ut = np.dot(F,mt)
		#KF results you can separate the list and plot individually
		#according to the mt index shown below
		pos_x = mt[0]
		pos_y = mt[1]
		theta = mt[2]
		#----------------End of Kalman Filter -------------------

		#---------------Pubblish the results -------------------
		#linear and angular orientation
		pubInfo.x = pos_x
		pubInfo.y = pos_y
		pubInfo.theta=theta


		print(pos_x)
		#linear and angular velocity
		pubInfo.v_x = vel_x
		pubInfo.v_y = vel_y
		pubInfo.omega = omega

		pub.publish(pubInfo)
       
	print("Exiting ... ")


