#! /usr/bin/python

import numpy as np
import math
from math import pow,atan2,sqrt,cos,sin,asin,pi
import rospy
from std_msgs.msg import UInt8, String
from geometry_msgs.msg import Twist, Vector3, PoseWithCovarianceStamped
import time
import tf

#custom messages
from three_wheel_robot.msg import robot_info
from three_wheel_robot.msg import Espeeds
from aruco_node.msg import measurement

#custom class for Kalman Filter
from KF_class.kalman import KF
from KF_class.Three_wheel_robot_system_1 import A,B,C,F
from KF_class.kalman_settings_1 import R,Q,K
from Master_Settings import d,r,SF

class measurment_listener(object):
	""" measure listener"""
	def __init__(self):
		self.last_time = time.time()
		self.x = 0
		self.y = 0
		self.theta = 0
		self.cov_x = 0
		self.cov_y = 0
		self.cov_theta = 0
		self.D2C = 0
		self.markernum = 0
		self.isValid = False
	def callback(self,info):
		self.last_time = time.time()
		self.x = info.x
		self.y = info.y
		self.theta = info.theta
		self.cov_x = info.cov_x
		self.cov_y = info.cov_y
		self.cov_theta = info.cov_theta
		self.D2C = info.D2C
		self.markernum = info.markernum
		self.isValid = info.isValid

class encoder_listener(object):
    """ Encoder listener"""
    def __init__(self):
        self.e_s1 = 0.0
        self.e_s2 = 0.0
        self.e_s3 = 0.0

    def callback(self,info):
        self.e_s1 = info.e_s1
        self.e_s2 = info.e_s2
        self.e_s3 = info.e_s3


#listener class (comes from controller node)
class robot_info_listener(object):
	""" robot info listener"""
	def __init__(self):
		self.x=0.0
		self.y=0.0
		self.theta=0.0
		self.v_x=0.0
		self.v_y=0.0
		self.omega=0.0

	def callback(self,data):
		self.x=data.x
		self.y=data.y
		self.theta=data.theta
		self.v_x=data.v_x
		self.v_y=data.v_y
		self.omega=data.omega


#listener class (comes from camera node)
class camera_listener(object):

	def __init__(self):

		self.last_time = time.time()
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

		self.last_time = time.time()

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
	control_vels = robot_info_listener()
	#measure_pose = camera_listener()
	encoder_vels = encoder_listener()
	pi_pose = measurment_listener()

	#init publisher and subscribers
	#Publisher of this node (Topic, mesage) 
	pub = rospy.Publisher('Pose_hat', robot_info, queue_size=10)
	#Subscribe to Encoder
	rospy.Subscriber('encoder_omegas',Espeeds,encoder_vels.callback)
	#Subscribe to camera
	#rospy.Subscriber('/ram/amcl_pose',PoseWithCovarianceStamped,measure_pose.callback)
	#PiCameraSub
	rospy.Subscriber('/aruco/robot_pose',measurement,pi_pose.callback)
	#rospy.Subscriber('',PoseWithCovarianceStamped,measure_pose.callback)

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

	#initialize Q when camera is not availible
	Q_no_camera = [[10**15,0,0],
		[0,10**15,0],
		[0,0,10**15]]

    # dt = 0.00006
   	t1 = time.time()
	t2 = time.time()
    # #----------------End of KF set up --------------------
	pos_x = [0.0]
	pos_y = [0.0]
	theta = [0.0]
#--------------------End of Definitions and Set-up--------

	zt = [[0],[0],[0]]

	t = 0
#------------------------- Main Loop --------------------
	# plt.figure()
	# plt.ion()

	while not rospy.is_shutdown():
		#measure time from last camera measurement
		no_camera_signal_time = time.time()-pi_pose.last_time

		#----------------Get World Velocities from the encoder measurements---------------------
		#get theta from previous kf output
		theta = last_pkg[0][2][0]
		print theta
		#multiply encoder omegas to get linear velocities of wheel (V=omega*r)
		v0 = encoder_vels.e_s1 * r
		v1 = encoder_vels.e_s2 * r
		v2 = encoder_vels.e_s3 * r
		#use inverse equations to get robot frame velocities
		v = (sqrt(3.0)/3.0)*(v2-v0)
		vn = ((1.0/3.0)*(v2+v0))-((2.0/3.0)*v1)
		omega = (1/(3.0*d))*(v0+v1+v2)
		robot_velocities = np.array([[v],[vn],[omega]])
		#define rotattion matrix with -theta to convert back to world frame
		rotation_matrix = np.array([[cos(-theta), sin(-theta), 0],[-sin(-theta),cos(-theta),0],[0,0,1]])
		#do calculation and publish to Pose_hat
		world_vels = np.dot(rotation_matrix,robot_velocities)
		pubInfo.v_x = world_vels[0][0]
		pubInfo.v_y = world_vels[1][0]
		pubInfo.omega = world_vels[2][0]
		

		#-----------------Get measurments---------------------

		#Here you obtain zt  which is a list containing all the elements
		#from the IMU using the callback function and then converted into 
		#euler angles.
		#be sure to encapsulate in the same order as shown below

		# zt = [[m_pos_x],       # x position
		#       [m_pos_y],       # y position
		#       [m_theta]]       # theta angular orientation
		'''
		zt = [[measure_pose.x],
			[measure_pose.y],
			[measure_pose.theta_z]]
		'''
		zt = [[pi_pose.x],
			[pi_pose.y],
			[pi_pose.theta]]

			
		#----------------- Get the sensor Covariance -------------
		# This is the covariance matrix Q wich comes from the measurments

		# Q = covariance_zt
		#If camera measurement is not detected, make Q(camera cov) very large
		if pi_pose.isValid:
			filter.Q = Q
		else:
			filter.Q = Q_no_camera

		#Q = measure_pose.cov #revisar que covarianza es


		#----------------- Get the system input -------------
		# From the topic you get the input apply to system in this case
		# World frame velocities as shown below

		# ut = [[Vx],           #linear x velocity
		#       [Vy],           #linear y velocity
		#       [omega]]        #angular omega velocity

		ut = [[world_vels[0][0]],
			[world_vels[1][0]],
			[world_vels[2]][0]]

		#ut = [[0],[0],[0]]
		#print(ut)

		#----------- Beginnign of Kalman Filter -----------
		t2 = time.time()
		dt = t2-t1

		# main function to compute the Kalman Filter
		#last_pkg[0]  this is last state
		#last_pkg[1] this is the prev. covariance 
		
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

		#Publish position calculated from Kalman Filter and Velocities from encoder
		pub.publish(pubInfo)
       
	print("Exiting ... ")


