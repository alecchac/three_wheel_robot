#!/usr/bin/env python
import matplotlib.pyplot as plt
import math
import time
import rospy
from three_wheel_robot.msg import robot_info

def main():
	#init node
	rospy.init_node('Three_Wheel_Sim', anonymous=True)
	#initialize bob robot (x,y,theta(radians))
	bob = robot(0,0,0)
	bob.init_plot()
	#initialize messages
	pubInfo = robot_info()
	#initialize listener class
	current_pos = robot_info_listener()
	#init publisher and subscriber
	pub=rospy.Publisher('current_robot_info',robot_info,queue_size=1)
	rospy.Subscriber('Pose_hat',robot_info,current_pos.callback)

	while not rospy.is_shutdown():
		#take velocities from controller and update velocites in robot class
		bob.update_velocities(current_pos.v_x,current_pos.v_y,current_pos.omega)
		#clears the plot
		bob.clear_robot()
		#calculates new position from new velocites and current position
		#bob.update_pos()
		bob.set_pos(current_pos.x,current_pos.y,current_pos.theta)
		#displays on plot
		bob.display_robot()
		#publish current robot pose and velocities to the robot_info topic
		pubInfo.x=bob.x
		pubInfo.y=bob.y
		pubInfo.theta=bob.theta
		pubInfo.v_x=bob.v_x
		pubInfo.v_y=bob.v_y
		pubInfo.omega=bob.omega
		pub.publish(pubInfo)
		print pubInfo
		#allows plot to plot continuously
		plt.pause(.001)
	rospy.spin()

class robot_info_listener(object):
	""" robot info listener"""
	def __init__(self):
		self.x=0.0
		self.y=0.0
		self.theta=0.0
		self.v_x=0.0
		self.v_y=0.0
		self.omega=0.0
		self.max_vel_linear=0.0
		self.max_vel_angular=0.0
		

	def callback(self,data):
		self.x=data.x
		self.y=data.y
		self.theta=data.theta
		self.v_x=data.v_x
		self.v_y=data.v_y
		self.omega=data.omega
		self.max_vel_linear=data.max_vel_linear
		self.max_vel_angular=data.max_vel_angular

class robot(object) :
	"""A three wheeled robot modeled in the world frame
	
	TODO: Resolve angle handling issue
	
	Attributes:
		x
		y
		theta
		v_x
		v_y
		omega
		poly
		vel_arrow
		theta_arrow
		time_record
	"""
	v_x=0
	v_y=0
	omega=0
	
	
	def __init__(self,x,y,theta):
		self.x=x
		self.y=y
		self.theta=theta
		self.time_record=time.time()
		self.t=1
		self.vel_arrow=plt.arrow(x, y, 0, 1, head_width=1, head_length=1, fc='k', ec='k')
		self.theta_arrow=plt.arrow(x, y, math.cos(theta), math.sin(theta), head_width=1, head_length=1, fc='r', ec='r')
	
	def init_plot(self):
		#allows plotting continuously
		plt.ion()
		#set bounds of the plot
		plt.xlim([0,580])
		plt.ylim([0,350])
		plt.grid('on')
		
	def update_pos(self):
		"""Velocity model based off of current time
		"""
		#calculate elapsed time from when this function was previously run
		self.t=time.time()-self.time_record
		#checks if program has not been run for more than 2 seconds
		#if so, resets the time interval to prevent jumping
		if self.t>2:
			self.t=0
			
		#updates position to with time interval t
		self.x=self.x+self.v_x*self.t
		self.y=self.y+self.v_y*self.t
		self.theta=self.theta+self.omega*self.t
		
		#update time record
		self.time_record=time.time()
		
		#check if theta is between -2pi and 2pi
		if self.theta> 2*math.pi:
			self.theta-=math.pi*2
		elif self.theta < -math.pi*2:
			self.theta+=math.pi*2
	def set_pos(self,x,y,theta):
		self.x=x
		self.y=y
		self.theta=theta
	
	def display_robot(self):
		#velocity vector arrow
		self.vel_arrow=plt.arrow(self.x, self.y, self.v_x, self.v_y, head_width=1, head_length=1, fc='k', ec='k')
		#theta vector arrow
		self.theta_arrow=plt.arrow(self.x, self.y, 5*math.cos(self.theta), 5*math.sin(self.theta), head_width=1, head_length=1, fc='r', ec='r')
	
	def clear_robot (self):
		#removes previous instance of the model on plot
		self.vel_arrow.remove()
		self.theta_arrow.remove()
		
	def update_velocities(self,v_x,v_y,omega):
		self.v_x=v_x
		self.v_y=v_y
		self.omega=omega
		
		
if __name__== "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass


		
	
		
	
