#!/usr/bin/env python

import rospy
from numpy import array
from numpy import dot
from math import *
from three_wheel_robot.msg import Speeds
from three_wheel_robot.msg import robot_info

def main():
	msgct=0
	vels=robot_info_listener()
	robot_listener=robot_info_listener()
	rospy.Subscriber('cmd_vel',robot_info,vels.callback)
	rospy.Subscriber('current_robot_info',robot_info,robot_listener.callback)
	rospy.init_node('Wheel_Velocites',anonymous=True)
	rate=rospy.Rate(10)#hz
	pub = rospy.Publisher('speeds',Speeds,queue_size=1)
	pwm_msg = Speeds()
	while not rospy.is_shutdown():
		if vels.max_vel_linear !=0:
			wheel_output=calc_wheel_velocities(vels.v_x,vels.v_y,vels.omega,robot_listener.theta,vels.max_vel_linear,vels.max_vel_angular)
			pwm_msg.s1 = wheel_output[0].round(0)
			pwm_msg.s2 = wheel_output[1].round(0)
			pwm_msg.s3 = wheel_output[2].round(0)
			pub.publish(pwm_msg)
			print pwm_msg
			rate.sleep()
		elif msgct == 0:
			rospy.loginfo('waiting for subscriber')
			msgct+=1
	rospy.spin()

def calc_wheel_velocities(v_x,v_y,omega,theta,maxLinear,maxAngular):
	d = 74*10**-3
	rotation = array([[cos(theta), sin(theta), 0],[-sin(theta),cos(theta),0],[0,0,1]])
	wheel=array([[-sin(pi/3),cos(pi/3),d],[0,-1,d],[sin(pi/3),cos(pi/3),d]])
	world_velocities = array([[v_x],[v_y],[omega]])
	robot_velocities = dot(rotation,world_velocities)
	wheel_velocities = dot(wheel,robot_velocities)
	#convert to PWM
	max_pwm=60
	mag=sqrt((v_x**2)+(v_y**2))
	maxWheel=abs(wheel_velocities).max()
	multiplierScaler=(mag/maxLinear)*(omega/maxAngular)
	wheel_velocities*=multiplierScaler
	multiplierPWM=max_pwm/maxWheel
	wheel_velocities*=multiplierPWM
	return wheel_velocities
	

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

	
	
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
