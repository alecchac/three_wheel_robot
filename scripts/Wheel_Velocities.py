#!/usr/bin/env python
"""
This node converts velocities from the world frame to wheel velocites of the robot
Current Mode: world frame velocities to PWM values of the robot
"""
import rospy
from numpy import array
from numpy import dot
from math import *
from three_wheel_robot.msg import Speeds
from three_wheel_robot.msg import robot_info


def main():
	msgct=0
	rospy.init_node('Wheel_Velocites',anonymous=True)
	#initializes the  listeners and publishers
	vels=robot_info_listener()
	robot_listener=robot_info_listener()
	pwm_msg = Speeds()
	#recieve velocities from the controller
	rospy.Subscriber('cmd_vel',robot_info,vels.callback)
	#recive angles from the KF
	rospy.Subscriber('Pose_hat',robot_info,robot_listener.callback)
	pub = rospy.Publisher('speeds',Speeds,queue_size=1)
	#rate of loop
	rate=rospy.Rate(60)#hz
	
	while not rospy.is_shutdown():
		#checks if subscriber has recieved a message (max linear velocity must not be zero)
		if vels.max_vel_linear !=0:
			wheel_output=calc_wheel_velocities(vels.v_x,vels.v_y,vels.omega,robot_listener.theta,vels.max_vel_linear,vels.max_vel_angular)
			pwm_msg.s1 = wheel_output[0].round(0)
			pwm_msg.s2 = wheel_output[1].round(0)
			pwm_msg.s3 = wheel_output[2].round(0)
			pub.publish(pwm_msg)
			rate.sleep()
		elif msgct == 0:
			rospy.loginfo('waiting for subscriber')
			msgct+=1
	rospy.spin()

def calc_wheel_velocities(v_x,v_y,omega,theta,maxLinear,maxAngular):
	d = 20
	#initialize rotation matrix
	rotation = array([[cos(theta), sin(theta), 0],[-sin(theta),cos(theta),0],[0,0,1]])
	#robot frame velocities to robot wheel velocities array
	wheel=array([[-sin(pi/3),cos(pi/3),d],[0,-1,d],[sin(pi/3),cos(pi/3),d]])
	#do the transformations using matrix multilplication (numpy)
	world_velocities = array([[v_x],[v_y],[omega]])
	robot_velocities = dot(rotation,world_velocities)
	wheel_velocities = dot(wheel,robot_velocities)

	#convert to PWM
	max_pwm=70.0
	absv = abs(wheel_velocities)
	# mag=sqrt((v_x**2)+(v_y**2))
	#maxWheel=abs(wheel_velocities).max()

	#checks if above max PWM
	for i in range(3):
		if absv[i]>max_pwm:
			if wheel_velocities[i]>0:
				wheel_velocities[i]=max_pwm
			else:
				wheel_velocities[i]=-max_pwm
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
