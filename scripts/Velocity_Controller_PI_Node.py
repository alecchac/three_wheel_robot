#!/usr/bin/env python
import time
import math
import rospy
from three_wheel_robot.msg import waypoints
from three_wheel_robot.msg import robot_info

def main():
	#initialize node
	rospy.init_node('Controller',anonymous=True)
	#initialize controller
	max_linear_speed=70 #pixels/sec
	max_angular_speed=1 #radians/sec
	Kc_linear=1
	Ti_linear=3 #Ki=Kc/Ti
	Kc_angular=.1 
	Ti_angular=3
	Kd_linear=.1
	Kd_angular=5
	bobControl=Velocity_Controller_PI(max_linear_speed,max_angular_speed,Kc_linear,Ti_linear,Kc_angular,Ti_angular,Kd_linear,Kd_angular)
	#initialize listener classes
	bobWay=waypoint_listener()
	bobInfo=robot_info_listener()
	#init subscribers
	rospy.Subscriber('goal_pos',waypoints,bobWay.callback)
	rospy.Subscriber('Pose_hat',robot_info,bobInfo.callback)
	#init publishers and publish message
	pub=rospy.Publisher('cmd_vel',robot_info,queue_size=1)
	bobPubInfo=robot_info()
	#set tolerances for goal pose 
	distance_tolerance=50
	angle_tolerance=.2
	
	while (not rospy.is_shutdown()) :
		#checks if waypoint message is not empty
		if len(bobWay.x)>0:
			#loops through all waypoints
			for i in range(len(bobWay.x)):
				print(bobWay.theta[i])
				#resets Integrator sums
				bobControl.reset_Iterms()
				#checks if robot within the distance and angle tolerances
				while getDistance(bobWay.x[i],bobWay.y[i],bobInfo.x,bobInfo.y)>distance_tolerance or abs(bobWay.theta[i]-bobInfo.theta)>angle_tolerance:
				#while abs(bobWay.theta[i]-bobInfo.theta)>angle_tolerance:
					#updates the current goal pose and the current pose of the robot for the controller class to use
					bobControl.update_current_positions(bobWay.x[i],bobWay.y[i],bobWay.theta[i],bobInfo.x,bobInfo.y,bobInfo.theta)
					#calculates the velocities that the robot needs to go (need to specify minimum velocity in the function)
					vels=bobControl.update_velocities(bobWay.min_velocity[i])
					#publish velocities to topic cmd_vel
					bobPubInfo.v_x=vels[0]
					bobPubInfo.v_y=vels[1]
					bobPubInfo.omega=vels[2]
					bobPubInfo.max_vel_linear=bobControl.saturation_l	
					bobPubInfo.max_vel_angular=bobControl.saturation_a		
					pub.publish(bobPubInfo)
			#once done set velocities to zero and publish velocities
			bobPubInfo.v_x=0
			bobPubInfo.v_y=0
			bobPubInfo.omega=0				
			pub.publish(bobPubInfo)
			print 'done'
			break		
	rospy.spin()

def getDistance(destX,destY,curX,curY):
	return math.sqrt((destX-curX)**2+(destY-curY)**2)

class waypoint_listener(object):
	""" waypoint listener"""
	def __init__(self):
		self.x=()
		self.y=()
		self.theta=()
		self.min_velocity=()
		

	def callback(self,data):
		self.x=data.x
		self.y=data.y
		self.theta=data.theta
		self.min_velocity=data.min_velocity

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


class Velocity_Controller_PI(object):
	"""PI Controller
	
	inputs:Linear saturation velocity, angular saturation velocity(rad/s), 
	Kc linear,Ti Linear,Kc Angular, Ti_angular
	outputs:v_x,v_y,omega(rad/s)
	
	**note: angular velocities are in radians/sec
	        Ki=Kc/Ti
	
	Dependencies:time
	"""
	
	def __init__(self,saturation_linear,saturation_angular,Kc_linear,Ti_linear,Kc_angular,Ti_angular,Kd_linear,Kd_angular):
		self.saturation_l=float(saturation_linear)
		self.saturation_a=float(saturation_angular)
		
		#if velocity input is saturated
		self.satL=False
		self.satT=False
		
		self.Ki_l=float(Kc_linear)/float(Ti_linear)
		self.Kc_l=float(Kc_linear)
		self.Ki_a=float(Kc_angular)/float(Ti_angular)
		self.Kc_a=float(Kc_angular)
		self.Ti_linear=float(Ti_linear)
		self.Ti_angular=float(Ti_angular)
		self.Kd_linear=float(Kd_linear)
		self.Kd_angular=float(Kd_angular)
		
		self.setX=0.0
		self.setY=0.0
		self.setTheta=0.0
		
		self.currentX=0.0
		self.currentY=0.0
		self.currentTheta=0.0
		
		self.errorX=0.0
		self.errorY=0.0
		self.errorTheta=0.0
		self.last_error_x=0.0
		self.last_error_y=0.0
		self.last_error_theta=0.0
		
		#integrator sums
		self.IX=0.0
		self.IY=0.0
		self.ITheta=0.0

		#Derivitive terms
		self.DX=0.0
		self.DY=0.0
		self.Dtheta=0.0
		
		self.last_time = time.time()
		
	def update_velocities(self,minVel):
		#update errors
		self.errorX=self.setX-self.currentX
		self.errorY=self.setY-self.currentY
		self.errorTheta=self.setTheta-self.currentTheta

		#Proportional Contribution
		v_xP=self.errorX*self.Kc_l
		v_yP=self.errorY*self.Kc_l
		v_thetaP=self.errorTheta*self.Kc_a

		#calculate time from last function run
		delta_t=time.time()-self.last_time
		#resets delta_t if function has not been run for a while
		if delta_t>2:
			delta_t=0
		
		#Integral Contribution 
		#Sums up the error term with the delta t to remove steady state error
		#incorporates windup protection when motor is saturated
		if not self.satL:
			self.IX+=self.Ki_l*self.errorX*delta_t
			self.IY+=self.Ki_l*self.errorY*delta_t
		if not self.satT:
			self.ITheta+=self.Ki_a*self.errorTheta*delta_t

		#derivitive contribution
		self.DX=self.Kd_linear*((self.errorX-self.last_error_x)/delta_t)
		self.DY=self.Kd_linear*((self.errorY-self.last_error_y)/delta_t)
		self.Dtheta=self.Kd_angular*((self.errorTheta-self.last_error_theta)/delta_t)

		#set last error
		self.last_error_x=self.errorX
		self.last_error_y=self.errorY
		self.last_error_theta=self.errorTheta


		#Set last time
		self.last_time=time.time()
	
		#Add both contributions
		v_x=v_xP+self.IX+self.DX
		v_y=v_yP+self.IY+self.DY
		v_theta=v_thetaP+self.ITheta+self.Dtheta
		
		#motor saturtaion (sets max speed)
		mag=math.sqrt((v_x**2)+(v_y**2))
		multiplierMax=self.saturation_l/mag 
		#x saturation
		if mag>self.saturation_l:
			#if motor is saturated, apply windup protection and stops integrating
			self.satL=True
			v_x*=multiplierMax
			v_y*=multiplierMax
		else:
			#if saturation does not occur, there is no effect
			self.satL=False
		#theta saturation
		if v_theta>self.saturation_a:
			self.satT=True
			v_theta=self.saturation_a
		elif v_theta<-self.saturation_a:
			self.satT=True
			v_theta=-self.saturation_a
		else:
			self.satT=False
		
		#controls minimum speed: if velocities are too low, increase to the min velocity
		mag=math.sqrt((v_x**2)+(v_y**2))
		multiplier=minVel/mag
		if mag<minVel:
			v_x*=multiplier
			v_y*=multiplier

		#return calculated velocities
		return [v_x,v_y,v_theta]
		
	
	def update_current_positions(self,setX,setY,setTheta,currentX,currentY,currentTheta):
		self.currentX=currentX
		self.currentY=currentY
		self.currentTheta=currentTheta
		self.setX=setX
		self.setY=setY
		self.setTheta=setTheta
		
	def reset_Iterms(self):
		self.IX=0.0
		self.IY=0.0
		self.ITheta=0.0
	
	def update_k_terms(self,Kc_linear,Ti_linear,Kc_angular,Ti_angular):
		self.Kc_l=Kc_linear
		self.Ti_linear=Ti_linear
		self.Kc_a=Kc_angular
		self.Ti_angular=Ti_angular

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
