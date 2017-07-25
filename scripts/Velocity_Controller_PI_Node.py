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
	#(self,saturation_linear,saturation_angular,Kc_linear,Ti_linear,Kc_angular,Ti_angular)
	bobControl=Velocity_Controller_PI(5,1,.05,15,.5,50)
	#initialize listeners
	bobWay=waypoint_listener()
	bobInfo=robot_info_listener()
	rospy.Subscriber('goal_pos',waypoints,bobWay.callback)
	rospy.Subscriber('current_robot_info',robot_info,bobInfo.callback)
	pub=rospy.Publisher('cmd_vel',robot_info,queue_size=10)
	bobPubInfo=robot_info()
	
	distance_tolerance=2.1
	angle_tolerance=1
	
	while (not rospy.is_shutdown()) :
		if len(bobWay.x)>0:
			for i in range(len(bobWay.x)):
				bobControl.reset_Iterms()
				while getDistance(bobWay.x[i],bobWay.y[i],bobInfo.x,bobInfo.y)>distance_tolerance or abs(bobWay.theta[i]-bobInfo.theta)>angle_tolerance:
					bobControl.update_current_positions(bobWay.x[i],bobWay.y[i],bobWay.theta[i],bobInfo.x,bobInfo.y,bobInfo.theta)
					vels=bobControl.update_velocities(bobWay.min_velocity[i])
					bobPubInfo.v_x=vels[0]
					bobPubInfo.v_y=vels[1]
					bobPubInfo.omega=vels[2]				
					pub.publish(bobPubInfo)
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
	
	def __init__(self,saturation_linear,saturation_angular,Kc_linear,Ti_linear,Kc_angular,Ti_angular):
		self.saturation_l=float(saturation_linear)
		self.saturation_a=float(saturation_angular)
		
		#if velocity input is saturated
		self.satX=False
		self.satY=False
		self.satT=False
		
		self.Ki_l=float(Kc_linear)/float(Ti_linear)
		self.Kc_l=float(Kc_linear)
		self.Ki_a=float(Kc_angular)/float(Ti_angular)
		self.Kc_a=float(Kc_angular)
		self.Ti_linear=float(Ti_linear)
		self.Ti_angular=float(Ti_angular)
		
		self.setX=0.0
		self.setY=0.0
		self.setTheta=0.0
		
		self.currentX=0.0
		self.currentY=0.0
		self.currentTheta=0.0
		
		self.errorX=0.0
		self.errorY=0.0
		self.errorTheta=0.0
		
		#integrator sums
		self.IX=0.0
		self.IY=0.0
		self.ITheta=0.0
		
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
		if not self.satX:
			self.IX+=self.Ki_l*self.errorX*delta_t
		if not self.satY:
			self.IY+=self.Ki_l*self.errorY*delta_t
		if not self.satT:
			self.ITheta+=self.Ki_a*self.errorTheta*delta_t
		
		#Set last time
		self.last_time=time.time()
	
			
		#Add both contributions
		v_x=v_xP+self.IX
		v_y=v_yP+self.IY
		v_theta=v_thetaP+self.ITheta
		
		#motor saturtaion (sets max speed) 
		#x saturation
		if v_x>self.saturation_l:
			#if motor is saturated, apply windup protection
			self.satX=True
			v_x=self.saturation_l
		elif v_x<-self.saturation_l:
			self.satX=True
			v_x=-self.saturation_l
		else:
			#if saturation does not occur, there is no effect
			self.satX=False
		#y saturation
		if v_y>self.saturation_l:
			self.satY=True
			v_y=self.saturation_l
		elif v_y<-self.saturation_l:
			self.satY=True
			v_y=-self.saturation_l
		else:
			self.satY=False
		#theta saturation
		if v_theta>self.saturation_a:
			self.satT=True
			v_theta=self.saturation_a
		elif v_theta<-self.saturation_a:
			self.satT=True
			v_theta=-self.saturation_a
		else:
			self.satT=False
		
		mag=math.sqrt((v_x**2)+(v_y**2))
		multiplier=minVel/mag
		oldang=math.atan2(v_y,v_x)

		if mag<minVel:
			v_x*=multiplier
			v_y*=multiplier
			print "mag: " + str(mag)
			print "mult: " + str(multiplier)
			print "min_vel: " + str( minVel)
			print "same angle?: " + str(math.atan2(v_y,v_x)==oldang)
			print "current mag: " + str(math.sqrt((v_x**2)+(v_y**2)))
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
