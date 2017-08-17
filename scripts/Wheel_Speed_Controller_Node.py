#!/usr/bin/env python
import time
import math
import rospy
from numpy import array
from numpy import dot
from math import *
from three_wheel_robot.msg import Speeds
from three_wheel_robot.msg import robot_info
from Master_Settings import saturation_omega,Kc,Ti,Kd,d,r


def main():
    #Print Only One Wait Message
    msgct=0
    #init Node
    rospy.init_node('Wheel_Speed_Controller',anonymous=True)
    #init controller
    wheel_control = Wheel_Speed_Controller(saturation_omega,Kc,Ti,Kd)
    #initializes the  listeners and publishers
    vels = robot_info_listener()
    robot_listener = robot_info_listener()
    encoder_listen=robot_info_listener()
    pwm_msg = Speeds()
    #recieve wheel speeds from encoder
    rospy.Subscriber('encoder_omegas',robot_info,encoder_listen.callback)
    #recieve velocities from the controller
    rospy.Subscriber('cmd_vel',robot_info,vels.callback)
    #recive angles from the KF
    rospy.Subscriber('Pose_hat',robot_info,robot_listener.callback)
    pub = rospy.Publisher('speeds',Speeds,queue_size=1)
    #rate of loop
    rate=rospy.Rate(30)#hz

    while not rospy.is_shutdown():
        #checks if subscriber has recieved a message (max linear velocity must not be zero)
        if vels.max_vel_linear !=0:
            set_vels = rotate_and_convert_to_wheel_speeds(vels.v_x,vels.v_y,vels.omega,robot_listener.theta)
            wheel_control.update_current_positions(set_vels[0],set_vels[1],set_vels[2],encoder_listen.v_x,encoder_listen.v_y,encoder_listen.omega)
            wheel_output = wheel_control.update_velocities()
            pwm_msg.s1 = int(wheel_output[0])
            pwm_msg.s2 = int(wheel_output[1])
            pwm_msg.s3 = int(wheel_output[2])
            pub.publish(pwm_msg)
            #print pwm_msg
            rate.sleep()
        elif msgct == 0:
            rospy.loginfo('Waiting for Subscriber')
            msgct+=1
    rospy.spin()

def rotate_and_convert_to_wheel_speeds(v_x,v_y,omega,theta):
    #initialize rotation matrix
    rotation = array([[cos(theta), sin(theta), 0],[-sin(theta),cos(theta),0],[0,0,1]])
    #robot frame velocities to robot wheel velocities array
    wheel=array([[-sin(pi/3),cos(pi/3),d],[0,-1,d],[sin(pi/3),cos(pi/3),d]])
    #do the transformations using matrix multilplication (numpy)
    world_velocities = array([[v_x],[v_y],[omega]])
    robot_velocities = dot(rotation,world_velocities)
    wheel_velocities = dot(wheel,robot_velocities) #in Pixels/sec
    wheel_velocities = wheel_velocities / r #Omega (V=r*omega) ===> omega = V/r
    return wheel_velocities

class encoder_listener(object):
    """ Encoder listener"""
    def __init__(self):
        self.s1 = 0.0
        self.s2 = 0.0
        self.s3 = 0.0

    def callback(self,info):
        self.s1 = info.s1
        self.s2 = info.s2
        self.s3 = info.s3

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

class Wheel_Speed_Controller(object):
    """PI Controller

    Controls Wheel Speed in Rad/s

    Ki=Kc/Ti

    Dependencies:time
    """

    def __init__(self,saturation,Kc,Ti,Kd):
        self.saturation = float(saturation)

        #if velocity input is saturated
        self.sat1 = False
        self.sat2 = False
        self.sat3 = False

        self.Ki=float(Kc)/float(Ti)
        self.Kc=float(Kc)
        self.Ti=float(Ti)
        self.Kd=float(Kd)

        self.set1=0.0
        self.set2=0.0
        self.set3=0.0

        self.current1=0.0
        self.current2=0.0
        self.current3=0.0

        self.v_1 = 0.0
        self.v_2 = 0.0
        self.v_3 = 0.0

        self.error1=0.0
        self.error2=0.0
        self.error3=0.0
        self.last_error_1=0.0
        self.last_error_2=0.0
        self.last_error_3=0.0

        #integrator sums
        self.I1=0.0
        self.I2=0.0
        self.I3=0.0

        #Derivitive terms
        self.D1=0.0
        self.D2=0.0
        self.D3=0.0

        self.last_time = time.time()

    def update_velocities(self):
        #update errors
        self.error1 = self.set1-self.current1
        self.error2 = self.set2-self.current2
        self.error3 = self.set3-self.current3

        print "error1: " + str(self.error1)

        #Proportional Contribution
        v_1P = self.error1*self.Kc
        v_2P  =self.error2*self.Kc
        v_3P = self.error3*self.Kc

        #calculate time from last function run
        delta_t=time.time()-self.last_time
        #resets delta_t if function has not been run for a while
        if delta_t>2:
            delta_t=0

        #Integral Contribution 
        #Sums up the error term with the delta t to remove steady state error
        #incorporates windup protection when motor is saturated
        if not self.sat1:
            self.I1 += self.Ki*self.error1*delta_t
        if not self.sat2:
            self.I2 += self.Ki*self.error2*delta_t
        if not self.sat3:
            self.I3 += self.Ki*self.error3*delta_t

        #derivitive contribution
        self.D1=self.Kd*((self.error1-self.last_error_1)/delta_t)
        self.D2=self.Kd*((self.error2-self.last_error_2)/delta_t)
        self.D3=self.Kd*((self.error3-self.last_error_3)/delta_t)

        #set last error
        self.last_error_1=self.error1
        self.last_error_2=self.error2
        self.last_error_3=self.error3


        #Set last time
        self.last_time=time.time()

        #Add Three contributions
        self.v_1 += v_1P+self.I1+self.D1
        self.v_2 += v_2P+self.I2+self.D2
        self.v_3 += v_3P+self.I3+self.D3

        #motor saturtaion (sets max speed)
        #wheel 1
        if self.v_1 > self.saturation:
            self.v_1 = self.saturation
            self.sat1 = True
        elif self.v_1 < -self.saturation:
            self.v_1 = -self.saturation
            self.sat1 = True
        else:
            self.sat1 = False

        #wheel 2
        if self.v_2 > self.saturation:
            self.v_2 = self.saturation
            self.sat2 = True
        elif self.v_2 < -self.saturation:
            self.v_2 = -self.saturation
            self.sat2 = True
        else:
            self.sat2 = False

        #wheel 3
        if self.v_3 > self.saturation:
            self.v_3 = self.saturation
            self.sat3 = True
        elif self.v_3 < -self.saturation:
            self.v_3 = -self.saturation
            self.sat3 = True
        else:
            self.sat3 = False

        #return calculated velocities
        return [self.v_1,self.v_2,self.v_3]


    def update_current_positions(self,set1,set2,set3,current1,current2,current3):
        self.current1=current1
        self.current2=current2
        self.current3=current3
        self.set1=set1
        self.set2=set2
        self.set3=set3
        print "set 1: " + str(set1)
        print "current 1: " + str(current1)

    def reset_Iterms(self):
        self.I1=0.0
        self.I2=0.0
        self.I3=0.0


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass