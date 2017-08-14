#!/usr/bin/env python

import rospy
from three_wheel_robot.msg import robot_info
import RPi.GPIO as GPIO
import math
import time

def main():
    rospy.init_node('encoder_counter',anonymous=True)
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    CPR_encoder = 20.0
    gear_ratio = 100.0
    wheel_one = encoder(CPR_encoder,gear_ratio,19,26)
    wheel_two = encoder(CPR_encoder,gear_ratio,7,12)
    wheel_three = encoder(CPR_encoder,gear_ratio,20,16)
    wheel_one.setup_pins()
    wheel_two.setup_pins()
    wheel_three.setup_pins()
    speed_msg=robot_info()
    pub = rospy.Publisher('encoder_omegas',robot_info,queue_size=1)
    #rate of loop
    rate = rospy.Rate(10)#hz
    while not rospy.is_shutdown():
        speed_msg.v_x=wheel_one.get_omega()
        speed_msg.v_y=wheel_two.get_omega()
        speed_msg.omega=wheel_three.get_omega()
        rate.sleep()
        pub.publish(speed_msg)
    #once done cleanup pins
    print "shutdown"
    wheel_one.cleanup()
    wheel_two.cleanup()
    wheel_three.cleanup()

class encoder(object):
    """
    counts wheel angular velocity from encoders
    (rad/s)
    """
    def __init__(self,CPR_encoder,gear_ratio,gpio_channel_A,gpio_channel_B):
        self.CPR_encoder = float(CPR_encoder) #counts per revolution
        self.gear_ratio = float(gear_ratio)
        self.CPR_output = float(CPR_encoder * gear_ratio) #counts per revolution of output shaft
        self.channel_A=gpio_channel_A
        self.channel_B=gpio_channel_B
        self.count=0.0
        self.last_count = 0.0
        self.state_A=0
        self.state_B=0
        self.direction = True #True = CCW, False = CW
        self.last_time = time.time()

    def setup_pins(self):
        GPIO.setup( self.channel_A, GPIO.IN)
        GPIO.setup( self.channel_B, GPIO.IN)
        GPIO.add_event_detect( self.channel_A, GPIO.BOTH, callback=self.callback_A)
        GPIO.add_event_detect( self.channel_B, GPIO.BOTH, callback=self.callback_B)

    def callback_A(self,channel):
        self.state_A=GPIO.input(self.channel_A)
        self.state_B=GPIO.input(self.channel_B)
        if self.state_A == self.state_B and self.state_A == 1:
            self.direction = True
        elif self.state_A != self.state_B and self.state_A == 1:
            self.direction = False
        self.count += 1

    def callback_B(self,channel):
        #self.state_B=GPIO.input(self.channel_B)
        self.count += 1

    def get_omega(self):
        #convert counts into rad/s
        counts = self.count-self.last_count
        delta_t = time.time()-self.last_time
        omega = ((counts / self.CPR_output) * 2 * math.pi)/delta_t #rad/s

        #set last time and count
        self.last_count = self.count
        self.last_time = time.time()

        #determine direction
        if self.direction == False and omega > 1:
            omega *= -1

	print self.direction

        return omega

    def cleanup(self):
        GPIO.cleanup(self.channel_A)
        GPIO.cleanup(self.channel_B)


if __name__ == '__main__':
    main()
