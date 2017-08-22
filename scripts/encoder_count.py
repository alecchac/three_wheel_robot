#!/usr/bin/env python

import rospy
from three_wheel_robot.msg import robot_info
from three_wheel_robot.msg import Espeeds
import RPi.GPIO as GPIO
import math
import time

def main():
    #init Node
    rospy.init_node('encoder_counter',anonymous=True)
    #set mode to bcm
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    #initialize encoder counts per revolution of the input shaft
    CPR_encoder = 20.0
    #initialize gear ratio of the motor output shaft (100:1)
    gear_ratio = 100.0
    #initialize encoder listener for each wheel
    wheel_one = encoder(CPR_encoder,gear_ratio,19,26)
    wheel_two = encoder(CPR_encoder,gear_ratio,7,12)
    wheel_three = encoder(CPR_encoder,gear_ratio,20,16)
    #setup pins for each wheel
    wheel_one.setup_pins()
    wheel_two.setup_pins()
    wheel_three.setup_pins()
    #create message used to send data 
    #TODO: replace with its own message ----- PROBLEM ------- cannot install new custom message on pi
    speed_msg=Espeeds()
    #init publisher 
    pub = rospy.Publisher('encoder_omegas',Espeeds,queue_size=1)
    #rate of loop
    rate = rospy.Rate(30)#hz

    while not rospy.is_shutdown():
        #continuously get encoder omegas (rad/s)
        speed_msg.e_s1=wheel_one.get_omega()
        speed_msg.e_s2=wheel_two.get_omega()
        speed_msg.e_s3=wheel_three.get_omega()
        #publish to topic
        pub.publish(speed_msg)
        print speed_msg
        #sleep for specified time (need enough time for encoder counts to build up or speeds will be inaccurate)
        rate.sleep()

    #once done cleanup pins
    print "shutdown"
    wheel_one.cleanup()
    wheel_two.cleanup()
    wheel_three.cleanup()

class encoder(object):
    """
    Obtains omega (rad/s) from wheel encoder
    Detects the rising and falling edges to get a counts/sec and converts to Rad/s
    http://raspi.tv/2014/rpi-gpio-update-and-detecting-both-rising-and-falling-edges
    https://sourceforge.net/p/raspberry-gpio-python/wiki/Inputs/
    """
    def __init__(self,CPR_encoder,gear_ratio,gpio_channel_A,gpio_channel_B):
        self.CPR_encoder = float(CPR_encoder) #counts per revolution
        self.gear_ratio = float(gear_ratio) #gear ratio of motor
        self.CPR_output = float(CPR_encoder * gear_ratio) #counts per revolution of output shaft
        self.channel_A=gpio_channel_A #Pin number of channel A
        self.channel_B=gpio_channel_B #Pin number of channel B
        self.count = 0.0 #counts the number of transisitons
        self.last_count = 0.0
        self.state_A = 0 #records the state of encoder (0 or 1) 
        self.state_B = 0
        self.direction = True #True = CCW, False = CW
        self.last_time = time.time()

    def setup_pins(self):
        #initialize pins as input
        GPIO.setup( self.channel_A, GPIO.IN)
        GPIO.setup( self.channel_B, GPIO.IN)
        #initialize to detect events (both rising and falling edge) with threaded callback
        GPIO.add_event_detect( self.channel_A, GPIO.BOTH, callback=self.callback_A)
        GPIO.add_event_detect( self.channel_B, GPIO.BOTH, callback=self.callback_B)

    def callback_A(self,channel):
        #get the state of both inputs when encounter rising or falling edge
        self.state_A=GPIO.input(self.channel_A)
        self.state_B=GPIO.input(self.channel_B)
        #determine direction
        if self.state_A == self.state_B and self.state_A == 1:
            self.direction = True
        elif self.state_A != self.state_B and self.state_A == 1:
            self.direction = False
        #add to count every edge
        self.count += 1

    def callback_B(self,channel):
        self.count += 1

    def get_omega(self):
        #convert counts/s into rad/s
        counts = self.count-self.last_count
        delta_t = time.time()-self.last_time
        omega = ((counts / self.CPR_output) * 2 * math.pi)/delta_t #rad/s

        #set last time and count
        self.last_count = self.count
        self.last_time = time.time()

        #determine direction
        if self.direction == False and omega > 1:
            omega *= -1


        return omega

    def cleanup(self):
        GPIO.cleanup(self.channel_A)
        GPIO.cleanup(self.channel_B)


if __name__ == '__main__':
    main()
