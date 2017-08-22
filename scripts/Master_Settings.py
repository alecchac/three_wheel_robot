#! /usr/bin/python

#---------Map Scaling Factor
#     SF = Pixels:Meters
SF = 37.0 / 0.15

#---------Velocity Controller------------
max_linear_speed= .3 * SF #pixels/sec
max_angular_speed = 3 #radians/sec
Kc_linear = .003
Ti_linear = 3 #Ki=Kc/Ti
Kc_angular = 1.5 
Ti_angular = 3
Kd_linear = 0
Kd_angular = 0
distance_tolerance = 20 #Pixels
angle_tolerance = 0.1 #Radians


#---------Wheel Speed Controller----------
saturation_omega = 90
Kc = 1.5
Ti = 1000
Kd = .15

#---------Robot Definitions---------------
d = 0.09 * SF
r = 0.05 * SF

