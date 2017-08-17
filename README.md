# three_wheel_robot
ROS Package
Package to run the three wheel robot on the Raspberry Pi on Ubuntu Mate 16.04
Hardware:
[3] Pololu - 100 1 Metal Gearmotor 20Dx44L mm 6V with Extended Motor Shaft_files 
[3] Pololu - Magnetic Encoder Pair Kit for 20D mm Metal Gearmotors, 20 CPR, 2.7-18V_files
[1] Raspberry Pi 3 Model B
[2] DRV8833 Dual Motor Driver Polulu part 2130
[2] 5V/2.1A Output 5000 mAh Poweradd powerbank
[3] Kornylak TransWheel
[1] 3D printed chassis
[1] aruco marker

Package Includes:
Kalman Filter
Velocity Controller
Wheel Speed Controller
PWM input
Encoder input

Topics:
Pose_hat: contains the current Position and Velocity of the Robot
cmd_vel: desired world velocities from controller
Speeds: PWM values to pi_pwm
encoder_omegas: encoder angular wheel velocities
goal_pos: waypoint inputs



