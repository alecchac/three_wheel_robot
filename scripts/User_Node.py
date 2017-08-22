#!/usr/bin/env python
import rospy
from three_wheel_robot.msg import waypoints


def main():
    myWaypoints = waypoints()
    pub = rospy.Publisher('goal_pos',waypoints,queue_size=1)
    rospy.init_node('User',anonymous=True)
    rate=rospy.Rate(1)
    myWaypoints.x=[500,300,200,300,100]
    myWaypoints.y=[300,60,60,150,300]
    myWaypoints.theta=[-2,2,-2,0,1.57]
    myWaypoints.min_velocity=[3,3,3,3,1] # Pixels Per Second

    while not rospy.is_shutdown():
        #ensures the arrays are the same size
        if len(myWaypoints.x)==len(myWaypoints.y) and len(myWaypoints.x)==len(myWaypoints.theta) and len(myWaypoints.x)==len(myWaypoints.min_velocity):
            pub.publish(myWaypoints)
            print(myWaypoints)
            rate.sleep()
        else:
            rospy.loginfo("ERROR: Arrays are different sizes, will not publish")
            break

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
