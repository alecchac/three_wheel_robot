#!/usr/bin/env python
import rospy
from three_wheel_robot.msg import waypoints

#TODO: Check to see if all lists have same lengths

def main():
    myWaypoints = waypoints()
    pub = rospy.Publisher('goal_pos',waypoints,queue_size=10)
    rospy.init_node('User',anonymous=True)
    rate=rospy.Rate(1)
    myWaypoints.x=[90,80,10]
    myWaypoints.y=[90,10,90]
    myWaypoints.theta=[3.14,2,5]
    myWaypoints.min_velocity=[2,2,2]

    while not rospy.is_shutdown():
        pub.publish(myWaypoints)
        print(myWaypoints.theta)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
