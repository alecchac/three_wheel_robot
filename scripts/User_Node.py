#!/usr/bin/env python
import rospy
from three_wheel_robot.msg import waypoints


def main():
    myWaypoints = waypoints()
    pub = rospy.Publisher('goal_pos',waypoints,queue_size=10)
    rospy.init_node('User',anonymous=True)
    rate=rospy.Rate(1)
    myWaypoints.x=[90,80,10]
    myWaypoints.y=[90,10,90]
    myWaypoints.theta=[3.14,2,5]
    myWaypoints.min_velocity=[3,3,3]

    while not rospy.is_shutdown():
        if len(myWaypoints.x)==len(myWaypoints.y) and len(myWaypoints.x)==len(myWaypoints.theta) and len(myWaypoints.x)==len(myWaypoints.min_velocity):
            pub.publish(myWaypoints)
            print(myWaypoints)
            rate.sleep()
        else:
            rospy.loginfo("ERROR: Arrays are different sizes, will not publish")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
