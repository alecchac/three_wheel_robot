#!/usr/bin/env python
import rospy
from three_wheel_robot.msg import waypoints


def main():
    myWaypoints = waypoints()
    pub = rospy.Publisher('goal_pos',waypoints,queue_size=1)
    rospy.init_node('User',anonymous=True)
    rate=rospy.Rate(1)
    myWaypoints.x=[1.25,2.5,.75,.3,0,.75,1.75]
    myWaypoints.y=[-.35,.65,-.35,.65,.65,-.35,.8]
    myWaypoints.theta=[0,0,0,0,0,0,0]#ignore for now
    myWaypoints.min_velocity=[.05,.05,.05,.05,.05,.05,.05] # Meters Per Second

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
