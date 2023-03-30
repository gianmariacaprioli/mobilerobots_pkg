#!/usr/bin/env python3

from roslib import *
import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from random import sample
from math import pow, sqrt
import sys
import signal


def test_point_publisher():

    pub = rospy.Publisher('/tb3_1/move_base_simple/goal', PoseStamped, queue_size=10)

    rospy.init_node('test_point_publisher', anonymous=True)

    target=PoseStamped()
    target.header.frame_id="map"
    target.header.stamp = rospy.Time.now()
    #target.header.seq = 10

    target.pose.position.x = 5.0
    target.pose.position.y = 2.0
    target.pose.orientation.w = -1.0
    target.pose.orientation.z = -1.0  
    r=rospy.Rate(0.5)
    rospy.loginfo(target)
    # pub.publish(target)

    while not rospy.is_shutdown():
        pub.publish(target)        
        r.sleep()


    msg="uscita dal while"

    return print(str(msg))



if __name__ == '__main__':
    try:
        test_point_publisher()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")