#!/usr/bin/env python3
import rospy
import src.nodes.tb3_1_odom_sub as tb3_1_odom_sub

if __name__ == '__main__':
    res= tb3_1_odom_sub.OdomListener()
    x= res.get_position
    print(res.x)

