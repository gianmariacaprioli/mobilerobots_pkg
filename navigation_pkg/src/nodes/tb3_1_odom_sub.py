#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry

class OdomListener:
    def __init__(self):
        self.x = None
        self.y = None
        self.sub = rospy.Subscriber("/tb3_1/odom", Odometry, self.callback)

    def callback(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y

    def get_position(self):
        return self.x, self.y
# import rospy
# from nav_msgs.msg import Odometry

# def callback(data):
#     global x, y
#     x = data.pose.pose.position.x
#     y = data.pose.pose.position.y

# def main():
#     global x, y
#     x = None
#     y = None
#     rospy.init_node('odom_subscriber', anonymous=True)
#     rospy.Subscriber('/tb3_1/odom', Odometry, callback)
#     rate = rospy.Rate(10) # 10 Hz
#     while not rospy.is_shutdown():
#         if x is not None and y is not None:
#             rospy.loginfo("x: {}, y: {}".format(x, y))
#             return x, y
#         rate.sleep()

# if __name__ == '__main__':
#     try:
#         main()
#     except rospy.ROSInterruptException:
#         pass