#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray, GoalID
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback

class RepeatMoveBaseGoal:
    def __init__(self):
        rospy.init_node('repeat_move_base_goal')
        
        self.goal_pub = rospy.Publisher('/tb3_1/move_base_simple/goal', PoseStamped, queue_size=5)
        rospy.Subscriber('/tb3_1/move_base/status', GoalStatusArray, self.goal_status_callback)
        rospy.Subscriber('/tb3_1/move_base/feedback', MoveBaseActionFeedback, self.feedback_callback)
        rospy.Subscriber('/tb3_1/odom',Odometry,self.odom_cb)
        self.cancel_pub = rospy.Publisher('/tb3_1/move_base/cancel', GoalID, queue_size=10)

        self.rate = rospy.Rate(0.5) 
        self.goal_reached = False
        self.xp_odom=0.0
        self.yp_odom=0.0

    def run(self):
        while not rospy.is_shutdown():
            x, y, qz = self.get_goal_from_user()
            #z, qx, qy, ,qw 
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.pose.position.x = x
            goal.target_pose.pose.position.y = y
            # goal.target_pose.pose.position.z = z
            # goal.target_pose.pose.orientation.x = qx
            # goal.target_pose.pose.orientation.y = qy
            goal.target_pose.pose.orientation.z = qz
            # goal.target_pose.pose.orientation.w = qw

            self.goal_pub.publish(goal.target_pose)
            self.goal_reached = False

            
            while not rospy.is_shutdown() and not self.goal_reached:
                print(f'x: {self.xp_odom}     y: {self.yp_odom}')
                self.rate.sleep()
            # if self.goal_pub==True:
            #     self.cancel_pub.publish()
            #     print(f'x: {self.xp_odom}   y: {self.yp_odom}')

    def get_goal_from_user(self):
        x = float(input("Enter x position: "))
        y = float(input("Enter y position: "))
        # z = float(input("Enter z position: "))
        # qx = float(input("Enter x orientation: "))
        # qy = float(input("Enter y orientation: "))
        qz = float(input("Enter z orientation: "))
        # qw = float(input("Enter w orientation: "))

        #z, qx, qy, ,qw
        return x, y, qz

    def goal_status_callback(self, data):
        status_list = data.status_list

        for status in status_list:
            if status.status == 3:  # SUCCEEDED
                self.goal_reached = True

    def feedback_callback(self, data):
        if data.status.status == 4:  # PREEMPTED
            self.goal_reached = True

    def odom_cb(self,data):
        self.xp_odom = data.pose.pose.position.x
        self.yp_odom = data.pose.pose.position.y


if __name__ == '__main__':
    try:
        node = RepeatMoveBaseGoal()
        node.run()
    except rospy.ROSInterruptException:
        pass