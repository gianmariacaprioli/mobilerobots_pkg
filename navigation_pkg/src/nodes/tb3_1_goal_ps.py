#!/usr/bin/env python3

import numpy as np
import random
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray, GoalID
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback

class RepeatMoveBaseGoal:
    def __init__(self):
        rospy.init_node('repeat_move_base_goal')
        
        self.goal_pub = rospy.Publisher('/tb3_1/move_base_simple/goal', PoseStamped, queue_size=10)
        rospy.Subscriber('/tb3_1/move_base/status', GoalStatusArray, self.goal_status_callback)
        rospy.Subscriber('/tb3_1/move_base/feedback', MoveBaseActionFeedback, self.feedback_callback)
        rospy.Subscriber('/tb3_1/odom',Odometry,self.odom_cb)
        self.cancel_pub = rospy.Publisher('/tb3_1/move_base/cancel', GoalID, queue_size=10)

        self.rate = rospy.Rate(0.90) 
        self.goal_reached = False
        self.goal_reaching = False
        self.xp_odom=0.0
        self.yp_odom=0.0
        self.i = 0
        self.goal_list=[[3.0,1.0,1.0],[5.0,3.0,1.0],[5.0,5.0,1.0]]

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

            #CAPIRE COME FUNZIONANO I QUATERNIONI

            print(goal)
            self.goal_reached = False
            self.goal_reaching = True
            print("nel while esterno i =" + str(self.i))
            
            while not rospy.is_shutdown() and not self.goal_reached:
                print(f'x: {self.xp_odom}     y: {self.yp_odom}')
                self.goal_reaching=True                
                self.goal_pub.publish(goal.target_pose)
                print("nel while interno i =" + str(self.i))
                self.rate.sleep()


                if (abs(abs(self.xp_odom) - abs(x)) <= 0.5
                    and (abs(abs(self.yp_odom) - abs(y)) <= 0.5)):
                    self.goal_reached = True
                    self.goal_reaching = False
                    self.i = self.i + 1
                    print("AGGIORNO I: " + str(self.i))
                    self.cancel_pub.publish()
                    print("goal reached and status canceled")

        
            # if self.goal_reached == True:
            #     self.cancel_pub.publish()
            #     print("goal reached and status canceled")

                ################################################################
                # HAI FATTO CASINO CON I FLAG REACHING NON COMMUTANO DA F -> T #
                ################################################################

    def get_goal_from_user(self):

        #goal_list = [[3.0,1.0,1.0],[5.0,3.0,1.0],[5.0,5.0,1.0]]

        if(self.i<=2):
            
            x = self.goal_list[self.i][0]
            y = self.goal_list[self.i][1]
            qz = self.goal_list[self.i][2]
            # x,y,qz = goal_list[self.i]
                      
            self.goal_reached = False

        if self.i > 2:
            self.goal_list.append([5,float(random.randint(-8,8)),1])
            # goal_list[self.i][0] = 5
            # goal_list[self.i][1] = float(random.randint(-8,8))
            # goal_list[self.i][2] = 1
            x,y,qz = self.goal_list[self.i]
                
            # goal_list[self.i][0] = 5
            # goal_list[self.i][1] = float(random.randint(-8,8))
            # goal_list[self.i][2] = 1

        # x = float(input("Enter x position: "))
        # y = float(input("Enter y position: "))
        # # z = float(input("Enter z position: "))
        # # qx = float(input("Enter x orientation: "))
        # # qy = float(input("Enter y orientation: "))
        # qz = float(input("Enter z orientation: "))
        # # qw = float(input("Enter w orientation: "))

        #z, qx, qy, ,qw
        return x, y, qz

    def goal_status_callback(self, data):
        status_list = data.status_list

        if self.goal_reaching == False:
            for status in status_list:
                if status.status == 3:  # SUCCEEDED
                    self.goal_reached = True

    def feedback_callback(self, data):

        if self.goal_reaching == False:
            if data.status.status == 4:  # PREEMPTED
                self.goal_reached = True

    def odom_cb(self,data):
        self.xp_odom = round(data.pose.pose.position.x,2)
        self.yp_odom = round(data.pose.pose.position.y,2)


if __name__ == '__main__':
    try:
        node = RepeatMoveBaseGoal()
        node.run()
    except rospy.ROSInterruptException:
        pass