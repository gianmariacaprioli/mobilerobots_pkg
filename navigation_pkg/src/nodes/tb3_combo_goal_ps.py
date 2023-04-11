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
        
        self.goal_pub_tb3_0 = rospy.Publisher('/tb3_0/move_base_simple/goal', PoseStamped, queue_size=10)
        self.cancel_pub_tb3_0= rospy.Publisher('/tb3_0/move_base/cancel', GoalID, queue_size=10)
        rospy.Subscriber('/tb3_0/move_base/status', GoalStatusArray, self.goal_status_callback_tb3_0)
        rospy.Subscriber('/tb3_0/move_base/feedback', MoveBaseActionFeedback, self.feedback_callback_tb3_0)
        rospy.Subscriber('/tb3_0/odom',Odometry,self.odom_cb_tb3_0)


        self.goal_pub_tb3_1 = rospy.Publisher('/tb3_1/move_base_simple/goal', PoseStamped, queue_size=10)
        self.cancel_pub_tb3_1 = rospy.Publisher('/tb3_1/move_base/cancel', GoalID, queue_size=10)
        rospy.Subscriber('/tb3_1/move_base/status', GoalStatusArray, self.goal_status_callback_tb3_1)
        rospy.Subscriber('/tb3_1/move_base/feedback', MoveBaseActionFeedback, self.feedback_callback_tb3_1)
        rospy.Subscriber('/tb3_1/odom',Odometry,self.odom_cb_tb3_1)

        self.rate = rospy.Rate(0.90) 
        self.tb3_0_goal_reached = False
        self.tb3_0_goal_reaching = False
        self.tb3_0_goal_ng = True

        self.tb3_1_goal_reached = False
        self.tb3_1_goal_reaching = False
        self.tb3_1_goal_ng = True


        self.tb3_1_xp_odom=0.0
        self.tb3_1_yp_odom=0.0

        self.tb3_0_xp_odom=0.0
        self.tb3_0_yp_odom=0.0

        self.j = 0
        self.i = 0

        self.tb3_0_goal_list=[[-5.0,1.0,1.0],[-5.0,3.0,1.0],[-5.0,5.0,1.0]]
        self.tb3_1_goal_list=[[5.0,1.0,1.0],[5.0,3.0,1.0],[5.0,5.0,1.0]]
        #self.goal_list = []

    def run(self):
        self.tb3_0_goal_reaching = False
        self.tb3_1_goal_reaching = False

        while not rospy.is_shutdown():

            if self.tb3_0_goal_ng == True:
                x0, y0, qz0= self.get_goal_from_user(self.i)
                tb3_0_goal = MoveBaseGoal()
                tb3_0_goal.target_pose.header.frame_id = "map"
                tb3_0_goal.target_pose.pose.position.x = x0
                tb3_0_goal.target_pose.pose.position.y = y0
                tb3_0_goal.target_pose.pose.orientation.z = qz0
                self.tb3_0_goal_reached = False
                self.tb3_0_goal_reaching = True
                self.tb3_0_goal_ng = False
            
            if self.tb3_1_goal_ng == True:
                x1, y1, qz1= self.get_goal_from_user(self.j)
                tb3_1_goal = MoveBaseGoal()
                tb3_1_goal.target_pose.header.frame_id = "map"
                tb3_1_goal.target_pose.pose.position.x = x1
                tb3_1_goal.target_pose.pose.position.y = y1
                tb3_1_goal.target_pose.pose.orientation.z = qz1
                self.tb3_1_goal_reached = False
                self.tb3_1_goal_reaching = True
                self.tb3_1_goal_ng = False


            print("############## l'indice i =" + str(self.i))
            print("############## l'indice j =" + str(self.j))
            
            if not rospy.is_shutdown() and not self.tb3_0_goal_reached:
                print(tb3_0_goal)
                print(f'x: {self.tb3_0_xp_odom}     y: {self.tb3_0_yp_odom}')
                self.tb3_0_goal_reaching=True                
                self.goal_pub_tb3_0.publish(tb3_0_goal.target_pose)
                print("nel while interno i =" + str(self.i))
                self.rate.sleep()

                if (((x0 - self.tb3_0_xp_odom) < 0.5 and (x0 - self.tb3_0_xp_odom) > -0.5) and
                    ((y0 - self.tb3_0_yp_odom) < 0.5 and (y0 - self.tb3_0_yp_odom) > -0.5)):
                    self.tb3_0_goal_reached = True
                    self.tb3_0_goal_reaching = False
                    self.tb3_0_goal_ng = True
                    self.i = self.i + 1
                    print("AGGIORNO I: " + str(self.i))
                    self.cancel_pub_tb3_0.publish()
                    print("goal reached and status canceled")
                    self.rate.sleep()

            if not rospy.is_shutdown() and not self.tb3_1_goal_reached:
                print(tb3_1_goal)
                print(f'x: {self.tb3_1_xp_odom}     y: {self.tb3_1_yp_odom}')
                self.tb3_1_goal_reaching=True                
                self.goal_pub_tb3_1.publish(tb3_1_goal.target_pose)
                print("nel while interno i =" + str(self.i))
                self.rate.sleep()


                if (((x1 - self.tb3_1_xp_odom) < 0.5 and (x1 - self.tb3_1_xp_odom) > -0.5) and
                    ((y1 - self.tb3_1_yp_odom) < 0.5 and (y1 - self.tb3_1_yp_odom) > -0.5)):
                    self.tb3_1_goal_reached = True
                    self.tb3_1_goal_reaching = False
                    self.tb3_1_goal_ng = True
                    self.i = self.i + 1
                    print("AGGIORNO I: " + str(self.i))
                    self.cancel_pub_tb3_1.publish()
                    print("goal reached and status canceled")
                    self.rate.sleep()

        
    def get_goal_from_user(self,index):

        #goal_list = [[3.0,1.0,1.0],[5.0,3.0,1.0],[5.0,5.0,1.0]]
        if(index<=2 and self.tb3_0_goal_ng == True):
            x0 = self.tb3_0_goal_list[self.i][0]
            y0 = self.tb3_0_goal_list[self.i][1]
            qz0 =self.tb3_0_goal_list[self.i][2]
            self.goal_reached = False
            self.i = self.i + 1
            return x0,y0,qz0
        
        elif(index>2 and self.tb3_0_goal_ng == True):
            self.tb3_0_goal_list.append([-5,float(random.randint(-8,8)),1])
            x = self.tb3_0_goal_list[self.i][0]
            y = self.tb3_0_goal_list[self.i][1]
            qz =self.tb3_0_goal_list[self.i][2]
            self.goal_reached = False
            self.i = self.i + 1
            return x,y,qz
        
        elif(index<=2 and self.tb3_1_goal_ng == True):
            x1 = self.tb3_1_goal_list[self.j][0]
            y1 = self.tb3_1_goal_list[self.j][1]
            qz1 =self.tb3_1_goal_list[self.j][2]
            self.tb3_1_goal_reached = False
            self.j = self.j + 1
            return x1,y1,qz1
        
        elif(index>2 and self.tb3_1_goal_ng == True):
            self.tb3_1_goal_list.append([-5,float(random.randint(-8,8)),1])
            x1 = self.tb3_1_goal_list[self.j][0]
            y1 = self.tb3_1_goal_list[self.j][1]
            qz1 =self.tb3_1_goal_list[self.j][2]
            self.tb3_1_goal_reached = False
            self.j = self.j + 1
            return x1,y1,qz1

        
    
    def goal_status_callback_tb3_0(self, data):
        status_list = data.status_list

        if self.tb3_0_goal_reaching == False:
            for status in status_list:
                if status.status == 3:  # SUCCEEDED
                    self.tb3_0_goal_reached = True

    def feedback_callback_tb3_0(self, data):

        if self.tb3_0_goal_reaching == False:
            if data.status.status == 4:  # PREEMPTED
                self.tb3_0_goal_reached = True

    def odom_cb_tb3_0(self,data):
        self.tb3_0_xp_odom = round(data.pose.pose.position.x,2)
        self.tb3_0_yp_odom = round(data.pose.pose.position.y,2)  

    def goal_status_callback_tb3_1(self, data):
        status_list = data.status_list

        if self.tb3_1_goal_reaching == False:
            for status in status_list:
                if status.status == 3:  # SUCCEEDED
                    self.tb3_1_goal_reached = True

    def feedback_callback_tb3_1(self, data):

        if self.tb3_1_goal_reaching == False:
            if data.status.status == 4:  # PREEMPTED
                self.tb3_1_goal_reached = True

    def odom_cb_tb3_1(self,data):
        self.tb3_1_xp_odom = round(data.pose.pose.position.x,2)
        self.tb3_1_yp_odom = round(data.pose.pose.position.y,2)





if __name__ == '__main__':
    try:
        node = RepeatMoveBaseGoal()
        node.run()
    except rospy.ROSInterruptException:
        pass