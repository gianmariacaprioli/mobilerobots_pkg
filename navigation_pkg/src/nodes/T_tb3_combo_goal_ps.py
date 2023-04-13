#!/usr/bin/env python3

import numpy as np
import centAct
import random
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray, GoalID
from move_base_msgs.msg import MoveBaseGoal, MoveBaseActionFeedback

class RepeatMoveBaseGoal:
    def __init__(self):
        rospy.init_node('repeat_move_base_goal')
        self.rate = rospy.Rate(0.90) 
        self.tb3_0_goal_reached = False
        self.tb3_0_goal_reaching = False
        self.tb3_0_goal_ng = True

        self.tb3_1_goal_reached = False
        self.tb3_1_goal_reaching = False
        self.tb3_1_goal_ng = True
        
        
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


        self.tb3_1_xp_odom=0.0
        self.tb3_1_yp_odom=0.0

        self.tb3_0_xp_odom=0.0
        self.tb3_0_yp_odom=0.0

        self.j = 0
        self.i = 0
        
        self.Lambda = 50 # concentration of pheromones carried by each AGV
        self.Q_Lambda = 5 # threshold of congestion
        self.T_s = 0.25 # sample time in [s] of the algorithm
        self.D_tol = 0.5 # tolerance distance in [m]
        self.N_nodes = 36

        self.Nodes_qt = np.array([ #contiene le coordinate cartesiane
        [9.0,   9.0,   0.7, 0.7], #0
        [9.0,   5.0,   0.7, 0.7],
        [9.0,   1.0,   0.7, 0.7],
        [9.0,  -5.0,   0.7, 0.7],
        [9.0,  -8.0,   0.7, 0.7],
        [9.0, -11.0,   0.7, 0.7], #5 hub
        [5.0,   9.0,   0.7, 0.7],
        [5.0,   5.0,   0.7, 0.7],
        [5.0,   1.0,   0.7, 0.7],
        [5.0,  -5.0,   1.0, 0.0],
        [5.0,  -8.0,   0.7, 0.7],
        [5.0, -11.0,   0.7, 0.7], #11 hub   
        [1.3,  10.0,   0.7, 0.7],
        [1.0,   5.0,   0.7, 0.7],
        [1.0,   1.0,   0.7, 0.7],
        [1.0,  -5.0,   0.7, 0.7],
        [1.0,  -8.0,   0.7, 0.7],
        [0.7, -11.0,   0.7, 0.7], #17 hub
        [-1.0, 12.0,  -0.7, 0.7], #18 hub
        [-1.0,  9.0,  -0.7, 0.7],
        [-1.0,  5.0,  -0.7, 0.7],
        [-1.0,  1.0,  -0.7, 0.7],
        [-1.0, -5.0,  -0.7, 0.7],
        [-1.3, -8.8,   0.0, 1.0],
        [-5.0, 12.0,  -0.7, 0.7], #24 hub
        [-5.0,  9.0,  -0.7, 0.7],
        [-5.0,  5.0,  -0.7, 0.7],
        [-5.0,  1.0,  -0.7, 0.7],
        [-5.0, -5.0,  -0.7, 0.7],
        [-5.0, -8.0,  -0.7, 0.7],
        [-9.0, 12.0,  -0.7, 0.7], #30 hub
        [-9.0,  9.0,  -0.7, 0.7],
        [-9.0,  5.0,  -0.7, 0.7],
        [-9.0,  1.0,  -0.7, 0.7],
        [-9.0, -5.0,  -0.7, 0.7],
        [-9.0, -8.0,   0.0, 1.0], #35
        ])

        self.A_env = np.array([
        #0  1  2  3  4  5  6  7  8  9  10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35
        [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], #0
        [1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], #1
        [0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], #2
        #0  1  2  3  4  5  6  7  8  9  10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35
        [0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], #3
        [0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], #4
        [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], #5
        ############################################################################################################
        #0  1  2  3  4  5  6  7  8  9  10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], #6
        [0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], #7
        [0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], #8
        #0  1  2  3  4  5  6  7  8  9  10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35
        [0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], #9
        [0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], #10 
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], #11
        ############################################################################################################
        #0  1  2  3  4  5  6  7  8  9  10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], #12
        [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], #13
        [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], #14
        #0  1  2  3  4  5  6  7  8  9  10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], #15
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], #16
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], #17
        ############################################################################################################
        #0  1  2  3  4  5  6  7  8  9  10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], #18
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], #19
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0], #20
        #0  1  2  3  4  5  6  7  8  9  10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0], #21
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0], #22
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], #23
        ############################################################################################################
        #0  1  2  3  4  5  6  7  8  9  10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], #24
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0], #25
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0], #26
        #0  1  2  3  4  5  6  7  8  9  10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0], #27
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0], #28
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], #29
        ############################################################################################################
        #0  1  2  3  4  5  6  7  8  9  10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0], #30
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0], #31
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0], #32
        #0  1  2  3  4  5  6  7  8  9  10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0], #33
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1], #34
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0], #35
        ])


        self.K = 2 # numero di robot della flotta 

        self.task = np.array([ 
        17, 
        18, 
        ])

        self.Pos_corr1 =np.array([
        [self.tb3_0_xp_odom, self.tb3_0_yp_odom],
        [self.tb3_1_xp_odom, self.tb3_1_yp_odom], 
        ])

        self.nextNode = np.array([
        21, # -1 1
        13, # 1 1
        ])



    def run(self):

        while not rospy.is_shutdown():


            # if self.tb3_0_goal_ng == True or self.tb3_1_goal_ng == True:
                
            #     nextNode = self.get_goal_from_CA()

            #     tb3_0_goal = MoveBaseGoal()
            #     tb3_0_goal.target_pose.header.frame_id = "map"
                                                        
            #     tb3_0_goal.target_pose.pose.position.x = self.Nodes_qt[nextNode[0]][0]
            #     tb3_0_goal.target_pose.pose.position.y = self.Nodes_qt[nextNode[0]][1]
            #     tb3_0_goal.target_pose.pose.orientation.z = self.Nodes_qt[nextNode[0]][2]
            #     tb3_0_goal.target_pose.pose.orientation.w = self.Nodes_qt[nextNode[0]][3]

            #     self.tb3_0_goal_reached = False
            #     self.tb3_0_goal_reaching = True
            #     self.tb3_0_goal_ng = False

            #     tb3_1_goal = MoveBaseGoal()
            #     tb3_1_goal.target_pose.header.frame_id = "map"

            #     tb3_1_goal.target_pose.pose.position.x = self.Nodes_qt[nextNode[1]][0]
            #     tb3_1_goal.target_pose.pose.position.y = self.Nodes_qt[nextNode[1]][1]
            #     tb3_1_goal.target_pose.pose.orientation.z = self.Nodes_qt[nextNode[1]][2]
            #     tb3_1_goal.target_pose.pose.orientation.w = self.Nodes_qt[nextNode[1]][3]

            #     self.tb3_1_goal_reached = False
            #     self.tb3_1_goal_reaching = True
            #     self.tb3_1_goal_ng = False

            #     self.rate.sleep()
            nextNode = self.get_goal_from_CA()

            if self.tb3_0_goal_ng == True:
                
                tb3_0_goal = MoveBaseGoal()
                tb3_0_goal.target_pose.header.frame_id = "map"
                                                        
                tb3_0_goal.target_pose.pose.position.x = self.Nodes_qt[nextNode[0]][0]
                tb3_0_goal.target_pose.pose.position.y = self.Nodes_qt[nextNode[0]][1]
                tb3_0_goal.target_pose.pose.orientation.z = self.Nodes_qt[nextNode[0]][2]
                tb3_0_goal.target_pose.pose.orientation.w = self.Nodes_qt[nextNode[0]][3]

                self.tb3_0_goal_reached = False
                self.tb3_0_goal_reaching = True
                self.tb3_0_goal_ng = False

            if self.tb3_1_goal_ng == True:
                tb3_1_goal = MoveBaseGoal()
                tb3_1_goal.target_pose.header.frame_id = "map"

                tb3_1_goal.target_pose.pose.position.x = self.Nodes_qt[nextNode[1]][0]
                tb3_1_goal.target_pose.pose.position.y = self.Nodes_qt[nextNode[1]][1]
                tb3_1_goal.target_pose.pose.orientation.z = self.Nodes_qt[nextNode[1]][2]
                tb3_1_goal.target_pose.pose.orientation.w = self.Nodes_qt[nextNode[1]][3]
                
                self.tb3_1_goal_reached = False
                self.tb3_1_goal_reaching = True
                self.tb3_1_goal_ng = False

                self.rate.sleep()
            
            
            if not rospy.is_shutdown() and not self.tb3_0_goal_reached:
                print(tb3_0_goal)
                print(f'x0: {self.tb3_0_xp_odom}     y0: {self.tb3_0_yp_odom}')
                self.tb3_0_goal_reaching=True
                self.tb3_0_goal_ng=False                
                self.goal_pub_tb3_0.publish(tb3_0_goal.target_pose)
                self.rate.sleep()

                # verifica del raggiungimento del goal con le tolleranze sulle x,y
                if (((self.Nodes_qt[self.nextNode[0]][0] - self.tb3_0_xp_odom) < 0.5 and
                      (self.Nodes_qt[self.nextNode[0]][0] - self.tb3_0_xp_odom) > -0.5) and
                    ((self.Nodes_qt[self.nextNode[0]][1] - self.tb3_0_yp_odom) < 0.5 
                     and (self.Nodes_qt[self.nextNode[0]][1]- self.tb3_0_yp_odom) > -0.5)):

                    # verifica del raggiungimento del task finale
                    if (((self.Nodes_qt[self.task][0][0] - self.tb3_0_xp_odom) < 0.5 and
                      (self.Nodes_qt[self.task][0][0] - self.tb3_0_xp_odom) > -0.5) and
                    ((self.Nodes_qt[self.task][0][1] - self.tb3_0_yp_odom) < 0.5 
                     and (self.Nodes_qt[self.task][0][1]- self.tb3_0_yp_odom) > -0.5)):

                    # verifica del raggiungimento del task finale
                        if self.tb3_0_xp_odom <= 0:
                            self.task[0]=float(random.choice([5,11,17]))
                            print("il task per il robot 0 diventa: " + str(self.task[0]))
                        elif self.tb3_0_xp_odom >=0:
                            self.task[0]=float(random.choice([18,24,30]))
                            print("il task per il robot 0 diventa: " + str(self.task[0]))

                    
                    
                    self.tb3_0_goal_reached = True
                    self.tb3_0_goal_reaching = False
                    self.tb3_0_goal_ng = True
                    self.cancel_pub_tb3_0.publish()
                    print("goal reached and status canceled")
                    self.rate.sleep()

            if not rospy.is_shutdown() and not self.tb3_1_goal_reached:
                print(tb3_1_goal)
                print(f'x1: {self.tb3_1_xp_odom}     y1: {self.tb3_1_yp_odom}')
                self.tb3_1_goal_reaching=True
                self.tb3_1_goal_ng=False                 
                self.goal_pub_tb3_1.publish(tb3_1_goal.target_pose)
                self.rate.sleep()

                # verifica del raggiungimento del goal con le tolleranze sulle x,y
                if (((self.Nodes_qt[self.nextNode[1]][0] - self.tb3_1_xp_odom) < 0.5 and
                      (self.Nodes_qt[self.nextNode[1]][0] - self.tb3_1_xp_odom) > -0.5) and
                    ((self.Nodes_qt[self.nextNode[1]][1]- self.tb3_1_yp_odom) < 0.5 
                     and (self.Nodes_qt[self.nextNode[1]][1] - self.tb3_1_yp_odom) > -0.5)):
                    
                    # verifica del raggiungimento del task finale
                    if (((self.Nodes_qt[self.task][1][0] - self.tb3_1_xp_odom) < 0.5 and
                      (self.Nodes_qt[self.task][1][0] - self.tb3_1_xp_odom) > -0.5) and
                    ((self.Nodes_qt[self.task][1][1]- self.tb3_1_yp_odom) < 0.5 
                     and (self.Nodes_qt[self.task][1][1] - self.tb3_1_yp_odom) > -0.5)):
                        
                        # va migliorata poiche i robot tendono a loopare sugli obiettivi, 
                        # può essere che sia dovuto alla mappa di adiacenza
                        # oppure al guess degli obiettivi, quindi devi riguardare le condizioni  
                        
                        # assegnazione di un nuovo task finale opposto alla Pos_cor del robot
                        if self.tb3_1_xp_odom<=0:
                            self.task[1]=float(random.choice([5,11,17]))
                            print("il task per il robot 1 diventa: " + str(self.task[1]))
                        elif self.tb3_1_xp_odom >=0:
                            self.task[1]=float(random.choice([18,24,30]))
                            print("il task per il robot 1 diventa: " + str(self.task[1]))

                        
                    
                    self.tb3_1_goal_reached = True
                    self.tb3_1_goal_reaching = False
                    self.tb3_1_goal_ng = True
                    self.cancel_pub_tb3_1.publish()
                    print("goal reached and status canceled")
                    self.rate.sleep()

            ## da qui aggiungere if per la richiesta di nuovi task FINALI
    def get_goal_from_CA(self):
        nextNode = centAct.nextNode_comp(self.A_env,self.Pos_corr1,self.N_nodes,
                                             self.Nodes_qt,self.K,self.D_tol,self.nextNode,
                                             self.Lambda,self.task,self.Q_Lambda)
#        new_nextNode=[]
#        for i in nextNode:
#            new_nextNode.append(i[0])
        print(nextNode)
        # return [i[0] for i in nextNode]
        return nextNode

        
    def get_goal_from_user(self):
        return        
    
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
        self.Pos_corr1[0]=[self.tb3_0_xp_odom,self.tb3_0_yp_odom]  

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
        self.Pos_corr1[1]=[self.tb3_1_xp_odom,self.tb3_1_yp_odom]





if __name__ == '__main__':
    try:
        node = RepeatMoveBaseGoal()
        node.run()
    except rospy.ROSInterruptException:
        pass