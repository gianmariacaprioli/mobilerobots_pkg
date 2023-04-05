#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from actionlib import GoalStatusArray, GoalID

class MoveBaseNode:

    def __init__(self):
        rospy.init_node('move_base_ps', anonymous=True)

        self.goal_pub = rospy.Publisher('/tb3_1/move_base_simple/goal', PoseStamped, queue_size=10)
        # self.cancel_goal_pub = rospy.Publisher("/tb3_1/move_base/cancel", GoalID, queue_size=10)
        rospy.Subscriber('/tb3_1/move_base/status', GoalStatusArray, self.status_callback)

        # self.cancel_msg=GoalID()
        self.goal_reached = False

    def input_fnc(self):

        # self.cancel_goal_pub.pub(self.cancel_msg.id)


        print(f'Enter the x (as 0.0) value: ')
        x = float(input())

        print(f'Enter the y (as 0.0) value: ')
        y = float(input())

        print(f'Enter the W (as 0.0) value: ')
        w = float(input())

        self.run(x,y,w)



    def status_callback(self, data):
        print(data.status_list[0])
        if data.status_list[0].status == 3:  # check if the goal has been reached
            print(data.status_list[0].status)
            # self.cancel_goal_pub.id = data.goalID.id
            self.goal_reached = True


    def run(self,x,y,w):
        rate = rospy.Rate(0.5)
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.orientation.w = w

        while not rospy.is_shutdown():

            self.goal_pub.publish(goal_msg)
            rate.sleep()

            if self.goal_reached == True:
                rospy.loginfo("goal raggiunto")
                break
        self.input_fnc()

if __name__ == '__main__':
    move_base_node = MoveBaseNode()
    x=5.0
    y=2.0
    w=1.0
    move_base_node.input_fnc()

    #rendere lo script con gli input ed aggiungere un controllo successivo
    #per non far stuckkare l'algoritmo dopo il primo goal
