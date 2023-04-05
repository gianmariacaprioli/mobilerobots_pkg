#!/usr/bin/env python3


########### scritto in data 04/04/23 pub/sub per goal

from roslib import *
import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from random import sample
from math import pow, sqrt
from navigation_pkg.srv import get_state, get_stateResponse
import sys
import signal

def callback(data):
    #print(f'cod: {data.status_list[0].status}   text_status:{data.status_list[0].text}')
        
        status=data.status_list[0].status
        if status == 3:
            print("goal reached")
            return 3
        else:
             print("new plan, code = 2")

def goal_publisher():
    
    pub = rospy.Publisher('/tb3_1/move_base_simple/goal', PoseStamped, queue_size=10)
    
    state = rospy.ServiceProxy('goal_get_state',get_state)
    
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
        cod = state()

        if cod.status == 3:
            break     
        r.sleep()


    msg="uscita dal while"

    return print(str(msg))



if __name__ == '__main__':
    try:
        rospy.wait_for_service("goal_status_server")
        rospy.sleep(0.3)
        rospy.init_node('goal_publisher', anonymous=True)
        goal_publisher()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

#################################################################################
#################################################################################
#################################################################################
#################################################################################


#!/usr/bin/env python3

########### scritto in data 04/04/23 pub/sub per goal da chatgpt e corretto

import rospy
from geometry_msgs.msg import PoseStamped
from actionlib import GoalStatusArray

class MoveBaseNode:

    def __init__(self):
        rospy.init_node('move_base_ps', anonymous=True)

        self.goal_pub = rospy.Publisher('/tb3_1/move_base_simple/goal', PoseStamped, queue_size=10)
        rospy.Subscriber('/tb3_1/move_base/status', GoalStatusArray, self.status_callback)

        self.goal_reached = False

    def status_callback(self, data):
        print(data.status_list[0].status)
        if data.status_list[0].status == 3:  # check if the goal has been reached
            print(data.status_list[0].status)
            self.goal_reached = True

    def run(self):
        rate = rospy.Rate(0.5)
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position.x = 5.0
        goal_msg.pose.position.y = 5.0
        goal_msg.pose.orientation.w = 1.0

        while not rospy.is_shutdown():
            
            self.goal_pub.publish(goal_msg)
            rate.sleep()

            if self.goal_reached == True:
                rospy.loginfo("goal raggiunto")
                break

if __name__ == '__main__':
    move_base_node = MoveBaseNode()
    move_base_node.run()

    #rendere lo script con gli input ed aggiungere un controllo successivo 
    #per non far stuckkare l'algoritmo dopo il primo goal


#################################################################################
#################################################################################
#################################################################################
#################################################################################

# 05/04/23 script di chat gpc che accetta parametri in input
#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray

class RecursiveGoalPublisher:
    def __init__(self):
        # initialize ROS node
        rospy.init_node('recursive_goal_publisher', anonymous=True)

        # initialize subscriber to robot's status
        self.status_sub = rospy.Subscriber('/tb3_1/move_base/status', GoalStatusArray, self.status_callback)

        # initialize publisher to robot's goal
        self.goal_pub = rospy.Publisher('/tb3_1/move_base_simple/goal', PoseStamped, queue_size=10)

        # set initial goal to None
        self.goal = None

        # start main loop
        self.loop()

    def loop(self):
        # ask user for goal position and orientation
        x = float(input('Enter x position of goal: '))
        y = float(input('Enter y position of goal: '))
        z = float(input('Enter z position of goal: '))
        qw = float(input('Enter w orientation of robot: '))
        # qx = float(input('Enter x orientation of robot: '))
        # qy = float(input('Enter y orientation of robot: '))
        # qz = float(input('Enter z orientation of robot: '))

        # create goal message
        self.goal = PoseStamped()
        self.goal.header.frame_id = "map"
        self.goal.pose.position.x = x
        self.goal.pose.position.y = y
        self.goal.pose.position.z = z
        self.goal.pose.orientation.w = qw
        # self.goal.pose.orientation.x = qx
        # self.goal.pose.orientation.y = qy
        # self.goal.pose.orientation.z = qz

        # publish goal message
        self.goal_pub.publish(self.goal)

    def status_callback(self, msg):
        # check if the current goal is still pending or active
        for goal_status in msg.status_list:
            if goal_status.goal_id.id == self.goal.header.seq and goal_status.status in [0, 1]:
                return

        # if the current goal is achieved, cancel it and ask for a new goal
        self.goal = None
        self.loop()

if __name__ == '__main__':
    try:
        RecursiveGoalPublisher()
    except rospy.ROSInterruptException:
        pass
