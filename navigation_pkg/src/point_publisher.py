# #!/usr/bin/env python3
from roslib import *
import rospy
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from random import sample
from math import pow, sqrt



def point_publisher():

    pub = rospy.Publisher('/tb3_1/move_base/goal', MoveBaseAction, queue_size=10)

    rospy.init_node('point_publisher', anonymous=True)
    
    target=MoveBaseGoal()

    target.target_pose.header.frame_id="map"
    target.target_pose.header.stamp = rospy.Time.now()
    #target.header.seq = 10

    target.target_pose.pose.position.x = 5.0
    target.target_pose.pose.position.x= 2.0
    target.target_pose.pose.orientation.w = -1.0
    target.target_pose.pose.orientation.z = -1.0  
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
        point_publisher()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")



# import rospy
# import actionlib
# from move_base_msgs.msg import *
# from geometry_msgs.msg import Pose,PoseStamped, PoseWithCovarianceStamped, Point, Quaternion, Twist

# # rostopic pub /tb3_0/move_base_simple/goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'.0}}}'


# # def callback(data):

# #     x = data.data.position.x
# #     print("Direct: ", x)
# #     print("Direct: ", x," / ", "Sub Obj: ", data.data )
    


# def tb3_0_move_base_client():

#     print("###### avvio il client.")
#     client = actionlib.SimpleActionClient('move_base1', MoveBaseAction)  
#     print("###### Pubblicazione dello status del client:\n\n#################\n\n" + str(client.get_state)+ "\n\n#################\n\n")
#     client.get_state
#     print("###### In attesa di risposta dal server.")
#     client.wait_for_server
#     print("###### Conneso al server!\n")
    
#     # goal = MoveBaseGoal()
#     # goal.target_pose.header.frame_id = "map"
#     # goal.target_pose.header.stamp = rospy.Time.now()
#     # print("Inserire coordinata X nel formato n.n: ")
#     # goal.target_pose.pose.position.x = input()
#     # print("Inserire coordinata y nel formato n.n: ")
#     # goal.target_pose.pose.position.y = input()
#     # print("Inserire coordinata z nel formato n.n: ")
#     # goal.target_pose.pose.orientation.z = input()
#     # print("Inserire coordinata w nel formato n.n: ")
#     # goal.target_pose.pose.orientation.w = input()

#     goal = MoveBaseGoal()
#     goal.target_pose.header.seq = 0
#     goal.target_pose.header.frame_id = "map"
#     goal.target_pose.header.stamp = rospy.Time.now()
    
#     goal.target_pose.pose = Pose(Point(2.0, 2.0, 0.000), Quaternion(0.000, 0.000, 0.223, 0.975))

#     # goal.target_pose.pose.position.x = 1.0
#     # print("Inserire coordinata x nel formato n.n: " + str(goal.target_pose.pose.position.x))
#     # goal.target_pose.pose.position.y = 1.0
#     # print("Inserire coordinata y nel formato n.n: " + str(goal.target_pose.pose.position.y))
#     # goal.target_pose.pose.orientation.z = 1.0
#     # print("Inserire coordinata Z nel formato n.n: " + str(goal.target_pose.pose.orientation.z))
#     # goal.target_pose.pose.orientation.w = 0.0
#     # print("Inserire coordinata W nel formato n.n: " + str(goal.target_pose.pose.orientation.w))

#     print("\n###### Inoltro della richiesta al server:\n" + str(goal))
#     client.send_goal(goal)
#     print("###### Attesa della risposta dal server.")
#     wait = client.wait_for_result()
#     if not wait:
#         rospy.logerr("Action server not available!")
#         rospy.signal_shutdown("Action server not available!")
#     else:
#         return client.get_result()

# if __name__ == '__main__':
#     try:
#         rospy.init_node('tb3_0_move_base_client')
#         result = tb3_0_move_base_client()
#         if result:
#             rospy.loginfo("Goal execution done!")
#     except rospy.ROSInterruptException:
#         rospy.loginfo("Navigation test finished.")