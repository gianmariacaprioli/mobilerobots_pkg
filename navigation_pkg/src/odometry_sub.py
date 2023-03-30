#!/usr/bin/env python3

from nav_msgs.msg import Odometry
import rospy


def callback(data):
    
    print("#################\n"+f'POSE: x: {data.pose.pose.position.x}  y:{data.pose.pose.position.y}   z:{data.pose.pose.position.z}')
    print("\n"+f'ORIENTATION: W:{data.pose.pose.orientation.w}   X:{data.pose.pose.orientation.x}    Y:{data.pose.pose.orientation.y} Z:{data.pose.pose.orientation.z}')
    
    # print(dir(data))
    # tipo=data
    # a={1,2,3,4}
    # b=(1,2,3)
    # c= lambda x: x
    #sto aprendo il dizionario status_list che si trova alla prima voce della lista
    #print(str(data.status_list[0].status) +"\t" + data.status_list[0].text)
    #print(f'{data.status_list[0].status}  gsgas  {data.status_list[0].text}')
    # print(f'{a} ertyui{b}  faaf  {c}')
    # data.status_list[0].text
    # data.status_list[0].status


def odometry_sub():
    rospy.init_node("odometry_sub", anonymous=True)
    rospy.Subscriber("/tb3_1/odom", Odometry, callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        odometry_sub()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")