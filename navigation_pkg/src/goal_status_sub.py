#!/usr/bin/env python3

from actionlib import GoalStatusArray
import rospy


def callback(data):
    # tipo=data
    # a={1,2,3,4}
    # b=(1,2,3)
    # c= lambda x: x
    #sto aprendo il dizionario status_list che si trova alla prima voce della lista
    #print(str(data.status_list[0].status) +"\t" + data.status_list[0].text)
    #print(f'{data.status_list[0].status}  gsgas  {data.status_list[0].text}')
    # print(f'{a} ertyui{b}  faaf  {c}')
    data.status_list[0].text
    data.status_list[0].status


def goal_status_subscriber():
    rospy.init_node("goal_status_subscriber", anonymous=True)
    rospy.Subscriber("/tb3_1/move_base/status", GoalStatusArray, callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        goal_status_subscriber()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")