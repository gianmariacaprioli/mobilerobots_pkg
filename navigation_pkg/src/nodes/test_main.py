#!/usr/bin/env python3
import rospy
import tb3_0_goal_ps
import tb3_1_goal_ps

if __name__ == '__main__':
    tb3_1 = tb3_1_goal_ps.RepeatMoveBaseGoal()
    tb3_1.run()
    tb3_0 = tb3_0_goal_ps.RepeatMoveBaseGoal()
    tb3_0.run()

