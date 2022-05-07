#!/usr/bin/env python3

LENGTH = 16
INCREMENT = 5

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

if __name__ == "__main__":
    rospy.init_node('movebase_client_py')
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
 
   # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

   # Creates a new goal with the MoveBaseGoal constructor
    for i in range(7):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "odom"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = LENGTH if (i%2)==0 else 0;
        goal.target_pose.pose.position.y = INCREMENT*(i+1);
        goal.target_pose.pose.orientation.w = 1.0
        rospy.loginfo(f"going to {goal}")


        client.send_goal(goal)
        wait = client.wait_for_result()
