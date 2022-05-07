#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseArray, Pose


def mb_client():
    i = 1
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    posearray = []
    for coord in goals:
        goal = Pose()
        goal.header.frame_id = "odom"
        goal.header.stamp = rospy.Time.now()
        goal.position.x = coord[0]
        goal.position.y = coord[1]
        goal.orientation.w = 1.0
        rospy.loginfo(f"going to {goal}")

        client.send_goal(goal)
        wait = client.wait_for_result()
        rospy.loginfo(f"goal no {i} reached")
        i += 1
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_sequence')
        pub = rospy.Publisher("/move_base_sequence/wayposes", PoseArray, queue_size=10)
        goals = [(3, 0), (3, -2), (0, -2), (0, -4), (3, -4), (3, -6), (0, -6)]
        posearray = []
        for coord in goals:
            goal = Pose()
            goal.position.x = coord[0]
            goal.position.y = coord[1]
            goal.orientation.w = 1.0
            posearray.append(goal)

            posearray_msg = PoseArray()
            posearray_msg.header.frame_id = "odom"
            posearray_msg.header.stamp = rospy.Time.now()
            posearray_msg.poses = posearray
            pub.publish(posearray_msg)

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")