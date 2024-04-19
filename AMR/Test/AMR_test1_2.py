#!/usr/bin/env python
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient
from std_msgs.msg import String

def move_to_goal(x, y):
    rospy.init_node('move_to_goal', anonymous=True)
    client = SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1.0
    rospy.loginfo("Sending goal to x=%f, y=%f", x, y)
    client.send_goal(goal)
    client.wait_for_result()
    rospy.loginfo("Reached goal!")

def destination_callback(data):
    rospy.loginfo("Received destination: %s", data.data)
    if data.data == '0':
        move_to_goal(-0.559, 0.165)  # Cafe 좌표
    elif data.data == '1':
        move_to_goal(-0.276, -0.579)  # Office1 좌표
    elif data.data == '2':
        move_to_goal(-0.704, -0.486)  # Office2 좌표
    else:
        rospy.loginfo("Invalid destination!")

def main():
    rospy.init_node('move_to_goal', anonymous=True)
    rospy.Subscriber('user_destination', String, destination_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass