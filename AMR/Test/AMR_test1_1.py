
#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def main():
    rospy.init_node('user_input_publisher', anonymous=True)
    pub = rospy.Publisher('user_destination', String, queue_size=10)
    rate = rospy.Rate(1)  # 1Hz

    while not rospy.is_shutdown():
        choice = raw_input("|-------------------------------|\n"
                           "| WHERE TO GO? (0: Cafe, 1: Office1, 2: Office2, q: Quit)\n"
                           "|-------------------------------|\n")
        if choice in ['0', '1', '2', 'q']:
            pub.publish(choice)
            if choice == 'q':
                rospy.loginfo("Exiting user_input_publisher node.")
                break
        else:
            rospy.loginfo("Invalid choice. Please enter a valid option.")

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass