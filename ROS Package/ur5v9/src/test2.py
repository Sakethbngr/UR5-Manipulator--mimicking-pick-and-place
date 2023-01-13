#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist


def talker():
    pub_j1 = rospy.Publisher('ur5v9/trans_link1_joint/command', Float64, queue_size=10)  # Add your topic here between ''. Eg '/my_robot/steering_controller/command'
    pub_j2 = rospy.Publisher('ur5v9/trans_link2_joint/command', Float64, queue_size=10)
    pub_j3 = rospy.Publisher('ur5v9/trans_link3_joint/command', Float64, queue_size=10)
    pub_j4 = rospy.Publisher('ur5v9/trans_link4_joint/command', Float64, queue_size=10)
    pub_j5 = rospy.Publisher('ur5v9/trans_link5_joint/command', Float64, queue_size=10)
    pub_j6 = rospy.Publisher('ur5v9/trans_link6_joint/command', Float64, queue_size=10)
    twist = Float64MultiArray()
    twist = 1.57
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub_j1.publish(twist)
        # pub_j2.publish(twist)
        # pub_j3.publish(twist)
        pub_j4.publish(twist)
        # pub_j5.publish(twist)        
        # pub_j6.publish(twist)
        rate.sleep()


    



if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass