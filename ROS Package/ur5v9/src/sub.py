#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

def callbackj1(data):
    rospy.loginfo(rospy.get_caller_id() + "Joint 1 position updated %f", data.data)
def callbackj2(data):
    rospy.loginfo(rospy.get_caller_id() + "Joint 1 position updated %f", data.data)
def callbackj3(data):
    rospy.loginfo(rospy.get_caller_id() + "Joint 1 position updated %f", data.data)
def callbackj4(data):
    rospy.loginfo(rospy.get_caller_id() + "Joint 1 position updated %f", data.data)
def callbackj6(data):
    rospy.loginfo(rospy.get_caller_id() + "Joint 1 position updated %f", data.data)
def callbackj7(data):
    rospy.loginfo(rospy.get_caller_id() + "Joint 1 position updated %f", data.data)


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("ur5v9/trans_link1_joint/command", Float64, callbackj1)
    rospy.Subscriber("ur5v9/trans_link1_joint/command", Float64, callbackj2)
    rospy.Subscriber("ur5v9/trans_link1_joint/command", Float64, callbackj3)
    rospy.Subscriber("ur5v9/trans_link1_joint/command", Float64, callbackj4)
    rospy.Subscriber("ur5v9/trans_link1_joint/command", Float64, callbackj6)
    rospy.Subscriber("ur5v9/trans_link1_joint/command", Float64, callbackj7)


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()