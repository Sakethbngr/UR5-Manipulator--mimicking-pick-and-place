#!/usr/bin/env python3
import rospy

from std_msgs.msg import Float64

import sys, select, termios, tty
msg = """
UR5 Controls!
---------------------------
Manipulator keys:
   Joint 1: 1,q
   Joint 2: 2,w
   Joint 3: 3,e
   Joint 4: 4,r
   Joint 5: 5,t

"""


def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key



if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('turtlebot_teleop')

    pub_j1 = rospy.Publisher('ur5v9/trans_link1_joint/command', Float64, queue_size=10) # Add your topic here between ''. Eg '/my_robot/steering_controller/command'
    pub_j2 = rospy.Publisher('ur5v9/trans_link2_joint/command', Float64, queue_size=10)
    pub_j3 = rospy.Publisher('ur5v9/trans_link3_joint/command', Float64, queue_size=10)
    pub_j4 = rospy.Publisher('ur5v9/trans_link4_joint/command', Float64, queue_size=10)
    pub_j5 = rospy.Publisher('ur5v9/trans_link5_joint/command', Float64, queue_size=10)
    # pub_j6 = rospy.Publisher('ur5v9/trans_link6_joint/command', Float64, queue_size=10)

    #pub_move = rospy.Publisher('', Float64, queue_size=10) # Add your topic for move here '' Eg '/my_robot/longitudinal_controller/command'

    x = 0
    th = 0
    status = 0
    count = 0
    acc = 0.1
    target_speed = 0
    target_turn = 0
    control_speed = 0
    control_turn = 0
    try:
        print(msg)
        while(1):
            key = getKey()
            if key == '1':
                pub_j1.publish(0.5)
                print("j1 = 0.5")
            elif key == 'q':
                pub_j1.publish(-0.5)
                print("j1 = -0.5")
            elif key == '2':
                pub_j2.publish(0.5)
                print("j2 = 0.5")
            elif key == 'w':
                pub_j2.publish(-0.5)
                print("j2 = -0.5")
            elif key == '3':
                pub_j3.publish(0.5)
                print("j3 = 0.5")
            elif key == 'e':
                pub_j3.publish(-0.5)
                print("j3 = -0.5")
            elif key == '4':
                pub_j4.publish(0.5)
                print("j4 = 0.5")
            elif key == 'r':
                pub_j4.publish(-0.5)
                print("j4 = -0.5")
            elif key == '5':
                pub_j3.publish(0.5)
                print("j5 = 0.5")
            elif key == 'r':
                pub_j3.publish(-0.5)
                print("j5 = -0.5")
            # elif key == '6':
            #     pub_j4.publish(0.5)
            #     print("j6 = 0.5")
            # elif key == 't':
            #     pub_j4.publish(-0.5)
            #     print("j6 = -0.5")
            

    except:
        print(e)



    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)