#!/usr/bin/env python3

# Importing necessary Libraries
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from geometry_msgs.msg import Twist

import sympy
from sympy import *
import math


theta1, theta2, theta3, theta4, theta5, theta6, theta7,a1, a2, a3, a4, a5, a6, a7, d1, d2, d3, d4, d5, d6, d7,  alpha1, alpha2, alpha3, alpha4, alpha5, alpha6, alpha7 = symbols('theta1, theta2, theta3, theta4, theta5, theta6, theta7, a1, a2, a3, a4, a5, a6, a7, d1, d2, d3, d4, d5, d6, d7, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6, alpha7 ', real=True)

theta = [theta1, theta2, theta3, theta4, theta5, theta6, theta7]
a = [a1, a2, a3, a4, a5, a6, a7]
d = [d1, d2, d3, d4, d5, d6, d7]
alpha = [alpha1, alpha2, alpha3, alpha4, alpha5, alpha6, alpha7]




def transform_mat(theta, a, d, alpha):
    mat = sympy.Matrix([[cos(theta), (-sin(theta)*cos(alpha)), (sin(theta)*sin(alpha)), (a*cos(theta))],
        [sin(theta), (cos(theta)*cos(alpha)), (-cos(theta)*sin(alpha)), (a*sin(theta))], 
        [0, sin(alpha), cos(alpha), d], [0, 0, 0, 1]])

    return mat


def Z_0i(T):
    return sympy.Matrix([T[2], T[6], T[10]])


# Below are the a and d values of UR5 manipulator. We have directly put them in the DH table constructed below.
# a1 = 0
# a2 = -0.425	
# a3 = -0.39225	
# a4 = 0
# a5 = 0	
# a6 = 0

# d1 = 0.089159
# d2 = 0
# d3 = 0
# d4 = 0.10915	
# d5 = 0.09465	
# d6 = 0.0823


DH = sympy.Matrix([[theta1, 0, 0.089159, pi/2], 
            [theta2, -0.425, 0, -pi/2], 
            [theta3, -0.39925, 0, -pi/2],
            [theta4, 0, 0.10915, pi/2],
            [theta5, 0, 0.09465, pi/2], 
            [theta6, 0, 0.0823, -pi/2]])

# sympy.pprint(DH)


def talker():
    #Initializing Node
    rospy.init_node('publish_node', anonymous=True) # defining the ros node - publish node
    turning = rospy.Publisher('turn', Float64MultiArray, queue_size=10) 
    rate = rospy.Rate(10) # 10hz # fequency at which the publishing occurs
    rospy.loginfo("Analysing the Robot!!!")
    pub_j1 = rospy.Publisher('ur5v9/trans_link1_joint/command', Float64, queue_size=10)
    pub_j2 = rospy.Publisher('ur5v9/trans_link2_joint/command', Float64, queue_size=10)
    pub_j3 = rospy.Publisher('ur5v9/trans_link3_joint/command', Float64, queue_size=10)
    pub_j4 = rospy.Publisher('ur5v9/trans_link4_joint/command', Float64, queue_size=10)
    pub_j5 = rospy.Publisher('ur5v9/trans_link5_joint/command', Float64, queue_size=10)
    pub_j6 = rospy.Publisher('ur5v9/trans_link6_joint/command', Float64, queue_size=10)

    
  
    T1 = transform_mat(theta[0], a[0], d[0], alpha[0])
    T2 = transform_mat(theta[1], a[1], d[1], alpha[1])
    T3 = transform_mat(theta[2], a[2], d[2], alpha[2])
    T4 = transform_mat(theta[3], a[3], d[3], alpha[3])
    T5 = transform_mat(theta[4], a[4], d[4], alpha[4])
    T6 = transform_mat(theta[5], a[5], d[5], alpha[5])

    T_01 = T1
    T_02 = T_01*T2
    T_03 = T_02*T3
    T_04 = T_02*T3*T4
    T_05 = T_04*T5
    T_06 = T_05*T6

    Z1 = Z_0i(T_01)
    Z2 = Z_0i(T_02)
    Z3 = Z_0i(T_03)
    Z4 = Z_0i(T_04)
    Z5 = Z_0i(T_05)
    Z6 = Z_0i(T_06)




    xp = [diff(T_06[3], theta1), diff(T_06[3], theta2), diff(T_06[3], theta4), diff(T_06[3], theta5), diff(T_06[3], theta6), diff(T_06[3], theta7)]

    yp = [diff(T_06[7], theta1), diff(T_06[7], theta2), diff(T_06[7], theta4), diff(T_06[7], theta5), diff(T_06[7], theta6), diff(T_06[7], theta7)]

    zp = [diff(T_06[11], theta1), diff(T_06[11], theta2), diff(T_06[11], theta4), diff(T_06[11], theta5), diff(T_06[11], theta6), diff(T_06[11], theta7)]

    Jac = sympy.Matrix([ xp, yp, zp, [Z1, Z2, Z3, Z4, Z5, Z6]])


    Tf = sympy.Matrix([[-0.66, -0.181, 0.729, 0.219], 
            [0.588, 0.728, 0.352, -0.100], 
            [0.467, 0.661, 0.587, 0.300],
            [0, 0, 0, 1]])


    P_05 = Tf*sympy.Matrix([ 0, 0, -0.0823, 1])

    P_05 = P_05 - sympy.Matrix([ 0, 0, 0, 1])
    
    theta1_f = math.atan2(P_05[1], P_05[0]) + math.acos(0.10915/math.sqrt(P_05[0]**2 + P_05[1]**2)) + 1.57
    P_06 = sympy.Matrix([Tf[3], Tf[7], Tf[11]])

    theta5_f = math.acos((P_06[0]*math.sin(theta1_f) - P_06[1]*math.cos(theta1_f) - 0.10915)/0.0823)         

    x = sympy.Matrix([Tf[0], Tf[4], Tf[8]])
    y = sympy.Matrix([Tf[1], Tf[5], Tf[9]])
    theta6_f = math.atan2(((x[1]*math.sin(theta1_f)+ y[1]*math.cos(theta1_f))/math.sin(theta5_f)), ((x[0]*math.sin(theta1_f)+ y[0]*math.cos(theta1_f))/math.sin(theta5_f)))


    T_5_f = transform_mat(theta5_f, 0, 0.09465, sympy.pi/2)
    T_6_f = transform_mat(theta6_f, 0, 0.0823, -sympy.pi/2)
    h = (T_5_f*T_6_f).inv()
    T_1_f = transform_mat(theta1_f, 0, 0.089159, sympy.pi/2)
    T_16_f = (T_1_f.inv())*Tf
    T_14_f = T_16_f*h



    P_4xz = math.sqrt(T_14_f[0]**2 + T_14_f[2]**2)
    theta3_f = math.acos((P_4xz**2 - 0.425**2 - 0.39225**2)/2*0.425* 0.39225)
    theta2_f = math.atan2(-T_14_f[11], -T_14_f[3]) - math.asin(0.39225*math.sin(theta3_f)/P_4xz)
    theta4_f = math.atan2(T_14_f[4], T_14_f[0])


    pub_j1.publish(theta1_f)
    pub_j2.publish(theta2_f)
    pub_j3.publish(theta3_f)
    pub_j4.publish(theta4_f)
    pub_j5.publish(theta5_f)
    pub_j6.publish(theta6_f)
    rate.sleep()



talker()