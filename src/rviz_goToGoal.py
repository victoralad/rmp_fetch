#!/usr/bin/env python
import roslib
#roslib.load_manifest('my_package')
from scipy import linalg
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
import tf
import time
from math import pi, cos, sin, atan2, sqrt
import numpy as np
import argparse
import sys
import rospy
from std_msgs.msg import String 
from std_msgs.msg import Float32

ang_comp = False 
lin_comp = False
init = False
global dx0, dy0, ang0, R 

def callback1(data):
    global dx0, dy0, ang0, R, init, ang_comp, lin_comp 
    
    dx0 = data.pose.pose.position.x
    dy0 = data.pose.pose.position.y
    dxo = data.pose.pose.orientation.x
    dyo = data.pose.pose.orientation.y
    dzo = data.pose.pose.orientation.z
    dwo = data.pose.pose.orientation.w
    
    quaternion = np.array([dxo, dyo, dzo, dwo])
    euler = tf.transformations.euler_from_quaternion(quaternion)
    ang0 = euler[2]
    #R = np.array([[cos(-ang0), -sin(-ang0)], [sin(-ang0), cos(-ang0)]])

    print "desired position:  ", dx0, ",  ", dy0, "   desired angle:  ", ang0*180/pi
    time.sleep(4)
    init = True
    ang_comp = False 
    lin_comp = False

def callback2(data):
    global init
    
    #print "callback2"
    
    dx = data.pose.pose.position.x
    dy = data.pose.pose.position.y
    dxo = data.pose.pose.orientation.x
    dyo = data.pose.pose.orientation.y
    dzo = data.pose.pose.orientation.z
    dwo = data.pose.pose.orientation.w
    d = np.array([dx, dy])

    quaternion = np.array([dxo, dyo, dzo, dwo])
    euler = tf.transformations.euler_from_quaternion(quaternion)
    ang = euler[2]

    if init:
        control(d, ang)
    

def control(d, ang):
    global dx0, dy0, ang0, R, ang_comp, lin_comp
    

    drive = Twist()
    
    #d0 = np.array([dx0, dy0])


    
    x_state = d[0]
    y_state = d[1]

    #print x_state, y_state
    
    y_diff = dy0 - y_state
    x_diff = dx0 - x_state

    ratio = y_diff/x_diff
    ang_goal = atan2(y_diff, x_diff)

    if ang_comp == False:
        phi = ang  
        ang_err = phi - ang_goal
        print "Bearing error  =  ", abs(ang_err * 180/pi), " degrees" 
        u_ang = -1.2*ang_err
        drive.angular.z = u_ang
        
        if abs(ang_err) < 0.05:
            print "############  Bearing angle reached at:  ", phi * 180/pi," degrees  #############"
            drive.angular.z = 0
            ang_comp = True
            time.sleep(4)

    if ang_comp == True and lin_comp == False:
        r_err = sqrt(x_diff**2 + y_diff**2)
        print "Distance error  =  ", r_err
        #print "current state  =  ", x_state, ",  ", y_state
        u_lin = 1.2*r_err
        drive.linear.x = u_lin
        if r_err < 0.05:
            print "------Reached goal position ------------------------------"#, x_state, ",  ", y_state, ", cur_ang = ",ang*180/pi, " ---------"
            #print "desired position:  ", dx0, ",  ", dy0, "   desired angle:  ", ang0*180/pi
            ang2_err = ang - ang0
            print "Second bearing error  =  ", abs(ang2_err * 180/pi), " degrees" 
            u2_ang = -1.2*ang2_err
            drive.angular.z = u2_ang
            
            if abs(ang2_err) < 0.005:
                print "############  Second bearing angle reached at:  ", ang * 180/pi," degrees  #############"
                drive.angular.z = 0
                time.sleep(4)
                print "------Reached goal at:  ", x_state, ",  ", y_state, ", cur_ang = ",ang*180/pi, " ---------"
                print "desired position:  ", dx0, ",  ", dy0, "   desired angle:  ", ang0*180/pi
                lin_comp = True

        

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    pub.publish(drive)
    




def goToGoal():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    
    rospy.init_node('goToGoal', anonymous=True)
    objectGoal = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, callback1)
    objectLocation = rospy.Subscriber("/odom", Odometry, callback2)

    
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    goToGoal()