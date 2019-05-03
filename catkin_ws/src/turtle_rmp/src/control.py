#!/usr/bin/env python

import rospy
import roslib
import math
import time
import numpy as np
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from turtlesim.msg import Pose

global x_goal, y_goal, ang_comp, lin_comp
ang_comp = False 
lin_comp = False

def callback(data):
    global x_goal, y_goal, ang_comp, lin_comp

    drive = Twist()
    x_state = data.x
    y_state = data.y

    #print x_state, y_state
    
    y_diff = y_goal - y_state
    x_diff = x_goal - x_state

    ratio = y_diff/x_diff
    ang_goal = math.atan2(y_diff, x_diff)

    if ang_comp == False:
        phi = data.theta  
        ang_err = phi - ang_goal
        print "Bearing error  =  ", abs(ang_err * 180/math.pi), " degrees" 
        u_ang = -0.2*ang_err
        drive.angular.z = u_ang
        
        if abs(ang_err) < 0.005:
            print "############  Bearing angle reached at:  ", phi * 180/math.pi," degrees  #############"
            drive.angular.z = 0
            ang_comp = True
            time.sleep(4)

    if ang_comp == True and lin_comp == False:
        r_err = math.sqrt(x_diff**2 + y_diff**2)
        print "Distance error  =  ", r_err
        #print "current state  =  ", x_state, ",  ", y_state
        u_lin = 0.2*r_err
        drive.linear.x = u_lin
        if r_err < 0.04:
            print "-------------Reached goal at:  ", x_state, ",  ", y_state, " ---------------"
            drive.linear.x = 0
            lin_comp = True

    

    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=5)
    pub.publish(drive)


def control():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    global x_goal, y_goal, ang_comp, lin_comp

    print "Please enter the x and then y coordinate of the goal"
    x_goal = int(input())
    y_goal = int(input())

    while x_goal < 0 or y_goal < 0 or x_goal > 11 or y_goal > 11:
        print "Please enter numbers between 0 and 11"
        x_goal = int(input())
        y_goal = int(input())

    print "Desired goal (Xg, Yg) = (", x_goal, ", ", y_goal, ")"
    time.sleep(4)
    
    rospy.init_node('getState', anonymous=True)
    robot_pose = rospy.Subscriber("/turtle1/pose", Pose, callback)
    #rospy.Subscriber('chatter', String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    control()