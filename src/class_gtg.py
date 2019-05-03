#!/usr/bin/env python
import roslib
import fcl
import moveit_commander
import moveit_msgs.msg
import actionlib
# import move_base_msgs
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# from moveit_msgs.msg import MoveItErrorCodes
# from moveit_python import MoveGroupInterface, PlanningSceneInterface
#roslib.load_manifest('my_package')
from scipy import linalg
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, PoseStamped, Quaternion, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
import tf
import time
from math import pi, cos, sin, atan2, sqrt
import numpy as np
import argparse
import sys
import rospy
from std_msgs.msg import String, Header
from std_msgs.msg import Float32

# if robot not moving, init has been set to False

class G2g:


    # init method or constructor  
    def __init__(self): 
        
        self.check_collision = True
        self.ang_comp = False 
        self.ang_comp2 = False
        self.lin_comp = False
        self.init = False
        self.angG = 0
        self.ang = 0
        self.quaternion = np.zeros((8, 4))
        self.translation = np.zeros((8, 3))
        self.ret = np.zeros(7)
        
        self.obj = [fcl.CollisionObject(fcl.Sphere(0.5), fcl.Transform()) for i in range(7)]
        for i in range(8):
            self.quaternion[i] = np.array([0, 0, 0, 0])
            self.translation[i] = np.array([0, 0, 0])
        self.quaternionG = np.array([0, 0, 0, 0])
        self.translationG = np.array([0, 0, 0])
       

    def goToGoal(self):
        
        self.callback()

        objectLocation = rospy.Subscriber("/gazebo/model_states", ModelStates, self.current_pose)
        # objectLocation = rospy.Subscriber("/odom", Odometry, self.current_pose)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    
    def callback(self): 
        ################################################# sets the goal pose  ###############################################

        dxG = 0.400110849627
        dyG = -0.464287270367
        dzG = 0.0004
        dxo = -0.00141823052058
        dyo = -0.00316135772783
        dzo = -0.408871135833
        dwo = 0.912585551454

        
        # self.dxG = -1.95996566627
        # self.dyG = -4.89012928885
        # self.dzG = 0.000615345201244
        
        # dxo = -3.86143497942e-05
        # dyo =  -0.00146424462051
        # dzo = -0.0239430488415
        # dwo = 0.999712251055
      

        self.translationG = np.array([dxG, dyG, dzG])
        self.quaternionG = np.array([dxo, dyo, dzo, dwo])
        eulerG = tf.transformations.euler_from_quaternion(self.quaternionG)
        self.angG = eulerG[2]
        
        # print "desired position:  ", self.dxG, ",  ", self.dyG, "   desired angle:  ", self.angG*180/pi
        time.sleep(1)

        self.init = False
        self.ang_comp = False 
        self.lin_comp = False


    def current_pose(self, data):

        ################################ reads the current pose of the robot and all objects in the world  ####################################

        world = ['blue_box', 'white_sphere', 'red_cylinder', 'green_box', 'turquoise_box', 'blue_cylinder', 'white_box', 'fetch']

        dx, dy, dz, dxo, dyo, dzo, dwo = np.zeros((7,len(world)))
        for i in range(len(world)):
            dx[i] = data.pose[i+1].position.x
            dy[i] = data.pose[i+1].position.y
            dz[i] = data.pose[i+1].position.z
            dxo[i] = data.pose[i+1].orientation.x
            dyo[i] = data.pose[i+1].orientation.y
            dzo[i] = data.pose[i+1].orientation.z
            dwo[i] = data.pose[i+1].orientation.w
              
            self.translation[i] = np.array([dx[i], dy[i], dz[i]])
            self.quaternion[i] = np.array([dxo[i], dyo[i], dzo[i], dwo[i]])

        # dx[7] = data.pose.pose.position.x
        # dy[7] = data.pose.pose.position.y
        # dz[7] = data.pose.pose.position.z
        # dxo[7] = data.pose.pose.orientation.x
        # dyo[7] = data.pose.pose.orientation.y
        # dzo[7] = data.pose.pose.orientation.z
        # dwo[7] = data.pose.pose.orientation.w
        # self.translation[7] = np.array([dx[7], dy[7], dz[7]])
        # self.quaternion[7] = np.array([dxo[7], dyo[7], dzo[7], dwo[7]])

        euler = tf.transformations.euler_from_quaternion(self.quaternion[7])
        #print self.quaternion[7]
        self.ang = euler[2]

        if self.check_collision:
            self.collision_check()
            #time.sleep(2)
            #self.check_collision = False

        if self.init:
            self.control(self.translation[7], self.ang)
            #self.control(self.translationG, self.quaternionG)
        
        #check if the distance starts increasing. If it is, try to re_orient the angle. and drive
        # consider state machines


   

    def control(self, d, ang):
        dxG = self.translationG[0]
        dyG = self.translationG[1]
        angG = self.angG
    
        drive = Twist()

        x_state = d[0]
        y_state = d[1]

        #print x_state, y_state
        
        y_diff = dyG - y_state
        x_diff = dxG - x_state

        ratio = y_diff/x_diff
        ang_goal = atan2(y_diff, x_diff)

        if self.ang_comp == False:
            ang_err = ang - ang_goal
            print "Bearing error  =  ", abs(ang_err * 180/pi), " degrees" 
            u_ang = -0.5*ang_err
            drive.angular.z = u_ang
            drive.linear.x = 0
            
            if abs(ang_err) < 0.01:
                print "############  Bearing angle reached at:  ", ang * 180/pi," degrees  #############"
                drive.angular.z = 0
                self.ang_comp = True
                time.sleep(4)

        if self.ang_comp == True and self.lin_comp == False:
            r_err = sqrt(x_diff**2 + y_diff**2)
            print "Distance error  =  ", r_err
            #print "current state  =  ", x_state, ",  ", y_state
            u_lin = 0.2*r_err
            drive.linear.x = u_lin

            ang_err = ang - ang_goal
            u_ang = -0.05*ang_err
            drive.angular.z = u_ang
            if r_err < 0.05:
                drive.linear.x = 0
                self.lin_comp = True
                print "------Reached goal position ------------------------------", x_state, ",  ", y_state, ", cur_ang = ",ang*180/pi, " ---------"
                time.sleep(4)
                #print "desired position:  ", dxG, ",  ", dyG, "   desired angle:  ", angG*180/pi
                
                
        if self.ang_comp == True and self.lin_comp == True and self.ang_comp2 == False:
            ang2_err = ang - angG
            print "Second bearing error  =  ", abs(ang2_err * 180/pi), " degrees" 
            u2_ang = -0.5*ang2_err
            drive.angular.z = u2_ang
            if abs(ang2_err) < 0.005:
                print "############  Second bearing angle reached at:  ", ang * 180/pi," degrees  #############"
                drive.angular.z = 0
                #time.sleep(4)
                print "------Reached goal at:  ", x_state, ",  ", y_state, ", cur_ang = ",ang*180/pi, " ---------"
                print "desired position:  ", dxG, ",  ", dyG, "   desired angle:  ", angG*180/pi
                self.ang_comp2 = True
                self.check_collision = False
            

            

        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        pub.publish(drive)


    def collision_check(self):

        #world = ['blue_box', 'white_sphere', 'red_cylinder', 'green_box', 'turquoise_box', 'blue_cylinder', 'white_box', 'fetch']

        robot = fcl.Cylinder(0.4, 1)
        tR = fcl.Transform(self.quaternion[7], self.translation[7])
        print self.translation[7]
        oR = fcl.CollisionObject(robot, tR)
        

        ob0 = fcl.Box(0.3, 1, 0.8)
        tr0 = fcl.Transform(self.quaternion[0], self.translation[0])
        self.obj[0] = fcl.CollisionObject(ob0, tr0)

        ob1 = fcl.Sphere(0.5)
        tr1 = fcl.Transform(self.quaternion[1], self.translation[1])
        self.obj[1] = fcl.CollisionObject(ob1, tr1)

        ob2 = fcl.Cylinder(0.5, 1)
        tr2 = fcl.Transform(self.quaternion[2], self.translation[2])
        self.obj[2] = fcl.CollisionObject(ob2, tr2)

        ob3 = fcl.Box(0.5, 1.4, 0.8)
        tr3 = fcl.Transform(self.quaternion[3], self.translation[3])
        self.obj[3] = fcl.CollisionObject(ob3, tr3)

        
        ob4 = fcl.Box(1, 5, 1)
        tr4 = fcl.Transform(self.quaternion[4], self.translation[4])
        self.obj[4] = fcl.CollisionObject(ob4, tr4)

        ob5 = fcl.Cylinder(0.5, 1)
        tr5 = fcl.Transform(self.quaternion[5], self.translation[5])
        self.obj[5] = fcl.CollisionObject(ob5, tr5)

        ob6 = fcl.Box(5, 1, 1)
        tr6 = fcl.Transform(self.quaternion[6], self.translation[6])
        self.obj[6] = fcl.CollisionObject(ob6, tr6)
        
       

        request = fcl.CollisionRequest()
        result = fcl.CollisionResult()
        
        for i in range(7):
            self.ret[i] = fcl.collide(oR, self.obj[i], request, result)

            # ret = fcl.continuousCollide(oR, tR, o_wb, t_wb, request, result)
            if self.ret[i]:
                print "--------------- YES  ", self.ret[i], " --------------------"
            else:
                print "--------------- NO ", self.ret[i], " -------------------"
            #time.sleep(2)





if __name__ == '__main__':

    print "============ Starting tutorial setup"
    # moveit_commander.roscpp_initialize(sys.argv)
    # rospy.init_node('move_group_python_interface_tutorial',
    #                 anonymous=True)
    rospy.init_node('goToGoal', anonymous=True)
    G = G2g()
    G.goToGoal()