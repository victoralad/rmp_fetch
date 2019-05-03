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
        self.lin_comp = False
        self.init = False
        self.dxG = 0
        self.dyG = 0
        self.dzG = 0
        self.angG = 0
        self.quaternion = np.zeros((10, 4))
        self.translation = np.zeros((10, 3))
        for i in range(10):
            self.quaternion[i] = np.array([0, 0, 0, 0])
            self.translation[i] = np.array([0, 0, 0])
        self.quaternionG = np.array([0, 0, 0, 0])
        self.translationG = np.array([0, 0, 0])
        # self.quaternion_wb = np.array([0, 0, 0, 0])
        # self.translation_wb = np.array([0, 0, 0])
        
        # self.robot = moveit_commander.RobotCommander()
        # self.scene = moveit_commander.PlanningSceneInterface()
        # self.group = moveit_commander.MoveGroupCommander("left_arm")

        # self.gripper_frame = 'base_link'
        # self.gripper_poses = [Pose(Point(0, 0, 0),
        #                   Quaternion(0, 0, 0, 0))]
        
        # self.gripper_pose_stamped = PoseStamped()
        # self.gripper_pose_stamped.header.frame_id = 'base_link'

    def goToGoal(self):
        
        

        self.callback()

        objectLocation = rospy.Subscriber("/gazebo/model_states", ModelStates, self.current_pose)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    
    def callback(self): 
        ################################################# sets the goal pose  ###############################################

        self.dxG = 0.400110849627
        self.dyG = -0.464287270367
        self.dzG = 0.0004
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

        self.translateG = [self.dxG, self.dyG, self.dzG]
        # self.translateG = np.array([dxG, dyG, dzG])
        self.quaternionG = np.array([dxo, dyo, dzo, dwo])
        eulerG = tf.transformations.euler_from_quaternion(self.quaternionG)
        self.angG = eulerG[2]
        
        # print "desired position:  ", self.dxG, ",  ", self.dyG, "   desired angle:  ", self.angG*180/pi
        time.sleep(1)

        self.init = True
        self.ang_comp = False 
        self.lin_comp = False


    def current_pose(self, data):

        ################################ reads the current pose of the robot and all objects in the world  ####################################

        world = ['cafe_table', 'blue_box', 'white_sphere', 'red_cylinder', 'green_box', 'purple_box', 'turquoise_box', 'blue_cylinder', 'white_box', 'fetch']

        # dx, dy, dz, dxo, dyo, dzo, dwo = np.zeros((7,len(world)))
        # for i in range(len(world)):
        #     dx[i] = data.pose[i+1].position.x
        #     dy[i] = data.pose[i+1].position.y
        #     dz[i] = data.pose[i+1].position.z
        #     dxo[i] = data.pose[i+1].orientation.x
        #     dyo[i] = data.pose[i+1].orientation.y
        #     dzo[i] = data.pose[i+1].orientation.z
        #     dwo[i] = data.pose[i+1].orientation.w
              
        #     self.translation[i] = np.array([dx[i], dy[i], dz[i]])
        #     self.quaternion[i] = np.array([dxo[i], dyo[i], dzo[i], dwo[i]])

        
        # euler = tf.transformations.euler_from_quaternion(self.quaternion[9])
        # ang = euler[2]

        if self.check_collision:
            self.collision_check()
            #time.sleep(2)
            #self.check_collision = False

        if self.init:
            #self.control(self.translation[9], ang)
            self.control(self.translationG, self.quaternionG)
        
        #check if the distance starts increasing. If it is, try to re_orient the angle. and drive
        # consider state machines

    def control(self, position, orientation):
        """Send a cartesian goal to the action server."""
        # #action_address = '/j2n6s300_driver/pose_action/tool_pose'
        # client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        # #print "hey"
        # client.wait_for_server()

        # goal = MoveBaseGoal()
        # goal.target_pose.header = Header(frame_id='map')
        # goal.target_pose.pose.position = Point(
        #     x=position[0], y=position[1], z=position[2])
        # goal.target_pose.pose.orientation = Quaternion(
        #     x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])

        # # print('goal.pose in client 1: {}'.format(goal.pose.pose)) # debug

        # client.send_goal(goal)
        

        # if client.wait_for_result(rospy.Duration(1.0)):
        #     return client.get_result()
        # else:
        #     client.cancel_all_goals()
        #     print('        the cartesian action timed-out')
        #     return None 
        
            # publish to whatever message the driving is going to take place


            ###############################################################################################
        
        client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        print "hey1"
        client.wait_for_server()
        print "hey2"

        
        #person found
        rospy.loginfo("Found person, generate goal")
        target_goal_simple = PoseStamped()
        #target_goal = MoveBaseGoal()

        #forming a proper PoseStamped message
        target_goal_simple.pose.position = Point(x=position[0], y=position[1], z=position[2])
        # target_goal_simple.pose.position.z = 0
        target_goal_simple.pose.orientation = Quaternion(x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])
        target_goal_simple.header.frame_id = 'base_link'
        target_goal_simple.header.stamp = rospy.Time.now()
        #target_goal.target_pose.pose.position = data.people[0].pos

        #sending goal
        rospy.loginfo("sending goal")
        pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size = 10)
        pub.publish(target_goal_simple)
        #client.send_goal(target_goal)

        #######################################################################################################

        # client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        # client.wait_for_server()

        # goal = MoveBaseGoal()
        # goal.target_pose.header.frame_id = "odom"
        # goal.target_pose.header.stamp = rospy.Time.now()
        # goal.target_pose.pose.position.x = 0.5
        # goal.target_pose.pose.orientation.w = 1.0

        # client.send_goal(goal)
        # wait = client.wait_for_result()
        # if not wait:
        #     rospy.logerr("Action server not available!")
        #     rospy.signal_shutdown("Action server not available!")
        # else:
        #     return client.get_result()

   

    def control2(self, d, ang):
        dxG = self.dxG
        dyG = self.dyG
        angG = self.angG
    
        drive = Twist()
        
        #d0 = np.array([dxG, dyG])


        
        x_state = d[0]
        y_state = d[1]

        #print x_state, y_state
        
        y_diff = dyG - y_state
        x_diff = dxG - x_state

        ratio = y_diff/x_diff
        ang_goal = atan2(y_diff, x_diff)

        if self.ang_comp == False:
            phi = ang  
            ang_err = phi - ang_goal
            print "Bearing error  =  ", abs(ang_err * 180/pi), " degrees" 
            u_ang = -0.5*ang_err
            drive.angular.z = u_ang
            drive.angular.x = 0
            
            if abs(ang_err) < 0.05:
                print "############  Bearing angle reached at:  ", phi * 180/pi," degrees  #############"
                drive.angular.z = 0
                self.ang_comp = True
                time.sleep(4)

        if self.ang_comp == True and self.lin_comp == False:
            r_err = sqrt(x_diff**2 + y_diff**2)
            print "Distance error  =  ", r_err
            #print "current state  =  ", x_state, ",  ", y_state
            u_lin = 0.5*r_err
            drive.linear.x = u_lin
            if r_err < 0.1:
                drive.linear.z = 0
                print "------Reached goal position ------------------------------"#, x_state, ",  ", y_state, ", cur_ang = ",ang*180/pi, " ---------"
                #print "desired position:  ", dxG, ",  ", dyG, "   desired angle:  ", angG*180/pi
                ang2_err = ang - angG
                print "Second bearing error  =  ", abs(ang2_err * 180/pi), " degrees" 
                u2_ang = -1.2*ang2_err
                drive.angular.z = u2_ang
                
                if abs(ang2_err) < 0.005:
                    print "############  Second bearing angle reached at:  ", ang * 180/pi," degrees  #############"
                    drive.angular.z = 0
                    time.sleep(4)
                    print "------Reached goal at:  ", x_state, ",  ", y_state, ", cur_ang = ",ang*180/pi, " ---------"
                    print "desired position:  ", dxG, ",  ", dyG, "   desired angle:  ", angG*180/pi
                    self.lin_comp = True

            

        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        pub.publish(drive)


    def collision_check(self):

        #world = ['cafe_table', 'blue_box', 'white_sphere', 'red_cylinder', 'green_box', 'purple_box', 'turquoise_box', 'blue_cylinder', 'white_box', 'fetch']

        robot = fcl.Box(1, 1, 2)
        tR = fcl.Transform(self.quaternion[9], self.translation[9])
        oR = fcl.CollisionObject(robot, tR)
        tR_final = fcl.Transform(self.quaternionG, self.translationG)

        wb = fcl.Box(5, 1, 1)
        t_wb = fcl.Transform(self.quaternion[8], self.translation[8])
        o_wb = fcl.CollisionObject(wb, t_wb)
        #tBb_final = fcl.Transform(self.quaternionG, self.translationG)

        request = fcl.ContinuousCollisionRequest()
        result = fcl.ContinuousCollisionResult()

        ret = fcl.continuousCollide(oR, tR_final, o_wb, t_wb, request, result)
        if ret:
            print "--------------- NO  ", ret, " --------------------"
        else:
            print "--------------- YES ", ret, " -------------------"
        #time.sleep(2)





if __name__ == '__main__':

    print "============ Starting tutorial setup"
    # moveit_commander.roscpp_initialize(sys.argv)
    # rospy.init_node('move_group_python_interface_tutorial',
    #                 anonymous=True)
    rospy.init_node('goToGoal', anonymous=True)
    G = G2g()
    G.goToGoal()