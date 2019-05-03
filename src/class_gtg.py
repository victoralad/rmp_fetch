#!/usr/bin/env python
import roslib
import fcl
import moveit_commander
import moveit_msgs.msg
import actionlib
from actionlib_msgs.msg import GoalStatusArray
# import move_base_msgs
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
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
        self.send_g = True
        self.status = 0
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
        
        self.set_goal()

        objectLocation = rospy.Subscriber("/gazebo/model_states", ModelStates, self.current_pose)
        goal_stat =   rospy.Subscriber("/move_base/status", GoalStatusArray, self.goal_status)
        self.arm_move()
        # objectLocation = rospy.Subscriber("/odom", Odometry, self.current_pose)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    
    def set_goal(self): 
        ################################################# sets the goal pose  ###############################################

        # dxG = 0.400110849627
        # dyG = -0.464287270367
        # dzG = 0.0004
        # dxo = -0.00141823052058
        # dyo = -0.00316135772783
        # dzo = -0.408871135833
        # dwo = 0.912585551454

        
        dxG = 3.16058202426
        dyG = 2.49855960764
        dzG = 0.000615345201244
        
        dxo = -0.00219576617675
        dyo =  -0.000729841179802
        dzo = -0.948937286123
        dwo = 0.315456293247
      

        self.translationG = np.array([dxG, dyG, dzG])
        self.quaternionG = np.array([dxo, dyo, dzo, dwo])
        eulerG = tf.transformations.euler_from_quaternion(self.quaternionG)
        self.angG = eulerG[2]
        
        # print "desired position:  ", self.dxG, ",  ", self.dyG, "   desired angle:  ", self.angG*180/pi
        time.sleep(1)

        self.init = False
        self.check_collision = False
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
            #self.check_collision = False

        if self.init:
            # self.control(self.translation[7], self.ang)
            self.control(self.translationG, self.quaternionG)
        
        #check if the distance starts increasing. If it is, try to re_orient the angle. and drive
        # consider state machines


    def control(self, position, orientation):
        """Send a cartesian goal to the action server."""
        #action_address = '/j2n6s300_driver/pose_action/tool_pose'
        client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        #print "hey"
        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header = Header(frame_id='map')
        goal.target_pose.pose.position = Point(
            x=position[0], y=position[1], z=position[2])
        goal.target_pose.pose.orientation = Quaternion(
            x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])

        # print('goal.pose in client 1: {}'.format(goal.pose.pose)) # debug
        # client.cancel_all_goals()

        if self.send_g:
            client.send_goal(goal)
            self.send_g = False

        # if self.status == 1:
        #     client.send_goal(goal)

        if self.status == 3:
            print "waypoint reached"
            
            # self.status = 0
            # self.init = False

        
        
        print self.status
        
        
        
            

        # if client.wait_for_result(rospy.Duration(1.0)):
        #     client.get_result()
        # else:
        #     client.cancel_all_goals()
            # print('        the cartesian action timed-out')
            # return None 
        
            #publish to whatever message the driving is going to take place


            ###############################################################################################
        
        # client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        # client.wait_for_server()

        
        # #person found
        # #rospy.loginfo("Found path, generate goal")
        # target_goal_simple = PoseStamped()
        # #target_goal = MoveBaseGoal()

        # #forming a proper PoseStamped message
        # target_goal_simple.pose.position = Point(x=position[0], y=position[1], z=position[2])
        # # target_goal_simple.pose.position.z = 0
        # target_goal_simple.pose.orientation = Quaternion(x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])
        # target_goal_simple.header.frame_id = 'map'
        # target_goal_simple.header.stamp = rospy.Time.now()
        # #target_goal.target_pose.pose.position = data.people[0].pos

        # #sending goal
        # #rospy.loginfo("sending goal")
        # pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size = 10)
        # pub.publish(target_goal_simple)

        # print "result", client.get_result()
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

    def goal_status(self, data):
        if(len(data.status_list) > 0):
            self.status = data.status_list[len(data.status_list)-1].status

    def arm_move(self):

        move_group = MoveGroupInterface("arm_with_torso", "base_link")

        # Define ground plane
        # This creates objects in the planning scene that mimic the ground
        # If these were not in place gripper could hit the ground
        planning_scene = PlanningSceneInterface("base_link")
        planning_scene.removeCollisionObject("my_front_ground")
        planning_scene.removeCollisionObject("my_back_ground")
        planning_scene.removeCollisionObject("my_right_ground")
        planning_scene.removeCollisionObject("my_left_ground")
        planning_scene.addCube("my_front_ground", 2, 1.1, 0.0, -1.0)
        planning_scene.addCube("my_back_ground", 2, -1.2, 0.0, -1.0)
        planning_scene.addCube("my_left_ground", 2, 0.0, 1.2, -1.0)
        planning_scene.addCube("my_right_ground", 2, 0.0, -1.2, -1.0)

        # TF joint names
        joint_names = ["torso_lift_joint", "shoulder_pan_joint",
                    "shoulder_lift_joint", "upperarm_roll_joint",
                    "elbow_flex_joint", "forearm_roll_joint",
                    "wrist_flex_joint", "wrist_roll_joint"]
        # Lists of joint angles in the same order as in joint_names
        disco_poses = [[1, 2.5, -0.6, 3.0, 1.0, 3.0, 1.0, 3.0]]

        for pose in disco_poses:
            if rospy.is_shutdown():
                break

            # Plans the joints in joint_names to angles in pose
            move_group.moveToJointPosition(joint_names, pose, wait=False)

            # Since we passed in wait=False above we need to wait here
            move_group.get_move_action().wait_for_result()
            result = move_group.get_move_action().get_result()

            if result:
                # Checking the MoveItErrorCode
                if result.error_code.val == MoveItErrorCodes.SUCCESS:
                    rospy.loginfo("Disco!")
                else:
                    # If you get to this point please search for:
                    # moveit_msgs/MoveItErrorCodes.msg
                    rospy.logerr("Arm goal in state: %s",
                                move_group.get_move_action().get_state())
            else:
                rospy.logerr("MoveIt! failure no result returned.")

        # This stops all arm movement goals
        # It should be called when a program is exiting so movement stops
        move_group.get_move_action().cancel_all_goals()

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