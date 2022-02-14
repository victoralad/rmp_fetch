#!/usr/bin/env python
import roslib
import fcl
import moveit_commander
import moveit_msgs.msg
import actionlib
import tf
import time
import numpy as np
import argparse
import sys
import rospy
from std_msgs.msg import String, Header
from std_msgs.msg import Float32
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from scipy import linalg
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, PoseStamped, Quaternion, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from actionlib_msgs.msg import GoalStatusArray
from math import pi, cos, sin, atan2, sqrt


# if robot not moving, init has been set to False

class G2g:

    # init method or constructor  
    def __init__(self): 
        
        self.check_collision = True
        self.ang_comp = False 
        self.ang_comp2 = False
        self.lin_comp = False
        self.init = False
        self.init_arm = False
        self.angG = 0
        self.ang = 0
        self.send_g = True
        self.status = 0
        self.goal_id = 0
        self.ret = np.zeros(7)
        self.obj = [fcl.CollisionObject(fcl.Sphere(0.5), fcl.Transform()) for i in range(7)]
        self.quaternion = np.zeros((8, 4))
        self.translation = np.zeros((8, 3))
        for i in range(8):
            self.quaternion[i] = np.array([0, 0, 0, 0])
            self.translation[i] = np.array([0, 0, 0])

        self.quaternionG = np.zeros((11, 4))
        self.translationG = np.zeros((11, 3))
        for i in range(10):
            self.quaternionG[i] = np.array([0, 0, 0, 0])
            self.translationG[i] = np.array([0, 0, 0])

        self.set_goal()

    def goToGoal(self):

        objectLocation = rospy.Subscriber("/gazebo/model_states", ModelStates, self.current_pose)
        goal_stat =   rospy.Subscriber("/move_base/status", GoalStatusArray, self.goal_status)

        if self.init_arm:
            self.arm_move()

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    
    def set_goal(self): 
        ################################################# sets the goal pose  ###############################################

        self.translationG = np.array([[0.355589882699,0.589154540941,0.000622224820887],
                  [0.56832946711,0.947066935758,0.000594851359371],
                  [0.715094586016,1.20544012708,0.000625826669336],
                  [0.84488231465,1.83842956679,0.000971872343016],
                  [0.97226731895,2.66532126274,0.000615122701821],
                  [1.33084944314,3.23785035243,0.000596003451584],
                  [2.0315973193,3.42024991393,0.00121327496004],
                  [2.52618618364,3.36677853412,0.000982675150928],
                  [3.3875974579,3.23216783153,0.000597231394861],
                  [3.42189393707,2.58356417629,0.00135884145739],
                  [3.26652914581,2.50571121007,0.000617821204471]])

        self.quaternionG = np.array([[0.00072271924718,-0.00129541031447,0.48852369384,0.872549368315],
                         [0.000700451319995,-0.00122983842053,0.497195328881,0.867637482941],
                         [0.000773128476447,-0.00127069620626,0.522683825411,0.852525311212],
                         [0.00150306657933,-0.00176022965198,0.649399971822,0.760443501505],
                         [0.00095453887819,-0.0011095503247,0.655891723286,0.754853565321],
                         [0.000683273592937,-0.00124227622298,0.478829537464,0.877906751278],
                         [-0.000627632822033,-0.00245223923504,0.108071515507,0.99413989968],
                         [-0.000132013652622,-0.00233663283052,-0.0563411119491,0.998408835009],
                         [-0.000960463026412,-0.00103996059978,-0.678740452197,0.734376875006],
                         [-0.00165905544165,-0.0023272285041,-0.804703350863,0.593670235613],
                         [-0.00146055847794,-0.000112261503438,-0.997539249583,0.070094933531]])

        self.init = True
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

        euler = tf.transformations.euler_from_quaternion(self.quaternion[7])
        self.ang = euler[2]

        if self.check_collision:
            self.collision_check()

        if self.init:
            self.control(self.translationG, self.quaternionG)
        
        # consider using state machines in the future

    def control(self, tG, qG):

        print "goal_id", self.goal_id
        position = tG[self.goal_id]
        orientation = qG[self.goal_id]

        """Send a cartesian goal to the action server."""
        #action_address = '/j2n6s300_driver/pose_action/tool_pose'
        client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        #print "hey"
        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header = Header(frame_id='map')
        goal.target_pose.pose.position = Point(x=position[0], y=position[1], z=position[2])
        goal.target_pose.pose.orientation = Quaternion(x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])

        client.send_goal(goal)
        self.goal_id = self.goal_id + 1
        time.sleep(7)
        
        if self.goal_id == 11:
            print "move arm now"
            self.init = False
            self.init_arm = True
            self.arm_move()
        
        print self.status
        
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
        #planning_scene.addCube("my_left_ground", 2, 0.0, 1.2, -1.0)
        planning_scene.addCube("my_left_ground", 1, 1.5, self.translation[2][0], self.translation[2][1], self.translation[2][2])
        planning_scene.addCube("my_right_ground", 2, 0.0, -1.2, -1.0)

        # TF joint names
        joint_names = ["torso_lift_joint", "shoulder_pan_joint",
                    "shoulder_lift_joint", "upperarm_roll_joint",
                    "elbow_flex_joint", "forearm_roll_joint",
                    "wrist_flex_joint", "wrist_roll_joint"]
        # Lists of joint angles in the same order as in joint_names
        disco_poses = [[0, 2.5, -0.1, 3.0, 1.5, 3.0, 1.0, 3.0]]

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

            if self.ret[i]:
                print "--------------- YES  ", self.ret[i], " --------------------"
            else:
                print "--------------- NO ", self.ret[i], " -------------------"
            
if __name__ == '__main__':
    rospy.init_node('goToGoal', anonymous=True)
    G = G2g()
    G.goToGoal()