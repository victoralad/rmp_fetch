#!/usr/bin/env python

# wave.py: "Wave" the fetch gripper
import rospy
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

# Note: fetch_moveit_config move_group.launch must be running
# Safety!: Do NOT run this script near people or objects.
# Safety!: There is NO perception.
#          The ONLY objects the collision detection software is aware
#          of are itself & the floor.
if __name__ == '__main__':
    rospy.init_node("hi")

    # Create move group interface for a fetch robot
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

    # This is the wrist link not the gripper itself
    gripper_frame = 'wrist_roll_link'
    # Position and rotation of two "wave end poses"
    gripper_poses = [Pose(Point(0.042, 0.384, 1.826),
                          Quaternion(0.173, -0.693, -0.242, 0.657)),
                     Pose(Point(0.047, 0.545, 1.822),
                          Quaternion(-0.274, -0.701, 0.173, 0.635))]

    # Construct a "pose_stamped" message as required by moveToPose
    gripper_pose_stamped = PoseStamped()
    gripper_pose_stamped.header.frame_id = 'base_link'

    while not rospy.is_shutdown():
        for pose in gripper_poses:
            # Finish building the Pose_stamped message
            # If the message stamp is not current it could be ignored
            gripper_pose_stamped.header.stamp = rospy.Time.now()
            # Set the message pose
            gripper_pose_stamped.pose = pose

            # Move gripper frame to the pose specified
            move_group.moveToPose(gripper_pose_stamped, gripper_frame)
            result = move_group.get_move_action().get_result()

            if result:
                # Checking the MoveItErrorCode
                if result.error_code.val == MoveItErrorCodes.SUCCESS:
                    rospy.loginfo("Hello there!")
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

      

       ob0 = fcl.Box(0.3, 1, 0.8)
        tr0 = fcl.Transform(self.quaternion[0], self.translation[0])
        self.obj[0] = fcl.CollisionObject(ob0, tr0)

        request = fcl.CollisionRequest()
        result = fcl.CollisionResult()
        
        ret = fcl.collide(oR, self.obj[0], request, result)

        # ret = fcl.continuousCollide(oR, tR, o_wb, t_wb, request, result)
        if ret:
            print "--------------- YES  ", ret, " --------------------"
        else:
            print "--------------- NO ", ret, " -------------------"
        ####################################################################################


        ob1 = fcl.Sphere(0.5)
        tr1 = fcl.Transform(self.quaternion[1], self.translation[1])
        self.obj[1] = fcl.CollisionObject(ob1, tr1)

        request = fcl.CollisionRequest()
        result = fcl.CollisionResult()
        
        self.ret = fcl.collide(oR, self.obj[1], request, result)

        # ret = fcl.continuousCollide(oR, tR, o_wb, t_wb, request, result)
        if ret:
            print "--------------- YES  ", ret, " --------------------"
        else:
            print "--------------- NO ", ret, " -------------------"
        #####################################################################################

        ob2 = fcl.Cylinder(0.5, 1)
        tr2 = fcl.Transform(self.quaternion[2], self.translation[2])
        self.obj[2] = fcl.CollisionObject(ob2, tr2)

        request = fcl.CollisionRequest()
        result = fcl.CollisionResult()
        
        self.ret = fcl.collide(oR, self.obj[2], request, result)

        # ret = fcl.continuousCollide(oR, tR, o_wb, t_wb, request, result)
        if ret:
            print "--------------- YES  ", ret, " --------------------"
        else:
            print "--------------- NO ", ret, " -------------------"
        ####################################################################################

        ob3 = fcl.Box(0.5, 1.4, 0.8)
        tr3 = fcl.Transform(self.quaternion[3], self.translation[3])
        self.obj[3] = fcl.CollisionObject(ob3, tr3)

        request = fcl.CollisionRequest()
        result = fcl.CollisionResult()
        
        self.ret = fcl.collide(oR, self.obj[3], request, result)

        # ret = fcl.continuousCollide(oR, tR, o_wb, t_wb, request, result)
        if ret:
            print "--------------- YES  ", ret, " --------------------"
        else:
            print "--------------- NO ", ret, " -------------------"
        ####################################################################################

        ob4 = fcl.Box(2, 0.2, 1)
        tr4 = fcl.Transform(self.quaternion[4], self.translation[4])
        self.obj[4] = fcl.CollisionObject(ob4, tr4)

        request = fcl.CollisionRequest()
        result = fcl.CollisionResult()
        
        self.ret = fcl.collide(oR, self.obj[4], request, result)

        # ret = fcl.continuousCollide(oR, tR, o_wb, t_wb, request, result)
        if ret:
            print "--------------- YES  ", ret, " --------------------"
        else:
            print "--------------- NO ", ret, " -------------------"
        ####################################################################################
        
        ob5 = fcl.Box(1, 5, 1)
        tr5 = fcl.Transform(self.quaternion[5], self.translation[5])
        self.obj[5] = fcl.CollisionObject(ob5, tr5)

        request = fcl.CollisionRequest()
        result = fcl.CollisionResult()
        
        self.ret = fcl.collide(oR, self.obj[5], request, result)

        # ret = fcl.continuousCollide(oR, tR, o_wb, t_wb, request, result)
        if ret:
            print "--------------- YES  ", ret, " --------------------"
        else:
            print "--------------- NO ", ret, " -------------------"
        ####################################################################################

        ob6 = fcl.Cylinder(0.5, 1)
        tr6 = fcl.Transform(self.quaternion[6], self.translation[6])
        self.obj[6] = fcl.CollisionObject(ob6, tr6)

        request = fcl.CollisionRequest()
        result = fcl.CollisionResult()
        
        self.ret = fcl.collide(oR, self.obj[6], request, result)

        # ret = fcl.continuousCollide(oR, tR, o_wb, t_wb, request, result)
        if ret:
            print "--------------- YES  ", ret, " --------------------"
        else:
            print "--------------- NO ", ret, " -------------------"
        ####################################################################################

        ob7 = fcl.Box(5, 1, 1)
        tr7 = fcl.Transform(self.quaternion[7], self.translation[7])
        self.obj[7] = fcl.CollisionObject(ob7, tr7)

        request = fcl.CollisionRequest()
        result = fcl.CollisionResult()
        
        self.ret = fcl.collide(oR, self.obj[7], request, result)

        # ret = fcl.continuousCollide(oR, tR, o_wb, t_wb, request, result)
        if ret:
            print "--------------- YES  ", ret, " --------------------"
        else:
            print "--------------- NO ", ret, " -------------------"
        ####################################################################################

       