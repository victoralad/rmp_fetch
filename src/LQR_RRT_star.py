#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sun Apr 28 15:18:01 2019

@author: dkilanga
"""

import numpy as np
import control as ctrl
import copy
from gazebo_msgs.msg import ModelStates
import fcl
import rospy
import sys


class LQR_RRT_star:
    
    def __init__(self, start_state, goal_state):
        rospy.init_node('LQR_RRT_star', anonymous = True)
        self.object_location = rospy.Subscriber("/gazebo/model_states", ModelStates, self.current_pose)
        
        self.start_node = Node(start_state)
        self.start_node.set_position(0)
        self.start_state = start_state
        self.goal_state = goal_state
        self.tree_vertices = []
        self.tree_vertices.append(self.start_state)
        self.tree_nodes = []
        self.tree_nodes.append(self.start_node)
        self.N = 1000
        self.dt = 0.1
        self.Q = np.eye(3)
        self.R = np.eye(2)
        self.epsilon = np.array([0.2, 0.2, np.deg2rad(10)])
        self.step = 1.5
        
        self.quaternion = np.zeros((8,4))
        self.translation = np.zeros((8,3))
        self.ret = np.zeros(7)
        self.obj = [fcl.CollisionObject(fcl.Sphere(0.5), fcl.Transform()) for i in range(7)]
        for i in range(8):
            self.quaternion[i] = np.array([0,0,0,0])
            self.translation[i] = np.array([0,0,0])
            
        my_path = self.operate()
        print "The found path is: ", my_path
    
        
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print "Shutting down LQR-RRT* node"
            
    def current_pose(self, data):
        
        world = ['blue_box', 'white_sphere', 'red_cylinder', 'green_box', 'turquoise_box', 'blue_cylinder', 'white_box', 'fetch']
        dx, dy, dz, dxo, dyo, dzo, dwo = np.zeros((7, len(world)))
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
            
    def sample(self):
        x_rand = np.zeros(3)
        x_rand[0] = np.random.uniform(-20,20)
        x_rand[1] = np.random.uniform(-20,20)
        x_rand[2] = np.random.randint(360)
        x_rand[2] = np.arctan2(np.sin(np.deg2rad(x_rand[2])), np.cos(np.deg2rad(x_rand[2])))
        
        return x_rand
    
    def LQR_Nearest(self, x_rand):
        try:
            A, B = self.linearize_system(x_rand, np.zeros(2), "LQR_Nearest")
            K, S, _ = ctrl.lqr(A,B,self.Q,self.R)
            print "K is: ", K
            print "S is: ", S
            vertices_matrix = np.array(self.tree_vertices)
            diff_matrix = np.subtract(vertices_matrix, x_rand)
            nearest_indx = np.argmin(np.diag((diff_matrix.dot(S)).dot(diff_matrix.T)))
            x_nearest = vertices_matrix[nearest_indx,:]
            x_nearest_node = self.tree_nodes[nearest_indx]
            return x_nearest, x_nearest_node
        except ValueError:
            return None, None
        
    def linearize_system(self, x, u, where):
        A = np.zeros((3,3))
        B = np.zeros((3,2))
        # print where
        # print "shape x: ", x.shape
        # print "shape u: ", u.shape
        A = np.array([[1,0,-u[0]*self.dt*np.sin(x[2])],[0,1,u[0]*self.dt*np.cos(x[2])],[0,0,1]])
        B = np.array([[self.dt*np.cos(x[2]),0],[self.dt*np.sin(x[2]),0],[0,self.dt]])
        # print "A is: ", A
        return A, B
    
    def LQR_Steer(self, x_nearest, x_rand, get_cost=False):
        x = np.zeros(3)
        x[0] = x_nearest[0]
        x[1] = x_nearest[1]
        x[2] = x_nearest[2]
        u = np.zeros(2)
        nominal_path = self.local_trajectory(x_nearest, self.step, x_rand, get_cost)
        path = []
        path.append(copy.deepcopy(x))
        Wx = np.eye(3)
        Wu = np.eye(2)
        gains, nominal_controls, nominal_states = self.compute_controller_gain(Wx, Wu, nominal_path)
        for i in range(gains.shape[2]):
            error = x - nominal_states[:,0,i]
            delta_u = -gains[:,:,i].dot(error)
            control = nominal_controls[:,0,i] + delta_u
            x[0]+=control[0]*self.dt*np.cos(x[2])
            x[1]+=control[0]*self.dt*np.sin(x[2])
            x[2]+=control[1]*self.dt
            path.append(copy.deepcopy(x))
        x_new = path[-1]
        array_path = np.array(path)
        path_x = array_path[:,0]
        path_y = array_path[:,1]
        dx = np.diff(path_x)
        dy = np.diff(path_y)
        cost = np.sum(np.sqrt(np.add(np.power(dx,2), np.power(dy,2))))
        
        return x_new, cost, path
    
    def LQR_Near(self, x_new):
        A, B = self.linearize_system(x_new, np.zeros(2), "LQR_Near")
        # print "A is: ", A
        # print "B is: ", B
        try:
            K, S, _ = ctrl.lqr(A,B,self.Q,self.R)
            # print "K is: ", K.shape
            # print "S is: ", S.shape
            vertices_matrix = np.array(self.tree_vertices)
            diff_matrix = np.subtract(vertices_matrix, x_new)
            num_nodes = float(len(self.tree_vertices))
            threshold = 50.0*(np.log(num_nodes)/num_nodes)**(1/3.0)
            near_indx = np.where(np.diag((diff_matrix.dot(S)).dot(diff_matrix.T)) <= threshold)
    #        X_near = list(vertices_matrix[near_indx,:])
            print "Near Index: ", near_indx[0]
            X_near = list(np.array(self.tree_nodes)[near_indx[0]])
            
            return X_near
        except ValueError:
            return []
    
    def choose_parent(self, X_near, x_new):
        min_cost = np.inf
        path_min = []
        if not X_near:
            
            return None, [], None
        else:
            for x_near in X_near:
                _, segment_cost, path = self.LQR_Steer(x_near.get_state(), x_new.get_state(), get_cost=True)
                if (x_near.get_cost() + segment_cost) < min_cost:
                    min_cost = x_near.get_cost() + segment_cost
                    x_min = x_near
                    path_min = copy.copy(path)
            
            return x_min, path_min, min_cost
    
    def rewire(self, X_near, x_new):
        for x_near in X_near:
            _, segment_cost, path = self.LQR_Steer(x_new.get_state(), x_near.get_state(), get_cost=True)
            if (x_new.get_cost() + segment_cost) < x_near.get_cost():
                if self.collision_free(path):
                    x_near.set_parent(len(self.tree_vertices)-1)
                    
    def collision_free(self, path):
        for i in range(len(path)):
            way_point = path[i]
            robot = fcl.Cylinder(0.4,1)
            tR = fcl.Transform(way_point)
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
            
            for j in range(7):
                self.ret[j] = fcl.collide(oR, self.obj[j], request, result)
                if self.ret[j]:
                    return False
        
        return True    
            
    
    def operate(self):
        path = []
        for i in range(self.N):
            print "A new point was sampled"
            x_rand = self.sample()
            x_nearest, x_nearest_node = self.LQR_Nearest(x_rand)
            if x_nearest != None:
                x_new, _, _ = self.LQR_Steer(x_nearest_node.get_state(), x_rand)
                x_new_node = Node(x_new)
                X_near = self.LQR_Near(x_new)
                x_min_node, path_min, min_cost =  self.choose_parent(X_near, x_new_node)
                if x_min_node != None:
                    if self.collision_free(path_min):
                        x_new_node.increment_cost(min_cost)
                        x_new_node.set_parent(x_min_node.get_position())
                        x_new_node.set_position(len(self.tree_nodes))
                        self.tree_nodes.append(x_new_node)
                        self.tree_vertices.append(x_new)
                        self.rewire(X_near, x_new_node)
                    if (np.abs(x_new_node.get_state()[0:2] - self.goal_state[0:2]) <= self.epsilon[0:2]).all():
                        path.append(x_new)
                        next_parent = x_new_node.get_parent()
                        while next_parent != None:
                            node = self.tree_nodes[next_parent]
                            path.append(node.get_state())
                            next_parent = node.get_parent()
                        return path[::-1]
        print "No Path was found"    
        return None
        
    def local_trajectory(self, x_nearest, step, x_rand, get_cost):
        angular_velocity = 0.2
        linear_velocity = 0.2
        x = np.zeros(3)
        u = np.zeros(2)
        path = []
        x[0] = x_nearest[0]
        x[1] = x_nearest[1]
        x[2] = x_nearest[2]
        dx = x_rand[0] - x_nearest[0]
        dy = x_rand[1] - x_nearest[1]
        distance = np.sqrt(dx**2 + dy**2)
        desired_angle = np.arctan2(dy,dx)
        x_destination = np.zeros(3)
        x_destination[0] = x_nearest[0] + step*np.cos(desired_angle)
        x_destination[1] = x_nearest[1] + step*np.sin(desired_angle)
        x_destination[2] = desired_angle
        delta_angle_start = desired_angle - x_nearest[2]
        delta_angle_start = np.arctan2(np.sin(delta_angle_start), np.cos(delta_angle_start))
        if get_cost:
            nsteps_translation = int(round(distance/(linear_velocity*self.dt)))
        else:
            nsteps_translation = int(round(step/(linear_velocity*self.dt)))
        nsteps_rotation_start = int(round(abs(delta_angle_start)/(angular_velocity * self.dt)))
        path.append((copy.deepcopy(x),copy.deepcopy(u)))
        
        if (np.abs(x_destination[0:2] - x_nearest[0:2]) > self.epsilon[0:2]).all():
            for i in range(nsteps_rotation_start):
                u[0] = 0
                u[1] = angular_velocity * delta_angle_start
                x[0] += u[0] * np.cos(desired_angle) * self.dt
                x[1] += u[0] * np.sin(desired_angle) * self.dt
                x[2] += u[1] * self.dt 
                path.append((copy.deepcopy(x),copy.deepcopy(u)))
                
            for i in range(nsteps_translation):
                u[0] = linear_velocity
                u[1] = 0
                x[0] += u[0] * np.cos(desired_angle) * self.dt
                x[1] += u[0] * np.sin(desired_angle) * self.dt
                x[2] += u[1] * self.dt
                path.append((copy.deepcopy(x),copy.copy(u)))
        
        return path
    
    def compute_controller_gain(self, Wx, Wu, path):
        S = Wx
        path_length = len(path)
        gains = np.empty((2,3,path_length))
        nominal_controls = np.empty((2,1,path_length))
        nominal_states = np.empty((3,1,path_length))
        j = path_length-1
        
        for i in range(path_length-1,-1,-1):
            x, u = path[i]
            A, B = self.linearize_system(x,u, "compute_controller_gain")
            L = (np.linalg.inv((B.T).dot(S.dot(B)) + Wu)).dot((B.T).dot(S.dot(A)))
            S = Wx + (A.T).dot(S.dot(A)) - (A.T).dot(S.dot(B.dot(L)))
            if i == j:
                gains[:,:,j] = copy.deepcopy(L)
                nominal_controls[:,0,j] = copy.deepcopy(u)
                nominal_states[:,0,j] = copy.deepcopy(x)
                j -= 1

        return gains, nominal_controls, nominal_states
            

class Node:
    
    def __init__(self, state):
        self.state = state
        self.cost = 0.0
        self.parent = None
        self.position = None
        
    def increment_cost(self, cost_to_add):
        self.cost+=cost_to_add
        
    def set_parent(self, parent_index):
        self.parent = parent_index
        
    def set_position(self, position):
        self.position = position
        
    def get_state(self):
        return self.state
    
    def get_cost(self):
        return self.cost
    
    def get_parent(self):
        return self.parent
    
    def get_position(self):
        return self.position

def main(args):
    '''Initialize and cleanup ROS node'''
    
    try:
        lqr_rrt_star = LQR_RRT_star(np.array([0,0,0]), np.array([10,10,0]))
    except rospy.ROSInterruptException:
        pass
   
    
if __name__ == '__main__':
    main(sys.argv)            

        