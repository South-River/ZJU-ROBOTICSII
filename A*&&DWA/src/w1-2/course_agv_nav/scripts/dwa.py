#!/usr/bin/env python
import math
import numpy as np

class DWAPlanner():
    def __init__(self):
        # DWA params
        self.min_v_speed = -0.2
        self.max_v_speed = 0.2  # [m/s]
        self.min_w_speed = -0.3
        self.max_w_speed = 0.3
        self.predict_time = 2  # [s]
        self.va = 0.2
        self.wa = 0.06
        self.v_step = 0.04
        self.w_step = 0.1
        self.radius = 0.15
        self.delta_t = 0.2
        self.alpha = 4         #goal
        self.beta = 1           #velocity
        self.gamma = 10   #ob
        self.norm = self.alpha + self.beta + self.gamma
        self.avoid_size = 0.05

    def plan(self, x, goal, ob):
        u = x[3:5]
        u = self.dwa_search(x,u,goal,ob)
        return u

    def dwa_search(self, X, u, goal, obstacles):
        vw=self.calculate_vw_range(X)
        min_score = 1e9
        #print('vw[0],vw[1]:', vw[0], vw[1])
        #print('vw[2],vw[3]:', vw[2], vw[3])      
        for v in np.arange(vw[0], vw[1], self.v_step):    
            for w in np.arange(vw[2], vw[3], self.w_step):                 
                traj = self.predict_trajectory(X,[v,w])
                if self.check_collision(traj,obstacles) == True:
                    print('check_collision == True')
                    continue
                goal_score = self.heading_func(traj, goal)
                vel_score = self.velocity_func(traj)
                obs_score = self.clearance_func(traj,obstacles)
                score = (self.alpha * goal_score + self.beta * vel_score + self.gamma * obs_score) / self.norm
                if score <= min_score:                   
                    min_score = score
                    u = np.array([v,w])
        return u

    def calculate_vw_range(self, X):
        v_min = X[3] - self.va * self.predict_time          
        v_max = X[3] + self.va * self.predict_time         
        w_min = X[4] - self.wa * self.predict_time          
        w_max = X[4] + self.wa * self.predict_time         

        VW = [max(self.min_v_speed, v_min), min(self.max_v_speed, v_max), max(self.min_w_speed, w_min), min(self.max_w_speed, w_max)]
        return VW

    def Motion(self,X,u):
        X[0]+=u[0] * self.delta_t * math.cos(X[2])           
        X[1]+=u[0] * self.delta_t * math.sin(X[2])           
        X[2]+=u[1] * self.delta_t                     
        X[3]=u[0]                         
        X[4]=u[1] 
        return X

    def predict_trajectory(self, x_init, u):
        Traj = np.array(x_init)
        Xnew = np.array(x_init)        
        time=0
        while time <= self.predict_time:
            Xnew=self.Motion(Xnew,u)
            time += self.delta_t
            Traj = np.vstack((Traj,Xnew))
        return Traj

    def check_collision(self, trajectory, ob):
        for i in range(len(trajectory)):
            for j in range(len(ob)):
                dist = math.hypot(trajectory[i,0] - ob[j,0], trajectory[i,1] - ob[j,1]) 
                if dist <= self.radius + self.avoid_size:
                    print('dist:', dist)
                    return True
        return False

    def heading_func(self, trajectory, goal):
        return math.hypot(trajectory[-1,0] - goal[0], trajectory[-1,1] - goal[1])
        
    def clearance_func(self, trajectory, ob):
        min_dist = 1e9
        for i in range(len(trajectory)):
            for j in range(len(ob)):
                dist = math.hypot(trajectory[i,0] - ob[j,0], trajectory[i,1] - ob[j,1])
                if dist < self.radius:
                    return float('Inf')

                if dist < min_dist:
                    min_dist = dist

        return 1/min_dist

    def velocity_func(self, trajectory):
        return self.max_v_speed - trajectory[-1,3]