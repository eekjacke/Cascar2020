#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Oct 15 15:56:33 2020

@author: gustav
"""
from vehiclecontrol import ControllerBase
import numpy as np
import casadi


class ModelPredictiveController(ControllerBase):
    def __init__(self, controller_params, path=None, goal_tol=1, dt=0.1):
        super().__init__()
        
        self.plan = path
        self.gamma_d = controller_params['gamma_d']
        self.gamma_theta = controller_params['gamma_theta']
        self.gamma_u = controller_params['gamma_u']
        self.L = controller_params['L']
        self.steer_limit = controller_params['steer_limit']

        self.sample_rate = dt
        self.prediction_horizon = controller_params['h_p']
        self.N = int(self.prediction_horizon / dt)
        
        self.goal_tol = goal_tol
        self.d = []
        self.s0 = 0
        self.optimizer = self.construct_problem()

    def heading_error(self, theta, s):
        """Compute theta error
        Inputs
            theta - current heading angle
            s - projection point on path
            
        Outputs
            theta_e - heading error angle
        """
        
        ### YOUR CODE HERE ###
        # Use your code from state-feedback controller in the basic exercise
        hs = self.plan.heading(s)[0]
        h=[np.cos(theta),np.sin(theta)]
        
        cos_theta_e=np.dot(hs,h)
        sin_theta_e=np.cross(np.array([hs[0], hs[1], 0]),np.array([h[0],h[1], 0]))
        
        theta_e = np.arctan2(sin_theta_e[2],cos_theta_e)
        return theta_e
        
    def construct_problem(self):
        """Formulate optimal control problem"""
        
        dt = self.sample_rate
        
        # Create an casadi.Opti instance.
        opti = casadi.Opti('conic')
        
        d0 = opti.parameter()
        th0 = opti.parameter()
        v = opti.parameter()
        curvature = opti.parameter(self.N)
        
        X = opti.variable(2, self.N + 1)
        proj_error = X[0, :]
        head_error = X[1, :]
        
        # Control variable (steering angle)
        Delta = opti.variable(self.N)

        # Goal function we wish to minimize   
        ### YOUR CODE HERE ###
        # Use the casadi.sumsqr function to compute sum-of-squares
        d_sum=casadi.sumsqr(proj_error)
        th_sum=casadi.sumsqr(head_error)
        u_sum=casadi.sumsqr(Delta)
        
        J = self.gamma_d*d_sum+self.gamma_theta*th_sum+self.gamma_u*u_sum

        opti.minimize(J)
         
        # Simulate the system forwards using RK4 and the implemented error model.
        for k in range(self.N):
            k1 = self.error_model(X[:, k], v, Delta[k], curvature[k])
            k2 = self.error_model(X[:, k] + dt / 2 * k1, v, Delta[k], curvature[k])
            k3 = self.error_model(X[:, k] + dt / 2 * k2, v, Delta[k], curvature[k])
            k4 = self.error_model(X[:, k] + dt * k3, v, Delta[k], curvature[k])
            x_next = X[:, k] + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
            opti.subject_to(X[:, k + 1] == x_next)
        
        # Problem constraints.
        opti.subject_to(proj_error[0] == d0)
        opti.subject_to(head_error[0] == th0)
        opti.subject_to(opti.bounded(-self.steer_limit, Delta, self.steer_limit))
        
        # The cost function is quadratic and the problem is linear by design,
        # this is utilized when choosing solver.
        # Other possible solvers provided by CasADi include: 'qpoases' and 'ipopt'...

        opts_dict = {
            "print_iter": False,
            "print_time": 0,
            #"max_iter": 100
        }
        
        opti.solver('qrqp', opts_dict)
        
        return opti.to_function('f', [d0, th0, v, curvature], [Delta])
        
    def error_model(self, w, v, delta, curvature):
        """Error model describing how the distance and heading error evolve for a certain input
            
        Input:
            w = (d, theta_e)
            v - velocity
            delta - input, steering angle
            curvature - current curvature
            
        Output:
            Casadi vector of Time derivative of d and theta_e
        """

        ### YOUR CODE HERE ###
        u=delta/self.L
        # d=w[0]
        theta_e=w[1]
        
        d_dot = v*theta_e
        theta_e_dot = v*u - v*curvature
        
        return casadi.vertcat(d_dot, theta_e_dot)

    def u(self, w):
        p_car = w[0:2]
        theta = w[2]
        v = w[3]
        
        # Compute d and theta_e errors as in the basic exercise state-feedback controller
        # YOUR CODE HERE
        ds=0.1 #Step used to expand search space
        s_lim=10 #Number of expansions of the search
        
        s,d = self.plan.project(p_car,self.s0,ds, s_lim)
        
        theta_e = self.heading_error(theta,s)
        
        
        self.s0 = s
        
        s_i = s  # Position for start of prediction
        
        # Solve optimization problem over the prediction-horizon
        s_horizon = np.linspace(s_i, s_i + self.N * v * self.sample_rate, self.N)        
        # YOUR CODE HERE, call self.optimizer() with proper arguments
        curve=self.plan.c(s_horizon)

        Delta = self.optimizer(d,theta_e,v,curve)
        # Collect the controller output
        delta = Delta[0]  # YOUR CODE HERE
        acc = 0        
        self.d.append(d)

        return np.array([delta, acc])
    
    def run(self, w):
        p_goal = self.plan.path[-1, :]
        p_car = w[0:2]
        dp = p_car - p_goal
        dist = np.sqrt(dp.dot(dp))
        if dist < self.goal_tol:
            return False
        else:
            return True

