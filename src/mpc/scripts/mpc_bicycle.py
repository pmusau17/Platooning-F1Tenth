#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import math
import numpy as np
from numpy import linalg as la
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import csv
import os
import rospkg 

# MPC Packages
# import relevant packages

import numpy as np
import sys

#CasADi is an open-source software tool for numerical optimization in general and optimal control 
# (i.e. optimization involving differential equations) in particular. 

from casadi import *
import do_mpc


class MPC: 

    # Constructor
    def __init__(self,n_steps=1,static_plot=False,plot=False,time_step=0.05):
        self.pose_msg = None
        self.goal_point = None

        # parameters for mpc, plotting fields used to disable plotting features during operation
        self.static_plot = static_plot
        self.plot = plot 
        self.time_step = time_step
        self.n_steps = n_steps

        # instatntiate subscribers
        self.drive_publish = rospy.Publisher('/vesc2/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=1)
        rospy.Subscriber('racecar2/odom', Odometry, self.pose_callback, queue_size=1)
        rospy.Subscriber('racecar2/goal_point', MarkerArray, self.goal_callback, queue_size=1)
        

        # define the model which will be used by thesimulator, estimator, and mpc controller
        self.model = self.template_model()
        

    
    def template_model(self):
        """Define the differential equations, parameters etc."""

        # Obtain an instance of the do-mpc model class
        # and select time discretization:

        model_type = 'continuous' # either 'discrete' or 'continuous'
        model = do_mpc.model.Model(model_type)

        # States struct (optimization variables):
        X_s = model.set_variable('_x',  'X_s') # Vehicle X position 
        Y_s = model.set_variable('_x',  'Y_s') # Vehicle Y position
        V_s = model.set_variable('_x',  'V_s') # Vehicle velocity
        Theta_s = model.set_variable('_x',  'Theta_s') # Vehicle yaw angle map frame

        # The control inputs are steering angle and throttle
        u_throttle = model.set_variable('_u',  'u_throttle')
        u_steering = model.set_variable('_u',  'u_steering') # in radians

        # System Identification Parameters 
        ca = 1.9569     # acceleration constant
        cm = 0.0342     # motor constant
        ch = -37.1967   # alleged hysteresis constant 
        lf = 0.225      # distance from car center of mass to front
        lr = 0.225      # distance from car center of mass to rear

        # Directly from the Differential equations. See paper
        model.set_rhs('X_s', V_s * cos(Theta_s))
        model.set_rhs('Y_s', V_s* sin(Theta_s))
        model.set_rhs('V_s', (-ca*V_s)+(ca*cm*(u_throttle-ch)))
        model.set_rhs('Theta_s', (V_s/(lf+lr))*tan(u_steering))

        # Build the model
        model.setup()
        return model 


    def template_mpc(self):
        """Define the mpc problem, with constraints, horizon estimates etc"""
        # Obtain an instance of the do-mpc MPC class
        # and initiate it with the model:

        mpc = do_mpc.controller.MPC(self.model)

        setup_mpc = {
            'n_horizon': 10,
            'n_robust': 0,                         # Robust horizon for robust scenario-tree MPC,
            'open_loop': 0,
            't_step': self.time_step,
            'state_discretization': 'collocation', # no other option at the moment
            'collocation_type': 'radau',           # no other option at the moment
            'collocation_deg': 2,
            'collocation_ni': 2,
            'store_full_solution': self.plot, # re
            #'nlpsol_opts': {'ipopt.linear_solver': 'MA27'} # highly recommended, Use MA27 linear solver in ipopt for faster calculations
        }
        
        mpc.set_param(**setup_mpc)

        # penalties or the cost function you are trying to optimize
        mterm = (self.model.x['X_s']-self.x_t)**2 + (self.model.x['Y_s']-self.y_t)**2 # stage cost
        lterm = (self.model.x['X_s']-self.x_t)**2 + (self.model.x['Y_s']-self.y_t)**2 # terminal cost

        # simple error without quadratic
        # mterm = (model.x['X_s']-self.x_t) + (model.x['Y_s']-self.y_t) # stage cost
        # lterm = (model.x['X_s']-self.x_t) + (model.x['Y_s']-self.y_t) # terminal cost

        mpc.set_objective(mterm=mterm, lterm=lterm)

        # input penalties
        mpc.set_rterm(u_throttle = 10, u_steering = 10)

        # set constraints
        # start with these are there any others????????
        # upper and lower bounds of the control input
        mpc.bounds['lower','_u','u_throttle'] = 0.3
        mpc.bounds['upper','_u','u_throttle'] = 5.0

        mpc.bounds['lower','_u','u_steering'] = -0.523599
        mpc.bounds['upper','_u','u_steering'] = 0.523599

        mpc.setup()
            
        return mpc


    def template_estimator(self): 
        """configures the estimator (MHE / EKF / state-feedback)"""
        return do_mpc.estimator.StateFeedback(self.model)

    def template_simulator(self):
        """configures the DAE/ODE/discrete simulator"""
        # Obtain an instance of the do-mpc simulator class
        # and initiate it with the model:
        simulator = do_mpc.simulator.Simulator(self.model)

        # Setup the parameters for the simulator
        params_simulator = {
        'integration_tool': 'cvodes',
        'abstol': 1e-10,
        'reltol': 1e-10,
        't_step': self.time_step
        }

        # Set parameter(s):
        simulator.set_param(**params_simulator)
        
        # Setup simulator:
        simulator.setup()

        return simulator


    def run_closedloop(self):

        self.mpc = self.template_mpc()

        # if(self.plot):
        #     self.estimator = self.template_estimator()
        #     self.simulator = self.template_simulator()

        self.mpc.x0 = self.x0
        print(self.x0)

        # if(self.plot):
        #     self.simulator.x0 = self.x0
        #     self.estimator.x0 = self.x0
        self.mpc.set_initial_guess()

        # if(self.plot):
        #     mpc_graphics = do_mpc.graphics.Graphics(self.mpc.data)
        #     sim_graphics = do_mpc.graphics.Graphics(self.simulator.data)

        #     inputs = []
        #     states = []
        #     for k in range(self.n_steps):
        #         u0 = self.mpc.make_step(self.x0)
        #         inputs.append(u0)
        #         y_next = self.simulator.make_step(u0)
        #         x0 = self.estimator.make_step(y_next)


        #     fig, ax = plt.subplots(6, sharex=True, figsize=(16,9))
        #     fig.align_ylabels()

        #     for g in [sim_graphics,mpc_graphics]:
        #         # Plot the state on axis 1 to 4:
        #         g.add_line(var_type='_x', var_name='X_s', axis=ax[0], color='#1f77b4')
        #         g.add_line(var_type='_x', var_name='Y_s', axis=ax[1], color='#1f77b4')
        #         g.add_line(var_type='_x', var_name='V_s', axis=ax[2], color='#1f77b4')
        #         g.add_line(var_type='_x', var_name='Theta_s', axis=ax[3], color='#1f77b4')

        #         # Plot the throttle control input on axis 5:
        #         g.add_line(var_type='_u', var_name='u_throttle', axis=ax[4], color='#1f77b4')
        #         # Plot the steering control input on axis 6:
        #         g.add_line(var_type='_u', var_name='u_steering', axis=ax[5], color='#1f77b4')

        #     ax[0].set_ylabel(r'$X_s$')
        #     ax[1].set_ylabel(r'$Y_s$')
        #     ax[2].set_ylabel(r'$V_s$')
        #     ax[3].set_ylabel(r'$\theta_s$')
        #     ax[4].set_ylabel(r'$u_{throttle}$')
        #     ax[5].set_ylabel(r'$u_{steering}$')
        #     ax[5].set_xlabel(r'$t (seconds)$')

        #     if(self.static_plot):
        #         sim_graphics.plot_results()
        #         sim_graphics.reset_axes()
        #         mpc_graphics.plot_results()
        #         plt.show(block=True)
        #     else:
        #         # nested function for convenience:
        #         def update(t_ind):
        #             sim_graphics.plot_results(t_ind)
        #             mpc_graphics.plot_predictions(t_ind)
        #             mpc_graphics.reset_axes()

        #         anim = FuncAnimation(fig, update, frames=self.n_steps, repeat=False)
        #         gif_writer = ImageMagickWriter(fps=10)
        #         anim.save('anim_bicycle_model.gif', writer=gif_writer)

        # else:
        rospy.logwarn("Here")
        u0 = self.mpc.make_step(self.x0)
        print(u0)

        return u0




    def goal_callback(self,goal_point):
        self.goal_point = goal_point
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        # make sure things are instantiated properly
        if(self.goal_point and self.pose_msg):
            point = self.goal_point.markers[0]
            self.x_t,self.y_t = point.pose.position.x,point.pose.position.y
            u0 = self.run_closedloop()
            print(u0)
            drive_msg.drive.steering_angle = float(u0[1])
            drive_msg.drive.speed = 2*float(u0[0])
            self.drive_publish.publish(drive_msg)

    def pose_callback(self,pose_msg):
        self.pose_msg = pose_msg

        # compute necessary state information
        quaternion = (pose_msg.pose.pose.orientation.x,pose_msg.pose.pose.orientation.y,
                        pose_msg.pose.pose.orientation.z,pose_msg.pose.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        yaw = np.double(euler[2])

        
        speed = la.norm(np.array([pose_msg.twist.twist.linear.x,pose_msg.twist.twist.linear.y,pose_msg.twist.twist.linear.z]))

        x0 = np.asarray([pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y, yaw,speed])
        self.x0 = x0


if __name__ == "__main__":
    rospy.init_node('mpc_node_bicycle')
    rospy.sleep(5)
    mpc = MPC()
    rospy.spin()