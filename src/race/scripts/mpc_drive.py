#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry
import sys
import os
import tf


import math
import numpy as np
from cvxpy import *
import scipy as sp
from scipy import sparse
import osqp



def pose_callback(pose_msg):

    quaternion = np.array([pose_msg.pose.pose.orientation.x,
                           pose_msg.pose.pose.orientation.y,
                           pose_msg.pose.pose.orientation.z,
                           pose_msg.pose.pose.orientation.w])

    euler = tf.transformations.euler_from_quaternion(quaternion)
    position = [pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y]

    quaternion_z = pose_msg.pose.pose.orientation.z
    quaternion_w = pose_msg.pose.pose.orientation.w
    head_angle = math.atan2(2 * (quaternion_z * quaternion_w), 1 - 2 * (quaternion_z * quaternion_z))

    mpc_drive(position[0], position[1], head_angle,  1.398147, 1.222223)
    

   

# Pass relevant information to publisher
def mpc_drive(posx, posy, head_angle, tarx, tary):

    m_v = 0 # temporary
    m_psi = 0 # temporary
    m_beta = 1 # temporary
    m_delta_t = 0.1 # temporary
    m_l = 0.3 # temporary


    drive_publish = rospy.Publisher('/vesc/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=1)

    Ad = sparse.csc_matrix([
        [1, 0, -m_v * math.sin(m_psi + m_beta) * m_delta_t],
        [0, 1, m_v * math.cos(m_psi + m_beta) * m_delta_t],
        [0, 0, 1]
    ])

    Bd = sparse.csc_matrix([
        [math.cos(m_psi + m_beta) * m_delta_t, -m_v * math.sin(m_psi + m_beta) * m_delta_t],
        [math.sin(m_psi + m_beta) * m_delta_t, m_v * math.cos(m_psi + m_beta) * m_delta_t],
        [m_delta_t / m_l * np.sin(m_beta), m_v * m_delta_t / m_l * np.cos(m_beta)]
    ])

    [nx, nu] = Bd.shape

    # constraints
    umin = np.array([0.0, -.4])
    umax = np.array([7.0, .4])
    xmin = np.array([0., 0., 0.])
    xmax = np.array([np.inf, np.inf, np.inf])

    # objective function
    Q = sparse.diags([1., 1.])
    QN = Q
    R = 0.1 * sparse.eye(2)

    # Initial and reference states
    x0 = np.zeros(3)
    u0 = np.zeros(2)
    xr0 = np.array([1., 1.])
    # x_r = [xr0, xr0, xr0]

    # Prediction horizon
    N = 5

    P = sparse.block_diag([sparse.kron(sparse.eye(N), Q), QN, sparse.kron(sparse.eye(N), R)], format='csc')

    q = np.hstack([np.kron(np.ones(N), -Q.dot(xr0)), -QN.dot(xr0), np.zeros(N * nu)])

    Ax = sparse.kron(sparse.eye(N + 1), -sparse.eye(nx)) + sparse.kron(sparse.eye(N + 1, k=-1), Ad)
    Bu = sparse.kron(sparse.vstack([sparse.csc_matrix((1, N)), sparse.eye(N)]), Bd)
    Aeq = sparse.hstack([Ax, Bu])
    leq = np.hstack([-x0, np.zeros(N * nx)])
    ueq = leq
    # - input and state constraints
    Aineq = sparse.eye((N + 1) * nx + N * nu)
    lineq = np.hstack([np.kron(np.ones(N + 1), xmin), np.kron(np.ones(N), umin)])
    uineq = np.hstack([np.kron(np.ones(N + 1), xmax), np.kron(np.ones(N), umax)])

    # - OSQP constraints
    A = sparse.vstack([Aeq, Aineq], format='csc')
    l = np.hstack([leq, lineq])
    u = np.hstack([ueq, uineq])

    # Create an OSQP object
    prob = osqp.OSQP()

    # Setup workspace
    prob.setup(P, q, A, l, u, warm_start=True)

    res = prob.solve()

    print(res.x[-N * nu:-(N - 1) * nu])


    drive_msg = AckermannDriveStamped()
    drive_msg.header.stamp = rospy.Time.now()
    drive_msg.header.frame_id = "base_link"
    drive_msg.drive.steering_angle = res.x[-N * nu:-(N - 1) * nu][0]
    drive_msg.drive.speed = res.x[-N * nu:-(N - 1) * nu][1]
    drive_publish.publish(drive_msg)




if __name__ == '__main__':
    rospy.init_node('mpc_node')
    rospy.Subscriber('racecar/odom', Odometry, pose_callback, queue_size=1)
    rospy.spin()
    
    
    
    
    
    
