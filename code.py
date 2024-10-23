#!/usr/bin/env python3
from math import pi, sqrt, atan2, cos, sin, asin, acos
import math
from turtle import position
import numpy as np
from numpy import NaN
import rospy
import tf
from std_msgs.msg import Empty, Float32
from nav_msgs.msg import Odometry
from mav_msgs.msg import Actuators
from geometry_msgs.msg import Twist, Pose2D
import pickle
import os
# import sympy as sym
import math
class Quadrotor():
    def __init__(self):
    # publisher for rotor speeds
        self.motor_speed_pub = rospy.Publisher("/crazyflie2/command/motor_speed", Actuators, queue_size=10)
        # subscribe to Odometry topic
        self.odom_sub = rospy.Subscriber("/crazyflie2/ground_truth/odometry", Odometry, self.odom_callback)
        self.t0 = None
        self.t = None
        self.t_series = []
        self.x_series = []
        self.y_series = []
        self.z_series = []
        self.mutex_lock_on = False
        # self.tm = sym.symbols('t')
        self.omega = 0
        self.w = None
        self.kf = 1.28192*10**-8; self.km = 5.964552*10**-3; self.l = 0.046
        rospy.on_shutdown(self.save_data)
        # TODO: include initialization codes if needed
    def traj_evaluate(self):
        # evaluating the corresponding trajectories designed in Part 1
        if self.t <= 5:
            a1 = np.array([0, 0, (6*self.t**5)/3125 - (3*self.t**4)/125 + (2*self.t**3)/25])
            a1_vel = np.array([0,0,(6*self.t**4)/625 - (12*self.t**3)/125 + (6*self.t**2)/25])
            a1_acc = np.array([0, 0, (24*self.t**3)/625 - (36*self.t**2)/125 + (12*self.t)/25])

            return a1 , a1_vel, a1_acc

        if self.t >= 5 and self.t <=20:
            a2 = np.array([(1166016415523369*self.t**5)/147573952589676412928 - self.t**4/2025 + (22*self.t**3)/2025 - (8*self.t**2)/81 + (32*self.t)/81 - 47/81, 0, 1]) #new
            a2_vel = np.array([((5830082077616845*self.t**4)/147573952589676412928 - (4*self.t**3)/2025 + (22*self.t**2)/675 - (16*self.t)/81 + 32/81), 0, 0])  #new
            a2_acc = np.array([((5830082077616845*self.t**3)/36893488147419103232 - (4*self.t**2)/675 + (44*self.t)/675 - 16/81), 0, 0])    #new
            return a2, a2_vel, a2_acc

        if self.t>20 and self.t <= 35:

            a3 = np.array([1, (1166016415523369*self.t**5)/147573952589676412928 - (11*self.t**4)/10125 + (118*self.t**3)/2025 - (616*self.t**2)/405 + (1568*self.t)/81 - 7808/81, 1])
            a3_vel = np.array([0, ((5830082077616845*self.t**4)/147573952589676412928 - (44*self.t**3)/10125 + (118*self.t**2)/675 - (1232*self.t)/405 + 1568/81), 0])
            a3_acc = np.array([0, ((5830082077616845*self.t**3)/36893488147419103232 - (44*self.t**2)/3375 + (236*self.t)/675 - 1232/405), 0])
            return a3, a3_vel, a3_acc

        if self.t>35 and self.t <= 50:
            a4 = np.array([(- (1166016415523369*self.t**5)/147573952589676412928 + (17*self.t**4)/10125 - (286*self.t**3)/2025 + (476*self.t**2)/81 - (9800*self.t)/81 + 80000/81), 1, 1])
            a4_vel = np.array([(- (5830082077616845*self.t**4)/147573952589676412928 + (68*self.t**3)/10125 - (286*self.t**2)/675 + (952*self.t)/81 - 9800/81), 0, 0])
            a4_acc = np.array([(- (5830082077616845*self.t**3)/36893488147419103232 + (68*self.t**2)/3375 - (572*self.t)/675 + 952/81), 0, 0])
            return a4, a4_vel, a4_acc

        if self.t>50 and self.t <= 65:
            a5 = np.array([0, - (1166016415523369*self.t**5)/147573952589676412928 + (23*self.t**4)/10125 - (526*self.t**3)/2025 + (1196*self.t**2)/81 - (33800*self.t)/81 + 1289825552459047/274877906944, 1])
            a5_vel = np.array([0, (- (23320328310467385*self.t**4)/590295810358705651712 + (92*self.t**3)/10125 - (526*self.t**2)/675 + (2392*self.t)/81 - 33800/81), 0])
            a5_acc = np.array([0, (- (23320328310467385*self.t**3)/147573952589676412928 + (92*self.t**2)/3375 - (1052*self.t)/675 + 2392/81), 0])
            return a5, a5_vel, a5_acc

    def allocation_mat (self, u):

        alloc_mat = np.array((  [1/(4*self.kf), -sqrt(2)/(4*self.kf*self.l), -sqrt(2)/(4*self.kf*self.l), -1/(4*self.km*self.kf)],
                                [1/(4*self.kf), -sqrt(2)/(4*self.kf*self.l),  sqrt(2)/(4*self.kf*self.l),  1/(4*self.km*self.kf)],
                                [1/(4*self.kf),  sqrt(2)/(4*self.kf*self.l),  sqrt(2)/(4*self.kf*self.l), -1/(4*self.km*self.kf)],
                                [1/(4*self.kf),  sqrt(2)/(4*self.kf*self.l), -sqrt(2)/(4*self.kf*self.l),  1/(4*self.km*self.kf)]))

        w = np.sqrt(np.dot(alloc_mat,u))
        return w

    def smc_control(self, xyz, xyz_dot, rpy, rpy_dot):
        
        # obtain the desired values by evaluating the corresponding trajectories        
        m= 0.027; l = 0.046; ix = 16.571710*10**-6; iy = ix; iz = 29.261652*10**-6; g= 9.81; k = 2
        ip =12.656258*10**-8; kf = 1.28192*10**-8; km = 5.964552*10**-3; wmax = 2618; wmin = 0

        desir_pos, desir_vel,desir_acc = self.traj_evaluate()
  
        # The Sliding Mode Control laws designed in Part 2 to calculate the control inputs "u"
        
        phi = rpy[0]
        theta = rpy[1]
        lambz = 5                   # tuning parameter
        kz = 15                     # tuning parameter
        ez = xyz[2] - desir_pos[2]
        ez_dot = xyz_dot[2] - desir_vel[2]
        
        sz = ez_dot + lambz*ez
        boundary = 1
        satz = min(max(sz/boundary,-1),1)
        u1 = (m*(desir_acc[2] + g - kz*satz - lambz*(ez_dot)))/(cos(phi)*cos(theta))

        # find desires rpy
        Kp = 60                     #tuning parameters 50
        Kd = 5                      # tuning parameters
        ex = xyz[0]-desir_pos[0]
        ex_dot = xyz_dot[0] - desir_vel[0]
        dx_ddot = desir_acc[ 0]
        ey = xyz[1]-desir_pos[1]
        ey_dot = xyz_dot[1] - desir_vel[1]
        dy_ddot = desir_acc[ 1]
        
        Fx = m*(-Kp*(ex)-Kd*(ex_dot)+dx_ddot)
        Fy = m*(-Kp*(ey)-Kd*(ey_dot)+dy_ddot)

        dtheta = math.asin(Fx/u1)
        dphi = math.asin(-Fy/u1)
        dzeta = 0

        # Designing u2 
        lambphi = 12             # tuning parameters 10
        k_phi = 150             # tuning parameter
        ephi = rpy[0] - dphi
        if (ephi >= np.pi) :
                ephi = ephi % np.pi
        elif (ephi <= -np.pi):
            ephi = ephi % -np.pi
        omega = self.omega
        dtheta_dot = dtheta_ddot = dzeta_dot = dzeta_ddot = dphi_dot = dphi_ddot = zeta = 0
        e_phi_dot = rpy_dot[0] - dphi_dot
        sphi =  e_phi_dot+ lambphi * ephi
        sat_phi = min(max(sphi/boundary, -1), 1)
        u2 = ix*(dphi_ddot - k_phi*sat_phi - lambphi*(e_phi_dot) + (ip*omega*rpy_dot[1])/ix - (rpy_dot[1]*rpy_dot[2]*(iy - iz))/ix)
  
        # Design u3
        lambtheta = 12;             # Tuning parameter 10
        k_theta = 150
        etheta = rpy[1]- dtheta
        if (etheta >= np.pi) :
                etheta = etheta % np.pi
        elif (etheta <= -np.pi):
            ezeta = etheta% -np.pi 
        etheta_dot = rpy_dot[1] - dtheta_dot
        stheta = etheta_dot + lambtheta * etheta
        sat_theta = min(max(stheta/boundary,-1),1)
        u3 = -iy*(dtheta_ddot + k_theta*sat_theta + lambtheta*(etheta_dot) + (ip*omega*rpy_dot[0])/iy - (rpy_dot[0]*rpy_dot[2]*(ix - iz))/iy)

        # Design u4
        lamb_zeta = 5               #tuning parameter
        k_zeta = 25                 #tuning parameter 20
        ezeta = rpy[2]-dzeta
        if (ezeta >= np.pi) :
                ezeta = ezeta % np.pi
        elif (ezeta <= -np.pi):
            ezeta = ezeta % -np.pi 
        ezeta_dot = rpy_dot[2] - dzeta_dot
        szeta = ezeta_dot + lamb_zeta * ezeta
        sat_zeta = min (max(szeta/boundary,-1),1)
        u4 = iz*(dzeta_ddot - k_zeta*sat_zeta - lamb_zeta*(ezeta_dot))

        # convert the desired control inputs "u" to desired rotor

        u_mat = np.array([u1,u2,u3,u4])
        motor_vel = self.allocation_mat(u_mat)
        
        omg1 = motor_vel[0]
        omg2 = motor_vel[1]
        omg3 = motor_vel[2]
        omg4 = motor_vel[3]
        
        # maintain the rotor velocities within the valid range of [0 to 2618]
        if omg1>wmax:
            omg1 = wmax
        if omg2>wmax:
            omg2 = wmax
        if omg3>wmax:
            omg3 = wmax
        if omg4>wmax:
            omg4 = wmax

        motor_vel = np.array([[omg1], [omg2], [omg3], [omg4]])
        

        # Obtain Rotor speeds 4x1 vector
        motor_speed = Actuators()
        motor_speed.angular_velocities = [motor_vel[0,0], motor_vel[1,0], motor_vel[2,0], motor_vel[3,0]]
        self.motor_speed_pub.publish(motor_speed)
        self.omg = motor_vel[0,0] - motor_vel[1,0] + motor_vel[2,0] - motor_vel[3,0]

    # odometry callback function (DO NOT MODIFY)
    def odom_callback(self, msg):
        if self.t0 == None:
            self.t0 = msg.header.stamp.to_sec()
        self.t = msg.header.stamp.to_sec() - self.t0
        # convert odometry data to xyz, xyz_dot, rpy, and rpy_dot
        w_b = np.asarray([[msg.twist.twist.angular.x], [msg.twist.twist.angular.y], [msg.twist.twist.angular.z]])
        v_b = np.asarray([[msg.twist.twist.linear.x], [msg.twist.twist.linear.y], [msg.twist.twist.linear.z]])
        xyz = np.asarray([[msg.pose.pose.position.x], [msg.pose.pose.position.
        y], [msg.pose.pose.position.z]])
        q = msg.pose.pose.orientation
        T = tf.transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
        T[0:3, 3] = xyz[0:3, 0]
        R = T[0:3, 0:3]
        xyz_dot = np.dot(R, v_b)
        rpy = tf.transformations.euler_from_matrix(R, 'sxyz')
        rpy_dot = np.dot(np.asarray([
        [1, np.sin(rpy[0])*np.tan(rpy[1]), np.cos(rpy[0])*np.tan(rpy[1])],
        [0, np.cos(rpy[0]), -np.sin(rpy[0])],
        [0, np.sin(rpy[0])/np.cos(rpy[1]), np.cos(rpy[0])/np.cos(rpy[1])]
        ]), w_b)
        rpy = np.expand_dims(rpy, axis=1)

        # store the actual trajectory to be visualized later
        if (self.mutex_lock_on is not True):
            self.t_series.append(self.t)
            self.x_series.append(xyz[0, 0])
            self.y_series.append(xyz[1, 0])
            self.z_series.append(xyz[2, 0])
        # call the controller with the current states
        self.smc_control(xyz, xyz_dot, rpy, rpy_dot)

        rospy.Rate(100)

		# save the actual trajectory data
    def save_data(self):
        # TODO: update the path below with the correct path
        with open("/home/saurabh/rbe502_project/src/project/scripts/log.pkl","wb") as fp:
            self.mutex_lock_on = True
            pickle.dump([self.t_series,self.x_series,self.y_series,self.z_series], fp)
if __name__ == '__main__':
    rospy.init_node("quadrotor_control")
    rospy.loginfo("Press Ctrl + C to terminate")
    whatever = Quadrotor()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
