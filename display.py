#!/usr/bin/env python2

import rospy
import numpy as np
import matplotlib.pyplot as plt

import tf.transformations
from nav_msgs.msg import Odometry
from mavros_msgs.msg import AttitudeTarget
from trajectory_msgs.msg import JointTrajectory

class Display:
    def __init__(self):
        self.node_name = 'display_node'
        self.command_sub = 'mavros/setpoint_raw/attitude'
        self.position_sub = 'mavros/global_position/local'
        self.trajectory_sub = 'mavros/JointTrajectory'

        self.time_window = 10 # [s]
        self.window_10 = self.time_window * 10
        self.window_100 = self.time_window * 100
        self.x_10 = np.linspace(0, self.time_window, self.window_10)
        self.x_100 = np.linspace(0, self.time_window, self.window_100)

        #Attitude commands published at 100Hz
        self.rol = np.zeros(self.x_100.size)
        self.pit = np.zeros(self.x_100.size)
        self.yaw = np.zeros(self.x_100.size)
        self.thr = np.zeros(self.x_100.size)

        #EKF fusion position published at 100Hz
        self.x_measured = np.zeros(self.x_100.size)
        self.y_measured = np.zeros(self.x_100.size)
        self.z_measured = np.zeros(self.x_100.size)

        #Trajectory points published at 10Hz
        self.x_desired = np.zeros(self.x_10.size)
        self.y_desired = np.zeros(self.x_10.size)
        self.z_desired = np.zeros(self.x_10.size)

    def attitudeTargetCallback(self, attitude):
        q = (attitude.orientation.x, attitude.orientation.y, attitude.orientation.z, attitude.orientation.w)
        euler = tf.transformations.euler_from_quaternion(q)

        self.rol = np.append(self.rol, euler[0])[-self.window_100:]
        self.pit = np.append(self.pit, euler[1])[-self.window_100:]
        self.yaw = np.append(self.yaw, euler[2])[-self.window_100:]
        self.thr = np.append(self.thr, attitude.thrust)[-self.window_100:]

    def positionCallback(self, odometry):
        self.x_measured = np.append(self.x_measured, odometry.pose.pose.position.x)[-self.window_100:]
        self.y_measured = np.append(self.y_measured, odometry.pose.pose.position.y)[-self.window_100:]
        self.z_measured = np.append(self.z_measured, odometry.pose.pose.position.z)[-self.window_100:]

    def trajectoryCallback(self, trajectory):
        position = trajectory.points[0].positions

        self.x_desired = np.append(self.x_desired, position[0])[-self.window_10:]
        self.y_desired = np.append(self.y_desired, position[1])[-self.window_10:]
        self.z_desired = np.append(self.z_desired, position[2])[-self.window_10:]

    def start(self):
        rospy.init_node(self.node_name, anonymous=True)

        rospy.Subscriber(self.command_sub, AttitudeTarget, self.attitudeTargetCallback)
        rospy.Subscriber(self.position_sub, Odometry, self.positionCallback)
        rospy.Subscriber(self.trajectory_sub, JointTrajectory, self.trajectoryCallback)

        start = rospy.Time.now()
        rate = rospy.Rate(10)

        plt.ion()

        fig = plt.figure(figsize=(14,12))
        ax1 = fig.add_subplot(221)
        ax1.set_title('Thrust command')
        ax1.legend('thrust')
        ax1.set_xlim(0, self.time_window)
        ax1.set_ylim(0, 1)
        line1, = ax1.plot(self.x_100, self.thr, 'r-')

        ax2 = fig.add_subplot(223)
        ax2.set_title('Attitude command')
        ax2.legend(['roll', 'pitch', 'yaw'])
        ax2.set_xlim(0, self.time_window)
        ax2.set_ylim(0, 3)
        line2, = ax2.plot(self.x_100, self.rol, 'r-')
        line3, = ax2.plot(self.x_100, self.pit, 'g-')
        line4, = ax2.plot(self.x_100, self.yaw, 'b-')

        ax3 = fig.add_subplot(222)
        ax3.set_title('Local position')
        ax3.legend(['x', 'y', 'z'])
        ax3.set_xlim(0, self.time_window)
        ax3.set_ylim(-4, 4)
        line5, = ax3.plot(self.x_100, self.x_measured, 'r-')
        line6, = ax3.plot(self.x_100, self.y_measured, 'g-')
        line7, = ax3.plot(self.x_100, self.z_measured, 'b-')

        ax4 = fig.add_subplot(224)
        ax4.set_title('Trajectory')
        ax4.legend(['x', 'y', 'z'])
        ax4.set_xlim(0, self.time_window)
        ax4.set_ylim(-4, 4)
        line8, = ax4.plot(self.x_10, self.x_desired, 'r--')
        line9, = ax4.plot(self.x_10, self.y_desired, 'g--')
        line0, = ax4.plot(self.x_10, self.z_desired, 'b--')

        while not rospy.is_shutdown():
            line1.set_ydata(self.thr)
            line2.set_ydata(self.rol)
            line3.set_ydata(self.pit)
            line4.set_ydata(self.yaw)
            line5.set_ydata(self.x_measured)
            line6.set_ydata(self.y_measured)
            line7.set_ydata(self.z_measured)
            line8.set_ydata(self.x_desired)
            line9.set_ydata(self.y_desired)
            line0.set_ydata(self.z_desired)
            fig.canvas.draw()
            fig.canvas.flush_events()

if __name__ == '__main__':

    d = Display()
    d.start()
