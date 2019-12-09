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
        self.position_sub = 'mavros/local_position/odom'
        self.trajectory_sub = 'mavros/JointTrajectory'
        self.reference_sub = 'mavros/referenceStates'
        self.estimated_sub = 'mavros/estimatedStates'

        self.time_window = 15  # [s]
        self.window_10 = self.time_window * 10
        self.window_30 = self.time_window * 30
        self.window_100 = self.time_window * 100
        self.x_10 = np.linspace(0, self.time_window, self.window_10)
        self.x_30 = np.linspace(0, self.time_window, self.window_30)
        self.x_100 = np.linspace(0, self.time_window, self.window_100)

        # Attitude commands published at 100Hz
        self.rol = np.zeros(self.x_100.size)
        self.pit = np.zeros(self.x_100.size)
        self.yaw = np.zeros(self.x_100.size)
        self.thr = np.zeros(self.x_100.size)

        # EKF fusion position published at 100Hz
        self.x_measured = np.zeros(self.x_30.size)
        self.y_measured = np.zeros(self.x_30.size)
        self.z_measured = np.zeros(self.x_30.size)

        # Trajectory points published at 10Hz
        self.r_desired = np.zeros(self.x_10.size)
        self.b_desired = np.zeros(self.x_10.size)
        self.vr_desired = np.zeros(self.x_10.size)
        self.vb_desired = np.zeros(self.x_10.size)
        self.ar_desired = np.zeros(self.x_10.size)
        self.ab_desired = np.zeros(self.x_10.size)

    def attitudeTargetCallback(self, attitude):
        q = (attitude.orientation.x, attitude.orientation.y, attitude.orientation.z, attitude.orientation.w)
        euler = tf.transformations.euler_from_quaternion(q)

        self.rol = np.append(self.rol, euler[0])[-self.window_100:]
        self.pit = np.append(self.pit, euler[1])[-self.window_100:]
        self.yaw = np.append(self.yaw, euler[2])[-self.window_100:]
        self.thr = np.append(self.thr, attitude.thrust)[-self.window_100:]

    def positionCallback(self, odometry):
        self.x_measured = np.append(self.x_measured, odometry.pose.pose.position.x)[-self.window_30:]
        self.y_measured = np.append(self.y_measured, odometry.pose.pose.position.y)[-self.window_30:]
        self.z_measured = np.append(self.z_measured, odometry.pose.pose.position.z)[-self.window_30:]

    def trajectoryCallback(self, trajectory):
        positions = trajectory.points[0].positions
        velocities = trajectory.points[0].positions
        accelerations = trajectory.points[0].positions

        self.r_desired = np.append(self.r_desired, positions[0])[-self.window_10:]
        self.b_desired = np.append(self.b_desired, positions[1])[-self.window_10:]
        self.vr_desired = np.append(self.vr_desired, velocities[0])[-self.window_10:]
        self.vb_desired = np.append(self.vb_desired, velocities[1])[-self.window_10:]
        self.ar_desired = np.append(self.ar_desired, accelerations[0])[-self.window_10:]
        self.ab_desired = np.append(self.ab_desired, accelerations[1])[-self.window_10:]

    def start(self):
        rospy.init_node(self.node_name, anonymous=True)

        rospy.Subscriber(self.command_sub, AttitudeTarget, self.attitudeTargetCallback)
        rospy.Subscriber(self.position_sub, Odometry, self.positionCallback)
        rospy.Subscriber(self.trajectory_sub, JointTrajectory, self.trajectoryCallback)

        rospy.Rate(10)

        plt.ion()

        fig = plt.figure(figsize=(14, 12))

        ax_thr = fig.add_subplot(321)
        ax_thr.set_title('Thrust command')
        ax_thr.legend('thrust')
        ax_thr.set_xlim(0, self.time_window)
        ax_thr.set_ylim(0, 1)
        line_thr, = ax_thr.plot(self.x_100, self.thr, 'r-')

        ax_att = fig.add_subplot(323)
        ax_att.set_title('Attitude command')
        ax_att.legend(['roll', 'pitch', 'yaw'])
        ax_att.set_xlim(0, self.time_window)
        ax_att.set_ylim(0, 3)
        line_rol, = ax_att.plot(self.x_100, self.rol, 'r-')
        line_pit, = ax_att.plot(self.x_100, self.pit, 'g-')
        line_yaw, = ax_att.plot(self.x_100, self.yaw, 'b-')

        ax_pos = fig.add_subplot(322)
        ax_pos.set_title('Measured (-) position')
        ax_pos.legend(['x_measured', 'y_measured', 'z_measured'])
        ax_pos.set_xlim(0, self.time_window)
        ax_pos.set_ylim(-4, 4)
        line_xmea, = ax_pos.plot(self.x_30, self.x_measured, 'r-')
        line_ymea, = ax_pos.plot(self.x_30, self.y_measured, 'g-')
        line_zmea, = ax_pos.plot(self.x_30, self.z_measured, 'b-')

        ax_r = fig.add_subplot(324)
        ax_r.set_title('Cable length')
        ax_r.legend(['r_desired'])
        ax_r.set_xlim(0, self.time_window)
        ax_r.set_ylim(-4, 4)
        line_rdes, = ax_r.plot(self.x_10, self.r_desired, 'r-')

        ax_vr = fig.add_subplot(326)
        ax_vr.set_title('Cable velocity and acceleration')
        ax_vr.legend(['vr_desired', 'ar_desired'])
        ax_vr.set_xlim(0, self.time_window)
        ax_vr.set_ylim(-4, 4)
        line_vrdes, = ax_vr.plot(self.x_10, self.vr_desired, 'b-')
        line_ardes, = ax_vr.plot(self.x_10, self.ar_desired, 'r-')

        ax_b = fig.add_subplot(325)
        ax_b.set_title('Elevation angle')
        ax_b.legend(['b_desired'])
        ax_b.set_xlim(0, self.time_window)
        ax_b.set_ylim(-4, 4)
        line_bdes, = ax_b.plot(self.x_10, self.b_desired, 'r-')

        while not rospy.is_shutdown():
            try:
                line_thr.set_ydata(self.thr)
                line_rol.set_ydata(self.rol)
                line_pit.set_ydata(self.pit)
                line_yaw.set_ydata(self.yaw)
                line_xmea.set_ydata(self.x_measured)
                line_ymea.set_ydata(self.y_measured)
                line_zmea.set_ydata(self.z_measured)
                line_rdes.set_ydata(self.r_desired)
                line_vrdes.set_ydata(self.vr_desired)
                line_ardes.set_ydata(self.ar_desired)
                line_bdes.set_ydata(self.b_desired)

                fig.canvas.draw()
                fig.canvas.flush_events()

            except Exception:
                break


if __name__ == '__main__':

    d = Display()
    d.start()
