#!/usr/bin/env python2

import sys
import math
import numpy as np
from scipy import signal
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from timeit import default_timer as time

import rospy
# from dynamic_reconfigure.server import Server
from std_msgs.msg import String, Header
from mavros_msgs.msg import AttitudeTarget
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from nav_msgs.msg import Odometry

class TrajectoryGeneration:
    def __init__(self, node_name='trajectory_gen_node', subscriber='mavros/local_position/odom' , publisher='mavros/JointTrajectory'):

        rospy.init_node(node_name, anonymous=True)

        # Define suscribers & publishers
        rospy.Subscriber(subscriber, Odometry, self.callback)
        self.pub = rospy.Publisher(publisher, JointTrajectory, queue_size=10)

        self.YAW_HEADING = ['center', [1, 1, 2]] # auto, center, axes
        self.TRAJECTORY_REQUESTED_SPEED = 7. # max. linear speed [m.s-1]
        self.MAX_LINEAR_ACC = 10. # max. linear acceleration [m.s-2]
        self.MAX_LINEAR_JERK = 20. # max. linear jerk [m.s-3]
        self.PUBLISH_RATE = 10 # publisher frequency
        self.FREQUENCY = 100. # [Hz]
        self.DELTA_L = self.TRAJECTORY_REQUESTED_SPEED / self.FREQUENCY # [m.s]
        self.BOX_LIMIT = [[-4., 4.], [-4., 4.], [0., 6.]] # [[x_min, x_max], [y_min, y_max], [z_min, z_max]]
        self.WINDOW_FRAME = .5 # publish future states comprise in the window time [s]
        self.EXTRA_POINTS_START = 35
        self.EXTRA_POINTS_END = 100

        self.x_discretized = [.0] * self.EXTRA_POINTS_START
        self.y_discretized = [.0] * self.EXTRA_POINTS_START
        self.z_discretized = [.0] * self.EXTRA_POINTS_START

        self.MAX_LINEAR_SPEED_XY = 10. # max. linear speed [m.s-1]
        self.MAX_LINEAR_SPEED_Z = 15. # max. linear speed [m.s-1]
        self.MAX_LINEAR_ACC_XY = 100. # max. linear acceleration [m.s-2]
        self.MAX_LINEAR_ACC_Z = 150. # max. linear acceleration [m.s-2]
        self.MAX_LINEAR_JERK_XY = 10. # max linear jerk [m.s-3]
        self.MAX_LINEAR_JERK_Z = 20. # max linear jerk [m.s-3]

        #Parameters for 2nd order filtering
        self.COEFF_X = .9
        self.COEFF_Y = .9
        self.COEFF_Z = .15

        self.is_filtered = False
        self.is_first_callback = False

    def discretise_trajectory(self, parameters=[]):

        start = time()

        x1 = self.x_discretized[-1]
        y1 = self.y_discretized[-1]
        z1 = self.z_discretized[-1]

        if parameters[0] == 'vector':

            steps = self.norm(parameters[1], [x1, y1, z1]) / self.DELTA_L

            x = np.linspace(x1, parameters[1][0], steps, endpoint=True)
            y = np.linspace(y1, parameters[1][1], steps, endpoint=True)
            z = np.linspace(z1, parameters[1][2], steps, endpoint=True)

        elif parameters[0] == 'circle':
            center = parameters[1]
            radius = self.norm(center, [x1, y1, z1])
            steps = int( 2 * math.pi * radius / self.DELTA_L)

            cos_a = (x1 - center[0]) / radius
            sin_a = (y1 - center[1]) / radius

            cos_b = lambda x: math.cos((2 * math.pi / steps) * x)
            sin_b = lambda x: math.sin((2 * math.pi / steps) * x)

            x = [((cos_a*cos_b(s) - sin_a*sin_b(s))*radius + center[0]) for s in range(0,steps+1)]
            y = [((sin_a*cos_b(s) + cos_a*sin_b(s))*radius + center[1]) for s in range(0,steps+1)]
            z = [(center[2]) for s in range(0,steps+1)]

        elif parameters[0] == 'hover':
            steps = int(parameters[1] * self.FREQUENCY)

            x = x1 * np.ones(steps)
            y = y1 * np.ones(steps)
            z = z1 * np.ones(steps)

        #elif parameters[0] == 'square':
        #elif parameters[0] == 'inf':

        self.x_discretized.extend(x[1:])
        self.y_discretized.extend(y[1:])
        self.z_discretized.extend(z[1:])

        print('discretise_trajectory() runs in {} s'.format(time() - start))

    def constraint_trajectory_to_box(self):
        self.x_discretized = [self.BOX_LIMIT[0][0] if x < self.BOX_LIMIT[0][0] else x for x in self.x_discretized]
        self.x_discretized = [self.BOX_LIMIT[0][1] if x > self.BOX_LIMIT[0][1] else x for x in self.x_discretized]
        self.y_discretized = [self.BOX_LIMIT[1][0] if x < self.BOX_LIMIT[1][0] else x for x in self.y_discretized]
        self.y_discretized = [self.BOX_LIMIT[1][1] if x > self.BOX_LIMIT[1][1] else x for x in self.y_discretized]
        self.z_discretized = [self.BOX_LIMIT[2][0] if x < self.BOX_LIMIT[2][0] else x for x in self.z_discretized]
        self.z_discretized = [self.BOX_LIMIT[2][1] if x > self.BOX_LIMIT[2][1] else x for x in self.z_discretized]

    def generate_states(self):

        start = time()

        self.x_discretized.extend([self.x_discretized[-1]] * self.EXTRA_POINTS_END)
        self.y_discretized.extend([self.x_discretized[-1]] * self.EXTRA_POINTS_END)
        self.z_discretized.extend([self.x_discretized[-1]] * self.EXTRA_POINTS_END)

        self.ya_discretized = [.0]
        self.vx_discretized = [.0]
        self.vy_discretized = [.0]
        self.vz_discretized = [.0]
        self.ax_discretized = [.0]
        self.ay_discretized = [.0]
        self.az_discretized = [.0]
        self.ti_discretized = [.0]

        for s,_ in enumerate(self.x_discretized[1:]):
            p1 = np.array([self.x_discretized[s], self.y_discretized[s], self.z_discretized[s]])
            p2 = np.array([self.x_discretized[s+1], self.y_discretized[s+1], self.z_discretized[s+1]])

            if self.YAW_HEADING[0] == 'center':
                heading = np.array(self.YAW_HEADING[1]) - p1
            elif self.YAW_HEADING[0] == 'axes':
                heading = np.array(self.YAW_HEADING[1])
            else:
                heading = p2 - p1

            heading = (heading / self.norm(heading)) if self.norm(heading) != 0 else np.array([.0, .0, .0])

            self.ya_discretized.append(math.atan2(heading[1], heading[0]))
            self.vx_discretized.append((self.x_discretized[s+1] - self.x_discretized[s]) * self.FREQUENCY)
            self.vy_discretized.append((self.y_discretized[s+1] - self.y_discretized[s]) * self.FREQUENCY)
            self.vz_discretized.append((self.z_discretized[s+1] - self.z_discretized[s]) * self.FREQUENCY)
            self.ax_discretized.append((self.vx_discretized[-1] - self.vx_discretized[-2]) * self.FREQUENCY)
            self.ay_discretized.append((self.vy_discretized[-1] - self.vy_discretized[-2]) * self.FREQUENCY)
            self.az_discretized.append((self.vz_discretized[-1] - self.vz_discretized[-2]) * self.FREQUENCY)
            self.ti_discretized.append((s + 1.) / self.FREQUENCY)

        print('generate_states() runs in {} s'.format(time() - start))

    def generate_states_filtered(self):

        start = time()

        self.is_filtered = True

        self.x_filtered = [.0]
        self.y_filtered = [.0]
        self.z_filtered = [.0]
        self.vx_filtered = [.0]
        self.vy_filtered = [.0]
        self.vz_filtered = [.0]
        self.ax_filtered = [.0]
        self.ay_filtered = [.0]
        self.az_filtered = [.0]

        saturate = lambda x, y: math.copysign(min(x, y, key=abs), x)

        for s,_ in enumerate(self.vx_discretized[1:]):
            self.ax_filtered.append(saturate((self.vx_discretized[s+1] - self.vx_filtered[-1]) * self.FREQUENCY, self.MAX_LINEAR_ACC_XY))
            self.ay_filtered.append(saturate((self.vy_discretized[s+1] - self.vy_filtered[-1]) * self.FREQUENCY, self.MAX_LINEAR_ACC_XY))
            self.az_filtered.append(saturate((self.vz_discretized[s+1] - self.vz_filtered[-1]) * self.FREQUENCY, self.MAX_LINEAR_ACC_Z))

            self.vx_filtered.append(saturate(self.vx_filtered[-1] + (self.ax_filtered[-1] / self.FREQUENCY), self.MAX_LINEAR_SPEED_XY))
            self.vy_filtered.append(saturate(self.vy_filtered[-1] + (self.ay_filtered[-1] / self.FREQUENCY), self.MAX_LINEAR_SPEED_XY))
            self.vz_filtered.append(saturate(self.vz_filtered[-1] + (self.az_filtered[-1] / self.FREQUENCY), self.MAX_LINEAR_SPEED_Z))

            self.x_filtered.append(self.x_filtered[-1] + (self.vx_filtered[-1] / self.FREQUENCY))
            self.y_filtered.append(self.y_filtered[-1] + (self.vy_filtered[-1] / self.FREQUENCY))
            self.z_filtered.append(self.z_filtered[-1] + (self.vz_filtered[-1] / self.FREQUENCY))

        print('generate_states_filtered() runs in {} s'.format(time() - start))

    def generate_states_sg_filtered(self, window_length=51, polyorder=3, deriv=0, delta=1.0, mode='mirror', on_filtered=False):
        ### Info: Apply Savitzky-Golay filter to velocities ###

        start = time()

        self.is_filtered = True

        self.x_filtered = [.0]
        self.y_filtered = [.0]
        self.z_filtered = [.0]

        if on_filtered:
            self.vx_filtered = signal.savgol_filter(x=self.vx_filtered, window_length=window_length, polyorder=polyorder, deriv=deriv, delta=delta, mode=mode)
            self.vy_filtered = signal.savgol_filter(x=self.vy_filtered, window_length=window_length, polyorder=polyorder, deriv=deriv, delta=delta, mode=mode)
            self.vz_filtered = signal.savgol_filter(x=self.vz_filtered, window_length=window_length, polyorder=polyorder, deriv=deriv, delta=delta, mode=mode)
        else:
            self.vx_filtered = signal.savgol_filter(x=self.vx_discretized, window_length=window_length, polyorder=polyorder, deriv=deriv, delta=delta, mode=mode)
            self.vy_filtered = signal.savgol_filter(x=self.vy_discretized, window_length=window_length, polyorder=polyorder, deriv=deriv, delta=delta, mode=mode)
            self.vz_filtered = signal.savgol_filter(x=self.vz_discretized, window_length=window_length, polyorder=polyorder, deriv=deriv, delta=delta, mode=mode)

        self.ax_filtered = [.0]
        self.ay_filtered = [.0]
        self.az_filtered = [.0]

        saturate = lambda x, y: math.copysign(min(x, y, key=abs), x)
        # saturate = lambda x, y: x

        for s,_ in enumerate(self.vx_discretized[1:]):
            self.ax_filtered.append(saturate((self.vx_filtered[s+1] - self.vx_filtered[s]) * self.FREQUENCY, self.MAX_LINEAR_ACC_XY))
            self.ay_filtered.append(saturate((self.vy_filtered[s+1] - self.vy_filtered[s]) * self.FREQUENCY, self.MAX_LINEAR_ACC_XY))
            self.az_filtered.append(saturate((self.vz_filtered[s+1] - self.vz_filtered[s]) * self.FREQUENCY, self.MAX_LINEAR_ACC_Z))

            self.vx_filtered[s+1] = self.vx_filtered[s] + (self.ax_filtered[-1] / self.FREQUENCY)
            self.vy_filtered[s+1] = self.vy_filtered[s] + (self.ay_filtered[-1] / self.FREQUENCY)
            self.vz_filtered[s+1] = self.vz_filtered[s] + (self.az_filtered[-1] / self.FREQUENCY)

            self.x_filtered.append(self.x_filtered[-1] + (self.vx_filtered[s+1] / self.FREQUENCY))
            self.y_filtered.append(self.y_filtered[-1] + (self.vy_filtered[s+1] / self.FREQUENCY))
            self.z_filtered.append(self.z_filtered[-1] + (self.vz_filtered[s+1] / self.FREQUENCY))

        print('generate_states_sg_filtered() runs in {} s'.format(time() - start))

    def second_order_filter(self, param, cmd, cmd_last, cmd_2last):
        cmd_2last = param * cmd_2last + (1.0 - param) * cmd_last
        cmd_last = param * cmd_last + (1.0 - param) * cmd

        return [cmd_last, cmd_2last]

    def generate_states_2nd_order_filtered(self):
        start = time()

        self.is_filtered = True

        self.vx_discretized = [.0]
        self.vy_discretized = [.0]
        self.vz_discretized = [.0]
        self.ax_discretized = [.0]
        self.ay_discretized = [.0]
        self.az_discretized = [.0]

        self.x_filtered = [.0]
        self.y_filtered = [.0]
        self.z_filtered = [.0]
        self.vx_filtered = [.0]
        self.vy_filtered = [.0]
        self.vz_filtered = [.0]
        self.ax_filtered = [.0]
        self.ay_filtered = [.0]
        self.az_filtered = [.0]

        saturate = lambda x, y: math.copysign(min(x, y, key=abs), x)

        ax_cmd_last = .0
        ax_cmd_2last = .0
        ay_cmd_last = .0
        ay_cmd_2last = .0
        az_cmd_last = .0
        az_cmd_2last = .0

        for s,_ in enumerate(self.x_discretized[1:]):

            self.vx_discretized.append(saturate((self.x_discretized[s+1] - self.x_discretized[s]) * self.FREQUENCY, self.MAX_LINEAR_SPEED_XY))
            self.vy_discretized.append(saturate((self.y_discretized[s+1] - self.y_discretized[s]) * self.FREQUENCY, self.MAX_LINEAR_SPEED_XY))
            self.vz_discretized.append(saturate((self.z_discretized[s+1] - self.z_discretized[s]) * self.FREQUENCY, self.MAX_LINEAR_SPEED_Z))
            self.ax_discretized.append(saturate((self.vx_discretized[-1] - self.vx_discretized[-2]) * self.FREQUENCY, self.MAX_LINEAR_ACC_XY))
            self.ay_discretized.append(saturate((self.vy_discretized[-1] - self.vy_discretized[-2]) * self.FREQUENCY, self.MAX_LINEAR_ACC_XY))
            self.az_discretized.append(saturate((self.vz_discretized[-1] - self.vz_discretized[-2]) * self.FREQUENCY, self.MAX_LINEAR_ACC_Z))

        for s,_ in enumerate(self.vx_discretized[1:]):

            ax_cmd_last, ax_cmd_2last = self.second_order_filter(self.COEFF_X, self.ax_discretized[s], ax_cmd_last, ax_cmd_2last)
            ay_cmd_last, ay_cmd_2last = self.second_order_filter(self.COEFF_Y, self.ay_discretized[s], ay_cmd_last, ay_cmd_2last)
            az_cmd_last, az_cmd_2last = self.second_order_filter(self.COEFF_Z, self.az_discretized[s], az_cmd_last, az_cmd_2last)

            self.ax_filtered.append(ax_cmd_2last)
            self.ay_filtered.append(ay_cmd_2last)
            self.az_filtered.append(az_cmd_2last)

            self.vx_filtered.append(self.vx_filtered[-1] + (self.ax_filtered[-1] / self.FREQUENCY))
            self.vy_filtered.append(self.vy_filtered[-1] + (self.ay_filtered[-1] / self.FREQUENCY))
            self.vz_filtered.append(self.vz_filtered[-1] + (self.az_filtered[-1] / self.FREQUENCY))

            self.x_filtered.append(self.x_filtered[-1] + (self.vx_filtered[-1] / self.FREQUENCY))
            self.y_filtered.append(self.y_filtered[-1] + (self.vy_filtered[-1] / self.FREQUENCY))
            self.z_filtered.append(self.z_filtered[-1] + (self.vz_filtered[-1] / self.FREQUENCY))

        print('generate_states_2nd_order_filtered() runs in {} s'.format(time() - start))

    def plot_trajectory_extras(self):

        start = time()

        n = 3 # Plot velocity and heading every n points to get a clearer graph
        alpha = .3 # Transparancy for velocity and heading arrows

        fig = plt.figure(figsize=(16,8))

        ax1 = fig.add_subplot(121, projection='3d')
        ax1.scatter(self.x_discretized, self.y_discretized, self.z_discretized, label='trajectory', color='blue')
        ax1.quiver(
            self.x_discretized[0::n], self.y_discretized[0::n], self.z_discretized[0::n],
            self.vx_discretized[0::n], self.vy_discretized[0::n], self.vz_discretized[0::n],
            length=.05, color='red', alpha=alpha, label='velocity')
        ax1.quiver(
            self.x_discretized[0::n], self.y_discretized[0::n], self.z_discretized[0::n],
            [math.cos(a) for a in self.ya_discretized[0::n]], [math.sin(a) for a in self.ya_discretized[0::n]], [.0 for a in self.ya_discretized[0::n]],
            length=.3, color='green', alpha=alpha, label='heading')
        ax1.axis('equal')
        plt.legend()
        plt.title('Trajectory')

        ax2 = fig.add_subplot(222)
        ax2.plot(self.vx_discretized, color='red', label='vx_desired')
        ax2.plot(self.vy_discretized, color='green', label='vy_desired')
        ax2.plot(self.vz_discretized, color='blue', label='vz_desired')
        plt.legend()
        plt.title('Velocity')

        ax3 = fig.add_subplot(224)
        ax3.plot(self.ax_discretized, color='red', label='ax_desired')
        ax3.plot(self.ay_discretized, color='green', label='ay_desired')
        ax3.plot(self.az_discretized, color='blue', label='az_desired')
        plt.legend()
        plt.title('Acceleration')

        print('plot_trajectory_extras() runs in {} s'.format(time() - start))

        fig.tight_layout()
        plt.show()

    def plot_trajectory_extras_filtered(self):

        start = time()

        n = 3 # Plot velocity and heading every n points to get a clearer graph
        alpha = .3 # Transparancy for velocity and heading arrows

        fig = plt.figure(figsize=(16,8))

        ax1 = fig.add_subplot(121, projection='3d')
        ax1.scatter(self.x_discretized, self.y_discretized, self.z_discretized, label='trajectory_desired', color='blue')
        ax1.scatter(self.x_filtered, self.y_filtered, self.z_filtered, label='trajectory_filtered', color='red')
        ax1.quiver(
            self.x_filtered[0::n], self.y_filtered[0::n], self.z_filtered[0::n],
            self.vx_filtered[0::n], self.vy_filtered[0::n], self.vz_filtered[0::n],
            length=.05, color='red', alpha=alpha, label='velocity')
        ax1.quiver(
            self.x_filtered[0::n], self.y_filtered[0::n], self.z_filtered[0::n],
            [math.cos(a) for a in self.ya_discretized[0::n]], [math.sin(a) for a in self.ya_discretized[0::n]], [.0 for a in self.ya_discretized[0::n]],
            length=.3, color='green', alpha=alpha, label='heading')
        # ax1.axis('equal')
        plt.legend()
        plt.title('Trajectory')

        ax2 = fig.add_subplot(222)
        ax2.plot(self.vx_discretized, color='red', label='vx_desired')
        ax2.plot(self.vy_discretized, color='green', label='vy_desired')
        ax2.plot(self.vz_discretized, color='blue', label='vz_desired')
        ax2.plot(self.vx_filtered, color='red', label='vx_filtered', linestyle='--')
        ax2.plot(self.vy_filtered, color='green', label='vy_filtered', linestyle='--')
        ax2.plot(self.vz_filtered, color='blue', label='vz_filtered', linestyle='--')
        plt.legend()
        plt.title('Velocity')

        ax3 = fig.add_subplot(224)
        ax3.plot(self.ax_discretized, color='red', label='ax_desired')
        ax3.plot(self.ay_discretized, color='green', label='ay_desired')
        ax3.plot(self.az_discretized, color='blue', label='az_desired')
        ax3.plot(self.ax_filtered, color='red', label='ax_filtered', linestyle='--')
        ax3.plot(self.ay_filtered, color='green', label='ay_filtered', linestyle='--')
        ax3.plot(self.az_filtered, color='blue', label='az_filtered', linestyle='--')
        plt.legend()
        plt.title('Acceleration')

        print('plot_trajectory_extras_filtered() runs in {} s'.format(time() - start))

        fig.tight_layout()
        plt.show()

    def start(self):
            start = rospy.Time.now()
            rate = rospy.Rate(self.PUBLISH_RATE)
            s = 0

            while not (rospy.is_shutdown() or s >= len(self.x_discretized)):

                # Build JointTrajectory message
                header = Header()
                header.seq = s
                header.stamp = rospy.get_rostime()
                header.frame_id = 'inertial frame'

                joint_trajectory_msg = JointTrajectory()
                joint_trajectory_msg.header = header
                joint_trajectory_msg.joint_names = ['t', 't1']

                # Build JointTrajectoryPoint
                for i in range(min(self.WINDOW_FRAME, len(self.x_filtered) - s)):
                    joint_trajectory_point = JointTrajectoryPoint()
                    joint_trajectory_point.positions = [self.x_filtered[s+i], self.y_filtered[s+i], self.z_filtered[s+i], self.ya_discretized[s+i]]
                    joint_trajectory_point.velocities = [self.vx_filtered[s+i], self.vy_filtered[s+i], self.vz_filtered[s+i]] #if i != (self.WINDOW_FRAME - 1) else [.0, .0, .0]
                    joint_trajectory_point.accelerations = [self.ax_filtered[s+i], self.ay_filtered[s+i], self.az_filtered[s+i]] #if i != (self.WINDOW_FRAME - 1) else [.0, .0, .0]
                    joint_trajectory_point.effort = []
                    joint_trajectory_point.time_from_start = rospy.Duration.from_sec(self.ti_discretized[s+i])

                    joint_trajectory_msg.points.append(joint_trajectory_point)

                s = s + int(self.FREQUENCY/self.PUBLISH_RATE)

                rospy.loginfo('##########################################')
                rospy.loginfo(joint_trajectory_msg)
                self.pub.publish(joint_trajectory_msg)
                rate.sleep()

    def norm(self, p1, p2=[.0, .0, .0]):
        return math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2 + (p2[2] - p1[2]) ** 2)

    def callback(self, odom):
        if not self.is_first_callback:

            position = odom.pose.pose.position

            print('position {} s'.format(position))

            self.x_discretized = [position.x] * self.EXTRA_POINTS_START
            self.y_discretized = [position.y] * self.EXTRA_POINTS_START
            self.z_discretized = [position.z] * self.EXTRA_POINTS_START

            self.is_first_callback = True

    def check_callback(self):
        rospy.loginfo("Waiting for position measurement callback ...")
        while not (rospy.is_shutdown() or self.is_first_callback):
            pass

if __name__ == '__main__':

    node_name='trajectory_gen_node'
    subscriber='mavros/local_position/odom'
    publisher='mavros/JointTrajectory'

    try:
        trajectory_object = TrajectoryGeneration(node_name=node_name, subscriber=subscriber , publisher=publisher)

        ########################################################################
        # Configuration
        trajectory_object.YAW_HEADING = ['auto', [1, 1, 2]] # auto, center, axes
        trajectory_object.TRAJECTORY_REQUESTED_SPEED = 0.4 # [m.s-1] to compute the step for discretized trajectory
        trajectory_object.PUBLISH_RATE = 10 # publisher frequency
        trajectory_object.FREQUENCY = 10 # [Hz]
        trajectory_object.DELTA_L = trajectory_object.TRAJECTORY_REQUESTED_SPEED / trajectory_object.FREQUENCY # step in [m] to discretize trajectory
        trajectory_object.BOX_LIMIT = [[-4., 4.], [-4., 4.], [0., 6.]] # [[x_min, x_max], [y_min, y_max], [z_min, z_max]]
        trajectory_object.WINDOW_FRAME = 5 # publish the n future states

        trajectory_object.MAX_LINEAR_SPEED_XY = 0.5 # max. linear speed [m.s-1]
        trajectory_object.MAX_LINEAR_SPEED_Z = 0.5 # max. linear speed [m.s-1]
        trajectory_object.MAX_LINEAR_ACC_XY = 1.5 # max. linear acceleration [m.s-2]
        trajectory_object.MAX_LINEAR_ACC_Z = 2. # max. linear acceleration [m.s-2]
        trajectory_object.MAX_LINEAR_JERK_XY = 4. # max linear jerk [m.s-3]
        trajectory_object.MAX_LINEAR_JERK_Z = 4. # max linear jerk [m.s-3]

        # Wait for the first measurement callback to initialize the starting position of the trajectory
        trajectory_object.check_callback()

        # Define trajectory shape/vertices in NED frame
        trajectory_object.discretise_trajectory(parameters=['vector', [.0, .0, 2.0]])
        trajectory_object.discretise_trajectory(parameters=['hover', 15.])
        # trajectory_object.discretise_trajectory(parameters=['vector', [.0, .0, 0.]])
        trajectory_object.discretise_trajectory(parameters=['circle', [.0, 2., 2.]])
        trajectory_object.discretise_trajectory(parameters=['vector', [1., 2., 3.]])
        trajectory_object.discretise_trajectory(parameters=['circle', [.0, 1., 3.]])
        trajectory_object.discretise_trajectory(parameters=['vector', [.0, .0, 3.]])
        trajectory_object.discretise_trajectory(parameters=['vector', [.0, .0, .0]])
        ########################################################################

        # Limit the trajectory to the BOX_LIMIT
        trajectory_object.constraint_trajectory_to_box()

        # Generate the list of states - start by generating the states and then filter them
        trajectory_object.generate_states()
        # trajectory_object.generate_states_filtered()
        trajectory_object.generate_states_sg_filtered(window_length=31, polyorder=1, mode='mirror')
        trajectory_object.generate_states_sg_filtered(window_length=31, polyorder=1, mode='mirror', on_filtered=True)
        # trajectory_object.generate_states_2nd_order_filtered()

        # Plot the trajectory
        # trajectory_object.plot_trajectory_extras()
        trajectory_object.plot_trajectory_extras_filtered()

        # Publish trajectory states
        trajectory_object.start()

    except rospy.ROSInterruptException:
        pass
