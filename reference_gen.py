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

class TrajectoryGeneration:
    def __init__(self, node_name='trajectory_gen', subscriber='mavros/global_position/local' , publisher='mavros/JointTrajectory'):

        rospy.init_node(node_name, anonymous=True)

        # Define suscribers & publishers
        rospy.Subscriber(subscriber, String, self.callback)
        self.pub = rospy.Publisher(publisher, JointTrajectory, queue_size=10)

        # Wait until the current location is known!!!
        self.current_state = [.2, math.pi/2]
        self.FREQUENCY = 100. # [Hz]

        self.r_discretized = [self.current_state[0]]
        self.b_discretized = [self.current_state[1]]

        self.PUBLISH_RATE = 10 # publisher frequency
        self.WINDOW_FRAME = .5 # publish future states comprise in the window time [s]

        self.is_filtered = False

    def discretise_trajectory(self, parameters=[]):

        start = time()

        r1 = self.r_discretized[-1]
        b1 = self.b_discretized[-1]

        if parameters[0] == 'vector':

            steps = int(parameters[1] * self.FREQUENCY)

            r = np.linspace(r1, parameters[2][0], steps, endpoint=True)
            b = np.linspace(b1, parameters[2][1], steps, endpoint=True)

        # elif parameters[0] == 'circle':
        # elif parameters[0] == 'hover':
        #elif parameters[0] == 'square':
        #elif parameters[0] == 'inf':

        self.r_discretized.extend(r[1:])
        self.b_discretized.extend(b[1:])

        print('discretise_trajectory() runs in {} s'.format(time() - start))

    # def constraint_trajectory_to_box(self):

    def generate_states(self):

        start = time()

        self.vr_discretized = [.0]
        self.vb_discretized = [.0]
        self.ar_discretized = [.0]
        self.ab_discretized = [.0]
        self.ti_discretized = [.0]

        for s,_ in enumerate(self.r_discretized[1:]):
            self.vr_discretized.append((self.r_discretized[s+1] - self.r_discretized[s]) * self.FREQUENCY)
            self.vb_discretized.append((self.b_discretized[s+1] - self.b_discretized[s]) * self.FREQUENCY)
            self.ar_discretized.append((self.vr_discretized[-1] - self.vr_discretized[-2]) * self.FREQUENCY)
            self.ab_discretized.append((self.vb_discretized[-1] - self.vb_discretized[-2]) * self.FREQUENCY)
            self.ti_discretized.append((s + 1.) / self.FREQUENCY)

        print('generate_states() runs in {} s'.format(time() - start))

    # def generate_states_filtered(self):

    # def generate_states_sg_filtered(self, window_length=5, polyorder=2, deriv=0, delta=1.0):

    def plot_trajectory_extras(self):

        start = time()

        fig = plt.figure(figsize=(16,8))

        ax2 = fig.add_subplot(221)
        ax2.plot(self.ti_discretized, self.r_discretized, color='blue', label='r_desired')
        plt.legend()
        plt.title('Cable length')

        ax3 = fig.add_subplot(223)
        ax3.plot(self.ti_discretized, self.b_discretized, color='red', label='b_desired')
        plt.legend()
        plt.title('Elevation angle')

        ax4 = fig.add_subplot(222)
        ax4.plot(self.ti_discretized, self.vr_discretized, color='blue', label='vr_desired')
        ax4.plot(self.ti_discretized, self.ar_discretized, color='red', label='ar_desired')
        plt.legend()
        plt.title('Cable velocity and acceleration')

        ax5 = fig.add_subplot(224)
        ax5.plot(self.ti_discretized, self.vb_discretized, color='blue', label='vb_desired')
        ax5.plot(self.ti_discretized, self.ab_discretized, color='red', label='ab_desired')
        plt.legend()
        plt.title('Elevation angle velocity and acceleration')

        print('plot_trajectory_extras() runs in {} s'.format(time() - start))

        fig.tight_layout()
        plt.show()

    # def plot_trajectory_extras_filtered(self):

    def start(self):
            start = rospy.Time.now()
            rate = rospy.Rate(self.PUBLISH_RATE)
            s = 0
            while not (rospy.is_shutdown() or s >= len(self.r_discretized)):

                # Build JointTrajectory message
                header = Header()
                header.seq = s
                header.stamp = rospy.get_rostime()
                header.frame_id = 'inertial frame'

                joint_trajectory_msg = JointTrajectory()
                joint_trajectory_msg.header = header
                joint_trajectory_msg.joint_names = ['t', 't1']

                # Build JointTrajectoryPoint
                for i in range(min(self.WINDOW_FRAME, len(self.r_discretized) - s)):
                    joint_trajectory_point = JointTrajectoryPoint()
                    joint_trajectory_point.positions = [self.r_discretized[s+i], self.b_discretized[s+i]]
                    joint_trajectory_point.velocities = [self.vr_discretized[s+i], self.vb_discretized[s+i]]
                    joint_trajectory_point.accelerations = [self.ar_discretized[s+i], self.ab_discretized[s+i]]
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

    def callback(self, position):
        pass

if __name__ == '__main__':

    node_name='trajectory_gen_node'
    subscriber='mavros/vision_pose/pose'
    publisher='mavros/JointTrajectory'

    try:
        trajectory_object = TrajectoryGeneration(node_name=node_name, subscriber=subscriber , publisher=publisher)

        ########################################################################
        # Configuration
        trajectory_object.FREQUENCY = 10 # [Hz]

        # Define trajectory shape/vertices in NED frame
        trajectory_object.discretise_trajectory(parameters=['vector', 30, [.2, math.pi/2]])
        trajectory_object.discretise_trajectory(parameters=['vector', 45, [5., math.pi/3]])
        trajectory_object.discretise_trajectory(parameters=['vector', 30, [5., math.pi/3]])
        trajectory_object.discretise_trajectory(parameters=['vector', 30, [.2, math.pi/2]])
        ########################################################################

        # Limit the trajectory to the BOX_LIMIT
        # trajectory_object.constraint_trajectory_to_box()

        # Generate the list of states - start by generating the states and then filter them
        trajectory_object.generate_states()
        # trajectory_object.generate_states_filtered()
        # trajectory_object.generate_states_sg_filtered(window_length=9, polyorder=3)

        # Plot the trajectory
        trajectory_object.plot_trajectory_extras()
        # trajectory_object.plot_trajectory_extras_filtered()

        # Publish trajectory states
        # trajectory_object.start()

    except rospy.ROSInterruptException:
		pass
