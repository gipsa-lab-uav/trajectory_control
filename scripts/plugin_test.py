#!/usr/bin/env python2

import math
# import numpy as np
# from scipy import signal
# from matplotlib import pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
# from timeit import default_timer as time

import rospy
from std_msgs.msg import String, Header, Float64, Int32
from geometry_msgs.msg import Vector3, Twist
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# from package_name.msgs import message_name
# from mavros_msgs.msgs import CommandMotorSpeed
# from physics_msgs.msgs import Wind


class WingController:
    def __init__(self, node_name="wing_control", subscriber="/gazebo/command/wing_speed", publisher="command_wing_speed"):
        rospy.init_node(node_name, anonymous=True)
        self.command_sub = rospy.Subscriber(subscriber, Float64, self.callback)
        self.failure_sub = rospy.Subscriber("/gazebo/wing_failure_num", Int32, self.callback)

        self.pub = rospy.Publisher(publisher, Float64, queue_size=10)
        self.pub_rate = rospy.Rate(1)

    def start(self):
        time_then = rospy.get_time()
        while not rospy.is_shutdown():
            cmd_msg = self.generateCommand(time_then)
            rospy.loginfo(cmd_msg)
            self.pub.publish(cmd_msg)
            self.pub_rate.sleep()
        print("\rExiting")

    def generateCommand(self, time_then):
        return math.sin(0.1*(rospy.get_time() - time_then))*200 + 600

    def callback(self, data):
        # rospy.loginfo("I heard %s", data.data)
        pass


if __name__ == "__main__":

    node_name = "wing_control_node"
    subscriber = "/gazebo/command/wing_speed"
    publisher = "command_wing_speed"

    try:
        wing_control = WingController(node_name=node_name, subscriber=subscriber, publisher=publisher)
        wing_control.start()

    except rospy.ROSInterruptException:
        pass
