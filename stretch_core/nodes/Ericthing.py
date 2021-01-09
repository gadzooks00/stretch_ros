#!/usr/bin/env python

from __future__ import print_function

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

from sensor_msgs.msg import PointCloud2

from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

import math
import time
import threading
import sys
import tf2_ros
import argparse as ap        

import hello_helpers.hello_misc as hm
import stretch_funmap.navigate as nv


import message_filters
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point

class ButtonPressingNode(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)
        self.rate = 10.0
        self.joint_states = None
        self.joint_states_lock = threading.Lock()
        self.move_base = nv.MoveBase(self)
        self.letter_height_m = 0.05
        self.letter_top_lift_m = 1.08 #1.05

    def main(self):
        hm.HelloNode.main(self, 'button_pressing', 'button_pressing', wait_for_first_pointcloud=False)
        
        
        rospy.wait_for_service('/detect_button/marker_id')
        rospy.loginfo('Node ' + self.node_name + ' connected to /detect_button/marker_id.')
        self.marker_id_service = rospy.ServiceProxy('/detect_button/marker_id', Trigger)
        variable = TriggerRequest()
        placeholder = self.marker_id_service(variable) 


        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    try:
        parser = ap.ArgumentParser(description='ButtonPressing demo for stretch.')
        args, unknown = parser.parse_known_args()
        node = ButtonPressingNode()
        node.main()
    except KeyboardInterrupt:
        rospy.loginfo('interrupt received, so shutting down')
