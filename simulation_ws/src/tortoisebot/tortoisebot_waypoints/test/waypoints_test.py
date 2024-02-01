#! /usr/bin/env python3

from nav_msgs.msg import Odometry
import rospy
import unittest
import rostest
import math
import actionlib
from geometry_msgs.msg import Point, Quaternion
from course_web_dev_ros.msg import WaypointActionAction, WaypointActionGoal
import tf.transformations as tf
from tf import transformations


class WaypointsServerTest(unittest.TestCase):

    def setUp(self):
        rospy.init_node('test_node')
        self.ac = actionlib.SimpleActionClient("tortoisebot_as", WaypointActionAction)
        self.ac.wait_for_server()
        self.subs = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.desti = WaypointActionGoal()
        self.curr = Point()
        self.curr_angle = Quaternion()
        self._yaw=0.0
        self.send()

    def odom_callback(self, msg):
        self.curr = msg.pose.pose.position
        self.curr_angle = msg.pose.pose.orientation
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = transformations.euler_from_quaternion(quaternion)
        self._yaw = euler[2]

    def send(self):
        self.desti.position.x = 0.5
        self.desti.position.y = 0.5
        self.ac.send_goal(self.desti)
        self.ac.wait_for_result(rospy.Duration(60))
        self.ar = self.ac.get_result()

    def test_position(self):
        x_error = abs(self.desti.position.x - self.curr.x)
        y_error = abs(self.desti.position.y - self.curr.y)
        dist_error_permissible = 0.4
        self.assertTrue(x_error <= dist_error_permissible and y_error <= dist_error_permissible)

    def test_orientation(self):
        yaw = math.atan2(self.desti.position.y - self.curr.y, self.desti.position.x - self.curr.x)
        yaw_error = abs(yaw - self._yaw)
        angle_error_permissible = 4.0
        self.assertTrue(yaw_error <= angle_error_permissible)

if __name__ == '__main__':
    rostest.rosrun("tortoisebot_waypoints", "waypoints_test", WaypointsServerTest)