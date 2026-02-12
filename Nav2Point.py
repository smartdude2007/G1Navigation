import math
import rclpy
import unitree_sdk2py
from rclpy.node import Node
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry_


class Nav2Point(Node):
    def __init__(self):
        super().__init__('nav2point')
        self.x = self.y = self.yaw = 0.0

        self.path = self.create_subscriber(Odometry_, '/odom', self.odomHandler, 10)

    def odomHandler(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = self.quantToYaw(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        

    def quantToYaw(self, x, y, z, w):
        s = 2.0 * (w * z + x * y)
        c = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(s, c)