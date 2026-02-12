import math
import rclpy
from unitree_sdk2_python.unitree_sdk2py.idl.geometry_msgs import Odometry_
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class Nav2Point(Node):
    def __init__(self):
        super().__init__('nav2point')
        self.x = self.y = self.yaw = 0.0
        self.timeout = 0
        self.pointReachedThreshold = 0.1
        self.idx = 0 
        self.have_pose = False

        self.odomSubscriber = self.create_subscription(Odometry_, '/odommodestate', self.odomHandler, 10)
        self.path = [Coordinate(1.0, 1.0, 0.0)]


    def odomHandler(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = self.quantToYaw(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        self.have_pose = True
        

    def quantToYaw(self, x, y, z, w):
        s = 2.0 * (w * z + x * y)
        c = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(s, c)
    
    def main(self):
        try:
            if (len(self.path) == 0):
                self.get_logger().info("No path to follow")
                return
            
            else:
                if (self.have_pose)
                    target = self.path[self.idx]
    


class Coordinate():
    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x = x
        self.y = y