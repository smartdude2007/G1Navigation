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
        self.directionReachedThreshold = 2
        self.idx = 0 
        self.have_pose = False
        self.max_v = 0.7
        self.max_w = 0.6
        self.wp = 0.5
        self.vp = 0.5

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
        return math.degrees(math.atan2(s, c))
    
    def main(self):
        try:
            if (len(self.path) == 0):
                self.get_logger().info("No path to follow")
                return
            
            else:
                while(self.have_pose):
                    turned = False
                    while(turned == False):
                        targetYaw = self.path[self.idx].yaw
                        yawError = 0
                        if (abs(targetYaw - self.yaw)>180):
                            if (targetYaw > self.yaw):
                                yawError = targetYaw - self.yaw - 360
                            else:
                                yawError = targetYaw - self.yaw + 360
                        else:
                            yawError = targetYaw - self.yaw
                        
                        self.get_logger().infor("Yaw error: %.2f" % yawError)

                        if (abs(yawError) < self.directionReachedThreshold):
                            turned = True
                            self.get_logger().info("Direction reached, start moving")

                        if (yawError > 0):
                            w = min(self.max_w, self.wp * yawError)
                        else:
                            w = max(-self.max_w, self.wp * yawError)

                        self.get_logger().info("Turning, angular velocity: %.2f" % w)

                    while(turned == True):
                        targetX = self.path[self.idx].x
                        targetY = self.path[self.idx].y
                        distance = math.hypot(targetX - self.x, targetY - self.y)
                        self.get_logger().info("Distance to target: %.2f" % distance)

                        if (distance < self.pointReachedThreshold):
                            self.idx += 1
                            if (self.idx >= len(self.path)):
                                self.get_logger().info("Path complete")
                                break
                            else:
                                turned = False
                                break
                        
                        v = min(self.max_v, self.vp * distance)
                        self.get_logger().info("Moving, linear velocity: %.2f" % v)

                        


    


class Coordinate():
    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x = x
        self.y = y
