#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class ControlLane(Node):
    def __init__(self):
        super().__init__('control_lane')
        self.sub_lane = self.create_subscription(Float64, '/control/lane', self.cbFollowLane, 1)
        self.sub_max_vel = self.create_subscription(Float64, '/control/max_vel', self.cbGetMaxVel, 1)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 1)

        self.lastError = 0
        self.MAX_VEL = 50.0

    def cbGetMaxVel(self, max_vel_msg):
        self.MAX_VEL = max_vel_msg.data

    def cbFollowLane(self, desired_center):
        center = desired_center.data

        error = center - 320

        # true value
        Kp = 0.09 

        Kd = 0.009

        angular_z = Kp * error + Kd * (error - self.lastError)
        self.lastError = error
        twist = Twist()
        twist.linear.x = min(self.MAX_VEL * ((1 - abs(error) / 1200) ** 2.2), 40.0)
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = -max(angular_z, -2.5) if angular_z < 0 else -min(angular_z, 2.5)
        self.pub_cmd_vel.publish(twist)

    def shutdown(self):
        self.get_logger().info("Shutting down. cmd_vel will be 0")
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.pub_cmd_vel.publish(twist) 

def main(args=None):
    rclpy.init(args=args)
    node = ControlLane()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
