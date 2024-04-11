import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int64, Bool
from enum import Enum

import launch
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription

def load_launch(launch_package, launch_name):
    launch_description = launch.LaunchDescription()
    launch_description.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([FindPackageShare(launch_package), '/launch', '/'+launch_name+'.py'])
        )
    )

    return launch_description

class Mode(Enum):
    LANE = 1
    INTERSECTION = 2
    OBSTACLES = 3
    PARKING = 4

class node(Node):
    def __init__(self):
        super().__init__('core_node')
        self.subscription_signs = self.create_subscription(String, '/detect/signs', self.signs_callback, 1)
        self.subscription_traffic_light = self.create_subscription(String, '/detect/traffic_light', self.traffic_light_callback, 1)
        self.subscription_yellow_fraction = self.create_subscription(Int64, '/detect/yellow_fraction', self.yellow_fraction_callback, 1)
        self.subscription_white_fraction = self.create_subscription(Int64, '/detect/white_fraction', self.white_fraction_callback, 1)
        self.subscription_parking_done = self.create_subscription(Bool, '/parking_done', self.parking_done_callback, 1)
        self.subscription_avoidance_done = self.create_subscription(Bool, '/avoidance_done', self.avoidance_done_callback, 1)

        self.subscription_signs
        self.subscription_traffic_light
        self.subscription_yellow_fraction
        self.subscription_white_fraction
        self.subscription_parking_done
        self.subscription_avoidance_done 

        self.publisher_which_line = self.create_publisher(Int64, '/detect/lane_mode', 1)
        self.pub_stop = self.create_publisher(Bool, '/control/go_stop', 1)
        self.pub_lane_toggle = self.create_publisher(Bool, '/detect/lane_toggle', 1)
        
        self.mode = Mode.LANE
        self.traffic_light = String()
 
        self.parking_launch_ls = launch.LaunchService()
        parking_launch_description = load_launch('control', '[停車launch]')
        self.parking_launch_ls.include_launch_description(parking_launch_description)

        self.avoidance_launch_ls = launch.LaunchService()
        avoidance_launch_description = load_launch('control', '[避障launch]')
        self.avoidance_launch_ls.include_launch_description(avoidance_launch_description)

    def parking_done_callback(self, msg):
        self.parking_done = msg.data

        if self.parking_done == True:
            self.parking_launch_ls.shutdown()

    def avoidance_done_callback(self, msg):
        self.avoidance_done = msg.data

        if self.avoidance_done == True:
            self.avoidance_launch_ls.shutdown()
        
    def yellow_fraction_callback(self, msg):
        self.yellow_fraction = msg.data

    def white_fraction_callback(self, msg):
        self.white_fraction = msg.data

    def traffic_light_callback(self, msg):
        self.traffic_light = msg.data

    def signs_callback(self, msg):
        if msg.data == 'Intersection_sign':
            self.get_logger().info('Received: Intersection sign')
            pass

        elif msg.data == 'Left_sign':
            self.get_logger().info('Received: LEFT sign')
            pass

        elif msg.data == 'Right_sign':
            self.get_logger().info('Received: RIGHT sign')
            pass

        elif msg.data == 'Stop_sign':
            self.get_logger().info('Received: STOP sign')
            pass

        elif msg.data == 'Obstacle_sign':
            self.get_logger().info('Received: OBSTACLE sign')
            pass

        elif msg.data == 'Park_sign':
            self.get_logger().info('Received: PARKING sign')
            pass

        elif msg.data == 'Stop_Bar_sign':
            self.get_logger().info('Received: STOPBAR sign')
            pass

        elif msg.data == 'Tunnel_sign':
            self.get_logger().info('Received: TUNNEL sign')
            pass
        else:
            self.get_logger().info('Received: NONE sign')
            pass


#==================================mode select===============================================
        if self.mode.value == Mode.LANE.value:
            self.get_logger().info('Mode: LANE')
            pass

        elif self.mode.value == Mode.INTERSECTION.value:
            self.get_logger().info('Mode: INTERSECTION')
            pass

        elif self.mode.value == Mode.OBSTACLES.value:
            self.get_logger().info('Mode: OBSTACLES')
            pass

        elif self.mode.value == Mode.PARKING.value:
            pass

def main(args=None):
    rclpy.init(args=args)
    core_node = node()
    rclpy.spin(core_node)
    core_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
