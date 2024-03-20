from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    meow_arg = DeclareLaunchArgument('calibration', default_value='False')

    # Define the nodes
    camera_node = Node(
        package='camera',
        executable='camera',
    )
    
    config=os.path.join(get_package_share_directory('autorace'), 'config', 'hsv_parameters_default.yaml')
    # config=os.path.join(get_package_share_directory('autorace'), 'config', 'hsv_parameters_own.yaml')

    detect_node = Node(
        package='autorace',
        executable='detect_lane_v1',
        parameters=[{'calibration': LaunchConfiguration('calibration')}, config],
    )

    # Create the launch description
    ld = LaunchDescription()

    # Add the launch arguments and nodes to the launch description
    ld.add_action(meow_arg)
    ld.add_action(camera_node)
    ld.add_action(detect_node)

    return ld


