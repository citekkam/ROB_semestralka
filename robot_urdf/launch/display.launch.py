from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to URDF - use the correct install path
    pkg_share = get_package_share_directory('crs_a465_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'crs_a465.urdf')

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_gui',
            default_value='true',
            description='Flag to enable joint_state_publisher_gui'
        ),

        # Joint State Publisher (with GUI sliders)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=None
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=[urdf_file]
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'crs_a465.rviz')]
        ),
    ])
