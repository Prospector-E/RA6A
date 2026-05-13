from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
import os

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory('ra6a')
    urdf_file = os.path.join(pkg_share, 'urdf', 'ra6a.urdf')

    robot_description = ParameterValue(
        Command(['cat ', urdf_file]),
        value_type=str
    )

    return LaunchDescription([

        # Static transform map -> base_link
        #Node(
        #    package='tf2_ros',
        #    executable='static_transform_publisher',
        #    arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link']
       # ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),

        # Joint State Publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        )
    ])
