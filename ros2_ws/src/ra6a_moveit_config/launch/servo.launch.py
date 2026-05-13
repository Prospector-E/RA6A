"""
MoveIt Servo launch for RA6A — AR4 approach.

Modeled directly on AR4's joystick_servo.launch.py:
- Launches move_group + servo_node + rviz
- Spawns arm_controller (JTC), NOT forward_position_controller
- Servo outputs JointTrajectory to /arm_controller/joint_trajectory
- Params loaded same way as AR4 (dict, not file path)

Usage:
  ros2 launch ra6a_moveit_config servo.launch.py
  # Then in another terminal:
  python3 ~/keyboard_teleop_servo.py
"""

import os
import yaml

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_name):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_name)
    with open(absolute_file_path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("ra6a", package_name="ra6a_moveit_config")
        .to_moveit_configs()
    )

    pkg_share = get_package_share_directory("ra6a_moveit_config")

    # Planning scene monitor parameters (like AR4)
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description_semantic": True,
    }

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher0",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control node
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            os.path.join(pkg_share, "config", "ros2_controllers.yaml"),
        ],
        output="both",
    )

    # Spawn controllers
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    # KEY: spawn arm_controller (JTC) — like AR4, NOT forward_position_controller
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager"],
    )

    # MoveGroup — use to_dict() like demo.launch.py
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            planning_scene_monitor_parameters,
            {"use_sim_time": False},
        ],
    )

    # Servo node — load params as dict like AR4 does with ParameterBuilder
    servo_yaml = load_yaml("ra6a_moveit_config", "config/servo_params.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        parameters=[
            servo_params,
            {"update_period": 0.01},
            {"planning_group_name": "arm"},
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            planning_scene_monitor_parameters,
            {"is_primary_planning_scene_monitor": False},
            {"use_sim_time": False},
        ],
        output="screen",
    )

    # Auto-start servo after 5 seconds
    start_servo = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2", "service", "call",
                    "/servo_node/start_servo",
                    "std_srvs/srv/Trigger", "{}"
                ],
                output="screen",
            )
        ],
    )

    # RViz
    rviz_config = os.path.join(pkg_share, "config", "moveit.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": False},
        ],
    )

    return LaunchDescription([
        static_tf,
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        move_group,
        servo_node,
        start_servo,
        rviz,
    ])
