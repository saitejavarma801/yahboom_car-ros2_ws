import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    
    joy_params = PathJoinSubstitution(
        [
            FindPackageShare("my_odrive_botwheel_explorer"),
            "config",
            "joystick.yaml",
        ]
    )


    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[joy_params],
    )
    
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        parameters=[joy_params],
        remappings=[('/cmd_vel','/cmd_vel_joy')]
    )

    twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        parameters=[{'use_sim_time': False}],
        remappings=[('/cmd_vel_in','/cmd_vel_joy'),
                    ('/cmd_vel_out','/botwheel_explorer/cmd_vel')]
    )
    
    nodes = [
        joy_node,
        teleop_node,
        twist_stamper,
    ]

    return LaunchDescription(declared_arguments + nodes)
