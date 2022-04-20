from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, Shutdown
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

    gazebo_ros_share = FindPackageShare('gazebo_ros')

    return LaunchDescription([
        # Gazebo
        DeclareLaunchArgument('world', default_value=PathJoinSubstitution([FindPackageShare('loc_sim'), 'worlds', 'test.world'])),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Set to "false" to run headless.'),

        DeclareLaunchArgument('server', default_value='true',
                              description='Set to "false" not to run gzserver.'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([gazebo_ros_share, '/launch', '/gzserver.launch.py']),
            condition=IfCondition(LaunchConfiguration('server')),
            launch_arguments=[('world', LaunchConfiguration('world'))]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([gazebo_ros_share, '/launch', '/gzclient.launch.py']),
            condition=IfCondition(LaunchConfiguration('gui'))
        ),

        # Localization
        Node(
            package='loc_sim',
            executable='localize',
        ),

        # RViz
        ExecuteProcess(
            cmd=['rviz2', '-d', PathJoinSubstitution([FindPackageShare('loc_sim'), 'config', 'loc_sim.rviz'])],
            on_exit=Shutdown()
        )

    ])