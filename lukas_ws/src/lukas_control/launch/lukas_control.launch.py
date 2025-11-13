from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    robot_description = Command([
        "xacro ",
        PathJoinSubstitution([FindPackageShare("lukas_control"), "description", "lukas.xacro.urdf"])
    ])

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
             {"use_realtime_threads": False},
            {"robot_description": robot_description},
            PathJoinSubstitution([FindPackageShare("lukas_control"), "config", "controllers.yaml"])
        ],
        output="screen"
    )

    # Spawners (ejemplos)
    joint_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen"
    )

    example_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller"],
        output="screen"
    )

    # Spawn controllers after manager is up
    delay_spawners = TimerAction(
    period=3.0,
    actions=[joint_broadcaster_spawner]
    )

    return LaunchDescription([
        controller_manager,
        delay_spawners
    ])
