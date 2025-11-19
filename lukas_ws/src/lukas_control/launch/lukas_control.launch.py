from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    robot_description = Command([
        "xacro ",
        PathJoinSubstitution([
            FindPackageShare("lukas_control"),
            "description",
            "lukas.xacro.urdf"
        ])
    ])

    controllers_yaml = PathJoinSubstitution([
        FindPackageShare("lukas_control"),
        "config",
        "controllers.yaml"
    ])

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen"
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        parameters=[
            {"robot_description": robot_description},
            controllers_yaml
        ],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        output="both"
    )

    joint_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    return LaunchDescription([
        robot_state_pub,
        controller_manager,
        TimerAction(period=2.0, actions=[joint_broadcaster_spawner]),
        TimerAction(period=3.0, actions=[trajectory_controller_spawner]),
    ])
