from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_name = 'lukas_description'
    pkg_share = FindPackageShare(pkg_name)

    # Paths
    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'lukas.xacro'])
    rviz_file = PathJoinSubstitution([pkg_share, 'rviz', 'model.rviz'])

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

    rviz2 =  Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_file]
        )

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
               {'use_rt_thread': False}  ,
                       {'use_multithread_executor': True} , # usa executor normal,
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
      rviz2,
        robot_state_pub,
        controller_manager,
        TimerAction(period=2.0, actions=[joint_broadcaster_spawner]),
    #TimerAction(period=3.0, actions=[trajectory_controller_spawner]),
    ])
