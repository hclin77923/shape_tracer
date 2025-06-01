from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    xarm_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("xarm_moveit_config"),
                    "launch",
                    "xarm7_moveit_fake.launch.py",
                ]
            )
        ),
    )

    return LaunchDescription(
        [
            xarm_moveit_launch,
            #######################
            # ADD YOUR NODES HERE #
            #######################
            Node(
            package='shape_tracer',
            executable='shape_tracer_node',
            name='shape_tracer',
            output='screen',
            parameters=[{'shape_file': 'config/shapes.json'}]
        )
        ]
    )
