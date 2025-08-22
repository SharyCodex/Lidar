from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare the RViz argument
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Flag to launch RViz for visualization.')

    # Ground plane estimation node
    ground_plane_node = Node(
        package='ground_plane_estimation',
        executable='ground_plane_estimation_node',
        name='ground_plane_estimation',
        output='screen',
        parameters=[PathJoinSubstitution([
            FindPackageShare('ground_plane_estimation'),
            'config', 'ground_plane_estimation.yaml'
        ])],
    )

    # RViz node for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('ground_plane_estimation'),
            'rviz', 'ground_plane_estimation.rviz'
        ])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        prefix='nice'
    )

    # Assemble the launch description
    ld = LaunchDescription([
        rviz_arg,
        ground_plane_node,
        rviz_node,
    ])

    return ld
