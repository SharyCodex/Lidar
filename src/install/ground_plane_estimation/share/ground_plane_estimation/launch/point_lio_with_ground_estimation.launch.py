from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Declare arguments
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Flag to launch RViz for visualization.')
    
    point_lio_rviz_arg = DeclareLaunchArgument(
        'point_lio_rviz', default_value='false',
        description='Flag to launch Point-LIO RViz (separate from ground estimation RViz).')

    # Include Point-LIO launch
    point_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('point_lio'),
                'launch', 'mapping_unilidar_l1.launch.py'
            ])
        ]),
        launch_arguments={
            'rviz': LaunchConfiguration('point_lio_rviz')
        }.items()
    )

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

    # RViz node for ground plane estimation visualization
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
        point_lio_rviz_arg,
        point_lio_launch,
        ground_plane_node,
        rviz_node,
    ])

    return ld
