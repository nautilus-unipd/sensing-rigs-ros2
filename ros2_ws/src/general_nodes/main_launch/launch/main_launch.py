from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():


    # Get lifecycle launcher
    lifecycle_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lifecycle_manager'),
                'launch',
                'lifecycle_manager_launch.py'
            )
        )
    )

    # Get Preprocessing node
    preprocessing_node = Node(
        package='sensing_preprocessing', 
        executable='PreprocessingNode',
        name='preprocessing_node',
        output='screen'
    )

    # Get cv_algorithms node launch
    cv_algorithms_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('cv_algorithms'),
                'launch',
                'cv_algorithms_launch.py'
            )
        )
    )

    # Get normalizator node launch
    normalizator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('normalizator'),
                'launch',
                'normalizator_node_launch.py'
            )
        )
    )

    server_debug = Node(
        package='debug_server',
        executable='debug_server',
        name='server',
        output='screen'
    )

    server_results = Node(
        package='results_server',
        executable='results_server',
        name='server',
        output='screen'
    )

    return LaunchDescription([
        lifecycle_launch,
        preprocessing_node,
        cv_algorithms_launch,
        normalizator_launch,
        server_debug,
        server_results
    ])
