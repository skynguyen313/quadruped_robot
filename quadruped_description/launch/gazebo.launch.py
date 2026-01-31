from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_quadruped = get_package_share_directory('quadruped_description')
    pkg_gazebo = get_package_share_directory('gazebo_ros')

    urdf_file = os.path.join(
        pkg_quadruped,
        'urdf',
        'quadruped_description.urdf'
    )

    gazebo_launch = os.path.join(
        pkg_gazebo,
        'launch',
        'gazebo.launch.py'
    )

    return LaunchDescription([

        # Launch Gazebo empty world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch)
        ),

        # Static TF: base_link -> base_footprint
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_footprint_base',
            arguments=[
                '--x', '0.0',
                '--y', '0.0',
                '--z', '0.0',
                '--roll', '0.0',
                '--pitch', '0.0',
                '--yaw', '0.0',
                '--frame-id', 'base_footprint',
                '--child-frame-id', 'base_link',
            ]
        ),

        # Spawn robot into Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_quadruped',
            arguments=[
                '-entity', 'quadruped_description',
                '-file', urdf_file
            ],
            output='screen'
        ),

        # Fake joint calibration flag
        ExecuteProcess(
            cmd=[
                'ros2', 'topic', 'pub',
                '/calibrated',
                'std_msgs/msg/Bool',
                '{data: true}'
            ],
            output='screen'
        )
    ])