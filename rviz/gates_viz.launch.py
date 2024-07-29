import os

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    rviz_config = os.path.join(os.getcwd(), 'rviz', 'gates_config.rviz')
    drone_0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('as2_visualization'), 'launch'),
            '/as2_visualization.launch.py']),
        launch_arguments={'rviz_config': rviz_config,
                          'namespace': 'cf0', 'use_sim_time': LaunchConfiguration('use_sim_time'),
                          'record_length': LaunchConfiguration('record_length')}.items(),
    )

    drone_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('as2_visualization'), 'launch'),
            '/as2_visualization.launch.py']),
        launch_arguments={'rviz': 'false',
                          'namespace': 'cf1', 'use_sim_time': LaunchConfiguration('use_sim_time'),
                          'record_length': LaunchConfiguration('record_length')}.items(),
    )

    sdf_file = os.path.join(get_package_share_directory(
        'as2_gazebo_assets'),
        'models', 'aruco_gate_1', 'aruco_gate_1.sdf')

    with open(sdf_file, 'r', encoding='utf-8') as infp:
        gate_desc = infp.read()

    gate_0_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='gate_0',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'robot_description': gate_desc}
        ]
    )

    gate_1_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='gate_1',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'robot_description': gate_desc}
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulation time.'),
        DeclareLaunchArgument('record_length', default_value='500',
                              description='Length for last poses.'),
        drone_0,
        drone_1,
        gate_0_state_publisher,
        gate_1_state_publisher
    ])
