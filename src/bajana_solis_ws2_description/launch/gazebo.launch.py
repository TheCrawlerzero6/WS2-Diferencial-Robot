"""Inicia Gazebo Harmonic, el robot y los puentes de ROS 2."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    """Carga los recursos instalados y crea una instancia del robot."""
    share = Path(get_package_share_directory('bajana_solis_ws2_description'))
    gz_share = Path(get_package_share_directory('ros_gz_sim'))
    description = xacro.process_file(str(share / 'urdf/bajana_solis_ws2.xacro')).toxml()
    gui = LaunchConfiguration('gui')
    software_rendering = LaunchConfiguration('renderizado_software')
    world = str(share / 'worlds/ws2.sdf')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('renderizado_software', default_value='true'),
        GroupAction(scoped=True, actions=[
            SetEnvironmentVariable(
                name='LIBGL_ALWAYS_SOFTWARE', value='1',
                condition=IfCondition(software_rendering),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(gz_share / 'launch/gz_sim.launch.py')),
                launch_arguments={
                    'gz_args': PythonExpression([
                        "'-r -v 3 ' + ('' if '", gui,
                        "'.lower() == 'true' else '-s ') + '", world, "'",
                    ]),
                    'on_exit_shutdown': 'true',
                }.items(),
            ),
        ]),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            parameters=[{'robot_description': description, 'use_sim_time': True}],
            output='screen',
        ),
        # create espera el servicio del mundo; no se presupone un retardo de arranque.
        Node(
            package='ros_gz_sim', executable='create',
            arguments=[
                '-world', 'ws2', '-topic', '/robot_description',
                '-name', 'ws2_robot', '-allow_renaming', 'false',
                '-x', '0', '-y', '0', '-z', '0.002', '-Y', '0',
            ],
            output='screen',
        ),
        Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            parameters=[{'config_file': str(share / 'config/ros_gz_bridge_gazebo.yaml')}],
            output='screen',
        ),
    ])
