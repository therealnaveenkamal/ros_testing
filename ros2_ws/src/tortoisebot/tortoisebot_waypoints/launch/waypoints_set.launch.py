import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import launch_ros

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    gazebo_launch_dir = os.path.join(get_package_share_directory('tortoisebot_bringup'), 'launch')

    gazebo_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_launch_dir, 'bringup.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items())

    action_server = Node(
                        package='tortoisebot_waypoints',
                        executable='action_server',
                        name='tortoisebot_as'        
                    )
    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                    description='Flag to enable use_sim_time'),
        gazebo_launch_cmd,
        action_server, 
    ])