import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node

def generate_launch_description():
    package_name="robot"
    world_name = 'willowgarage' #maze_1 or maze_2
    
    nav2_params_file = os.path.join(get_package_share_directory(package_name),'config', 'nav2_params2.yaml')

    map_file = os.path.join(get_package_share_directory(package_name), 'map', f'{world_name}', 'map.yaml')

    nav2 = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py'
                    )]), launch_arguments={'use_sim_time': 'true', 
                                            'autostart' : 'true',
                                            'map' : map_file,
                                            'params_file' : nav2_params_file
                                        }.items()
    )

    return LaunchDescription([
        nav2
    ])