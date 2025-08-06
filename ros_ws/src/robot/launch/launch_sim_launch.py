import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import shutil

def generate_launch_description():
    
    shutil.rmtree('src/robot/bag_files/data1', ignore_errors=True)

    package_name='robot'

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','robot_launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control' : 'true'}.items()
    )
    
    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot'],
                        output='screen')

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["diff_controller"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_broad"],
    )

    encoder_data_record = Node(
        package="robot",
        executable="reading_encoder",
        arguments=["reading_encoder"],
        output={'both': 'log'}
    )

    laser_data_record = Node(
        package="robot",
        executable="reading_laser",
        arguments=["reading_laser"],
        output={'both': 'log'}
    )

    robot_velocity_data_record = Node(
        package="robot",
        executable="reading_velocities",
        arguments=["reading_velocities"],
        output={'both': 'log'}
    ) 

    robot_monitor = Node(
        package="robot",
        executable="robot_monitor",
        arguments=["robot_monitor"],
        output={'both': 'log'}
    ) 

    rosbag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', './src/robot/bag_files/data1', '/robot_monitor'],
        output='screen'
    )

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        rosbag_record,
        encoder_data_record,
        laser_data_record,
        robot_velocity_data_record,
        robot_monitor,
    ])