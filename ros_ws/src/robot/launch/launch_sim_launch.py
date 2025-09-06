import os

from ament_index_python.packages import get_package_share_directory, get_package_share_path


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit, OnProcessStart, OnExecutionComplete

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

    ros2_control = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[diff_drive_spawner, joint_broad_spawner]
        )
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

    nav2_bringup = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','navigation_launch.py'
                )])
    )

    nav2_launch = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[nav2_bringup]
        )
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_path(package_name),'config','nav2_view.rviz')]
    )

    plotjuggler = Node(
        package="plotjuggler",
        executable="plotjuggler",
        arguments=['-l', os.path.join(get_package_share_path(package_name),'config','plot_config.xml')]
    )

    plotjuggler_launch = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[plotjuggler]
        )
    )

    rviz_launch_delayed = TimerAction(period=20.0, actions=[rviz])
    rviz_launch = RegisterEventHandler( 
        event_handler=OnProcessStart( 
            target_action=plotjuggler, 
            on_start=[rviz_launch_delayed] ) )

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        ros2_control,
        nav2_launch,
        rosbag_record,
        encoder_data_record,
        laser_data_record,
        robot_velocity_data_record,
        robot_monitor,  
        plotjuggler_launch,
        rviz_launch_delayed
    ])