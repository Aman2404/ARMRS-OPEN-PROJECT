#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Get the urdf file
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL
    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        model_folder,
        'model.sdf'
    )


    my_package_dir = get_package_share_directory('turtlebot3_gazebo')
    world_path = os.path.join(my_package_dir, 'worlds', 'workshop_example.world')


    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time', default='true')}.items()
    )

    ld = LaunchDescription()

    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)

    number_of_robots = 3  # Set this to the number of robots you want to spawn

    robot_positions = [(0.03,-0.64), (1.28, 0.45), (-0.50, 0.95)]  # Add more positions as needed

    for robot_id in range(1, number_of_robots + 1):
        ns = f'robot{robot_id}'
        urdf_path = os.path.join(get_package_share_directory('turtlebot3_gazebo'), "models", model_folder, 'model.sdf')

            # Get the position for this robot
        x_pose, y_pose = robot_positions[robot_id - 1]
        spawn_entity = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            output='screen',
            arguments=[
                '-entity', f'turtlebot3_{robot_id}',
                '-file', urdf_path,
                '-robot_namespace', ns,
                '-x', str(x_pose),
                '-y', str(y_pose),
                '-z', '0.01',
            ],
            name=f'spawn_entity_{ns}'
        )

        ld.add_action(spawn_entity)
            
        
    return ld
