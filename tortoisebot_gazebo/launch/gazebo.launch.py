from ament_index_python import get_package_share_directory
import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
from launch_ros.actions import Node
import os
import sys

positions = [(0, 0), (1, 1), (2, 1)]

def generate_launch_description():
    # world_path= os.path.join(get_package_share_directory('ttb_description'), 'models/worlds/house_env.world'),
    # world_path=os.path.join(get_package_share_directory('tortoisebot_description'), 'worlds/big_env.sdf'),
    world_path=os.path.join(get_package_share_directory('tortoisebot_gazebo'), 'worlds/room2.sdf'),
    
    if "NUM_ROBOTS" in os.environ:
        num_robots = int(os.environ["NUM_ROBOTS"])
    else:
        num_robots = 1

    use_sim_time = LaunchConfiguration('use_sim_time')
    
    print(num_robots)
    
    spawn_nodes = []
    for i in range(num_robots):
        robot_name = f'tortoisebot_simple_{i}'
        spawn_nodes.append(Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            # namespace=robot_name,
            arguments=['-entity', robot_name, '-robot_namespace', robot_name, '-topic', f'{robot_name}/robot_description', "-x", str(positions[i][0]), "-y", str(positions[i][1])],
            parameters= [{'use_sim_time': use_sim_time}],
            output='screen'))

    return launch.LaunchDescription([
        launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 
                                            'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so',world_path], 
                                           output='screen'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                description='Flag to enable use_sim_time'),
        # Node(
        #     package='gazebo_ros',
        #     executable='spawn_entity.py',
        #     arguments=['-entity', 'tortoisebot_simple', '-topic', 'robot_description'],
        #     parameters= [{'use_sim_time': use_sim_time}],
        #     output='screen'),
        *spawn_nodes,
    ])
    