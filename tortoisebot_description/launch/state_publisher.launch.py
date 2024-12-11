from ament_index_python import get_package_share_directory
import launch
import os
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    if "NUM_ROBOTS" in os.environ:
        num_robots = int(os.environ["NUM_ROBOTS"])
    else:
        num_robots = 1

    use_sim_time = LaunchConfiguration('use_sim_time')
    default_model_path = os.path.join(get_package_share_directory('tortoisebot_description'), 'models/urdf/tortoisebot_simple.xacro')
    
    nodes = []

    for i in range(num_robots):
        nodes.append(launch_ros.actions.Node(
            package='robot_state_publisher',
            executable=f'robot_state_publisher',
            namespace=f'tortoisebot_simple_{i}',
            parameters=[{'use_sim_time': use_sim_time},{'robot_description': ParameterValue(Command(['xacro ', LaunchConfiguration('model'), f" namespace:=tortoisebot_simple_{i}"]),value_type=str)}]
        ))
        nodes.append(launch_ros.actions.Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            namespace=f'tortoisebot_simple_{i}',
            name=f'joint_state_publisher_{i}',
            parameters= [{'use_sim_time': use_sim_time}],
            remappings=[('joint_states', f'tortoisebot_simple_{i}/joint_states')]
        ))
        nodes.append(launch_ros.actions.Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f"static_transform_publisher_{i}",
            namespace=f'tortoisebot_simple_{i}',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=["0", "0", "0", "0", "0", "0", "world", f"tortoisebot_simple_{i}/odom"]
        ))

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='False',
                                    description='Flag to enable use_sim_time'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        # robot_state_publisher_node,
        # joint_state_publisher_node
        *nodes,
    ])
