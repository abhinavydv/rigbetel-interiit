import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration,PythonExpression
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.conditions import IfCondition

pkg_name = "tortoisebot_slam"

def generate_launch_description():
  prefix_address = get_package_share_directory('tortoisebot_slam') 
  config_directory = os.path.join(prefix_address, 'config')
  # slam_config = 'slam.lua'
  res = LaunchConfiguration('resolution', default='0.05')
  publish_period = LaunchConfiguration('publish_period_sec', default='1.0')
  use_sim_time=LaunchConfiguration('use_sim_time')
  exploration=LaunchConfiguration('exploration')  # slam in exploration MODE

  if "NUM_ROBOTS" in os.environ:
      num_robots = int(os.environ["NUM_ROBOTS"])
  else:
      num_robots = 1

  nodes = []

  turtlebot3_cartographer_prefix = get_package_share_directory(pkg_name)
  for i in range(num_robots):
        robot_name = f'tortoisebot_simple_{i}'
    
        cartographer_config_dir = LaunchConfiguration(pkg_name, default=os.path.join(
                                                  turtlebot3_cartographer_prefix, 'config'))
        configuration_basename = LaunchConfiguration('configuration_basename',
                                                 default=f'slam_config_{i}.lua')

        nodes.append(DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Full path to config file to load'),
        )

        nodes.append(DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Name of lua file for cartographer'))
        
        nodes.append(DeclareLaunchArgument(
          'localization_configuration_basename',
          default_value=configuration_basename,
          description='name of .lua file to be used'
        ),)
        
        nodes.append(Node(
          package='cartographer_ros',
          condition= IfCondition(exploration),
          executable='cartographer_node',
          name=f'as21_cartographer_node_{i}',
          namespace=robot_name,
          arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', configuration_basename
          ],
          parameters= [{'use_sim_time':use_sim_time}],
          output='screen',
        ),)
        nodes.append(Node(
          package='cartographer_ros',
          condition=IfCondition(PythonExpression(['not ', exploration])),
          executable='cartographer_node',
          name=f'as21_cartographer_node_{i}',
          namespace=robot_name,
          arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', configuration_basename
          ],
          parameters= [{'use_sim_time':use_sim_time}],
          output='screen',
        ),)

        nodes.append(Node(
          package='cartographer_ros',
          condition= IfCondition(exploration),
          executable='cartographer_occupancy_grid_node',
          name=f'cartographer_occupancy_grid_node_{i}',
          namespace=robot_name,
          arguments=[
            '-resolution', res,
            '-publish_period_sec', publish_period
          ],
          remappings=[
            ("/submap_list", f"robot_{i}/submap_list"),
            ("/landmark_poses_list", f"robot_{i}/landmark_poses_list"),
            ("/trajectory_node_list", f"robot_{i}/trajectory_node_list"),
            ("/scan_matched_points2", f"robot_{i}/scan_matched_points2"),
            ("/constraint_list", f"robot_{i}/constraint_list"),
          ],
        ), ) 

        # nodes.append(Node(
        #     package='cartographer_ros',
        #     executable='cartographer_node',
        #     name=f'cartographer_node',
        #     output='screen',
        #     namespace=f'robot_{i}',
        #     remappings=[
        #         # ('/tf', f'/robot_{i}/tf'),
        #         # ("/tf_static", f"/robot_{i}/tf_static"),
        #         # ("/scan", f"/robot_{i}/scan"),
        #         # (f"/robot_{i}/odom", f"/robot_{i}/filtered_odom"),
        #     ],
        #     arguments=['-configuration_directory', cartographer_config_dir,
        #                '-configuration_basename', configuration_basename]
        # ))

        # nodes.append(Node(
        #     package='cartographer_ros',
        #     executable='cartographer_occupancy_grid_node',
        #     name=f'cartographer_occupancy_grid_node',
        #     output='screen',
        #     namespace=f'robot_{i}',
        #     arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec],
        #     remappings=[
        #         ("/submap_list", f"robot_{i}/submap_list"),
        #         ("/landmark_poses_list", f"robot_{i}/landmark_poses_list"),
        #         ("/trajectory_node_list", f"robot_{i}/trajectory_node_list"),
        #         ("/scan_matched_points2", f"robot_{i}/scan_matched_points2"),
        #         ("/constraint_list", f"robot_{i}/constraint_list"),
        #     ]
        # ))   

  return LaunchDescription([

    SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
    launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='False',
                                            description='Flag to enable use_sim_time'),
    launch.actions.DeclareLaunchArgument(name='exploration', default_value='True',
                                            description='Flag to enable use_sim_time'),
    DeclareLaunchArgument(
      'resolution',
      default_value=res,
      description='configure the resolution'
    ),

    DeclareLaunchArgument(
      'publish_period_sec',
      default_value=publish_period,
      description='publish period in seconds'
    ),

    ################### cartographer_ros_node ###################
    # DeclareLaunchArgument(
    #   'configuration_directory',
    #   default_value=config_directory,
    #   description='path to the .lua files'
    # ),
    # DeclareLaunchArgument(
    #   'slam_configuration_basename',
    #   default_value=slam_config,
    #   description='name of .lua file to be used'
    # ),
    # DeclareLaunchArgument(
    #   'localization_configuration_basename',
    #   default_value=slam_config,
    #   description='name of .lua file to be used'
    # ),
    # Node(
    #   package='cartographer_ros',
    #   condition= IfCondition(exploration),
    #   executable='cartographer_node',
    #   name='as21_cartographer_node',
    #   arguments=[
    #     '-configuration_directory', config_directory,
    #     '-configuration_basename', slam_config
    #   ],
    #   parameters= [{'use_sim_time':use_sim_time}],
    #   output='screen'
    # ),
    # Node(
    #   package='cartographer_ros',
    #   condition=IfCondition(PythonExpression(['not ', exploration])),
    #   executable='cartographer_node',
    #   name='as21_cartographer_node',
    #   arguments=[
    #     '-configuration_directory', config_directory,
    #     '-configuration_basename', slam_config
    #   ],
    #   parameters= [{'use_sim_time':use_sim_time}],
    #   output='screen'
    # ),

    # Node(
    #   package='cartographer_ros',
    #   condition= IfCondition(exploration),
    #   executable='cartographer_occupancy_grid_node',
    #   name='cartographer_occupancy_grid_node',
    #   arguments=[
    #     '-resolution', res,
    #     '-publish_period_sec', publish_period
    #   ]
    # ),   
  *nodes,
  ]
)
