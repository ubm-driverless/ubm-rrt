import os

import launch
import yaml
import launch_ros.actions
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory


IS_SIMULATION = os.environ.get('CAR_NAME') == 'sim'
print(f'IS_SIMULATION: {IS_SIMULATION}')

if IS_SIMULATION:
    POSE_TOPIC = '/odom'
else:
    POSE_TOPIC = '/pf/pose/odom'

config_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'config')
settings = {}

with open(os.path.join(config_dir, 'launch_config.yaml')) as f:
    config = yaml.safe_load(f)
    for setting in config:
        settings[setting] = config[setting]


def generate_launch_description():
    ld = LaunchDescription()

    rrt_node = Node(
        package='rrt',
        executable='rrt_node',
        name='rrt',
        output='screen',
        # prefix=['gdbserver localhost:3000'],
        arguments=['--ros-args', '--log-level', 'info'],  # debug, info, warn, error, fatal, unset (default: info)
        parameters=[{
            'use_obstacle_callback': False,
            'map_filepath': settings['map_filepath'],
            'map_yamlpath': settings['map_yamlpath'],
            'disable_print_timeout': 2000,
            'visualize_roi': True,
            'visualize_roi_on_map': True,
            'raceline_points_before': 3,
            'raceline_points_after': 50,
            'rrt_goal_offset': 1,  # [pixel] where to locate the goal point in the raceline wrt the ROI's last point. The higher the value, the more the goal point will be closer to the car
            'track_dilation_shape': 1,   # 0 -> Rectangular, 1 -> Cross, 2 -> Ellipse
            'track_dilation_size': 1,
            'obstacle_dilation_size': 1.0,
            'lidar_obstacle_dilation_size': 0.2,
            'max_lidar_range': 2.0,
            'scan_step': 25,

            'check_min_distance_to_obstacle': True,
            'min_distance_to_obstacle': 0.2,
            'slow_down_topic': '/rrt/slow_down',
            'send_slow_down': False,

            'ax_max_machines_path': '/home/ubm/repo/raceline/raceline/TUM/inputs/veh_dyn_info/ax_max_machines.csv',
            'ggv_path': '/home/ubm/repo/raceline/raceline/TUM/inputs/veh_dyn_info/ggv.csv',

            'topic_for_real_position': POSE_TOPIC,
            'scan_topic': '/scan'
        }]
    )

    ld.add_action(rrt_node)

    return ld
