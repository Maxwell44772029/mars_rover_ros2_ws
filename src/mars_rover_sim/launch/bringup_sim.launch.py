from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory  

import os

def generate_launch_description():

    # nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    # mars_rover_sim_dir = get_package_share_directory('mars_rover_sim')

    # nav2_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
    #     ),
    #     launch_arguments={
    #         'use_sim_time': 'true',
    #         'params_file': str(os.path.join(mars_rover_sim_dir, 'config', 'nav2_params.yaml')),
    #         'map_subscribe_transient_local': 'true',
    #         'autostart': 'true',
    #         'use_map_topic': 'False',
    #         'use_lifecycle_mgr': 'False',
    #         'use_composition': 'False',
    #         'slam': 'False',
    #         'map': '__none__', # Pass empty map to satisfy the argument
    #     }.items()
    # )
    return LaunchDescription([
        
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_tf_lidar',
        #     arguments=[
        #         '0', '0', '0', '0', '0', '0', 'base_link', 'lidar_link'
        #     ],
        #     output='screen'
        # ),
        
        # Octomap Server
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            parameters=[{
                'frame_id': 'map',
                'resolution': 1.0,
                'sensor_model.max_range': 60.0,
                'sensor_model.hit': 0.7,
                'sensor_model.miss': 0.4,
                'sensor_model.min': 0.12,
                'sensor_model.max': 0.97,
                'pointcloud_min_z': -1.0,
                'pointcloud_max_z': 50.0,
                'occupancy_thres': 0.5,
                'occupancy_min_thres': 0.12,
                'occupancy_max_thres': 0.97,
                'transform_tolerance': 0.3,
                'latch': False,
                'use_sim_time': True,
                'wait_for_transform': True,
            }],
            remappings=[
                ('/cloud_in', '/pointcloud'),
            ]
        ),
        Node(
            package='mars_rover_sim',
            executable='simple_astar_navigator',
            name='simple_astar',
            output='screen'
        ),

        # Delayed RViz2 launch (fixes timing issues)
        TimerAction(
            period=2.0,  # wait 2 seconds before launching RViz
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    output='screen',
                    arguments=['-d', 'config/octomap.rviz'],
                    parameters=[{'use_sim_time': True}]
                )
            ]
        ),

        Node(
            package='mars_rover_sim',
            executable='project_octomap_to_costmap',  # No .py here
            name='costmap_projection',
            output='screen'
        ),

        # nav2_launch,


        # Pose to TF broadcaster
        # ExecuteProcess(
        #     cmd=['python3', 'scripts/pose_to_tf.py'],
        #     cwd='/home/maxwellg/ros2_ws/src/mars_rover_sim',
        #     name='pose_to_tf_process',
        #     output='screen'
        # )
    ])

