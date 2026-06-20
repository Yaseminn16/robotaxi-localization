from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    config = os.path.join(
        get_package_share_directory('localization_pkg'),
        'config',
        'ekf.yaml'
    )

    return LaunchDescription([
        
        # EKF Node - GPS + IMU birleştirir
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[config]
        ),

        # GPS'i odometry formatına çevirir
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[{
                'magnetic_declination_radians': 0.0,
                'yaw_offset': 0.0,
                'zero_altitude': True,
                'use_sim_time': True,
                'wait_for_datum': False,
                'frequency': 30.0,
                'delay': 3.0,
            }],
            remappings=[
                ('imu/data', '/gazebo_ros_imu/out'),
                ('gps/fix', '/gazebo_ros_gps/out')
            ]
        ),
    ])
