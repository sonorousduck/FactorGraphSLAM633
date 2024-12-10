from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    kiss_icp_package_path = FindPackageShare("kiss_icp").find("kiss_icp")
    odometry_launch_file = os.path.join(kiss_icp_package_path, "launch", "odometry.launch.py")

    
    return LaunchDescription([

        
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            parameters=[
                {'resolution': 0.05},
                {'frame_id': 'odom'},
                {'sensor_model.max_range': 20.0}
            ],
            remappings=[
                ('cloud_in', '/kiss/frame')
            ]
        ),

        # Node(
        #     package='gtsam_graph',
        #     executable='factorgraph',
        #     name='factorgraph_node',
        # ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(odometry_launch_file),
            launch_arguments={"topic": "/ouster/points"}.items()
        )
        
    ])
