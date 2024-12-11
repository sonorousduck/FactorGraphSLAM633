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

    lidar_transform_path = FindPackageShare("lidar_transformer").find("lidar_transformer")
    lidar_transform_launch_file = os.path.join(lidar_transform_path, "launch", "lidar_transformer.launch.py")

    
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

        Node(
            package='gtsam_graph',
            executable='factorgraph',
            name='factorgraph_node',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_transform_launch_file),
            launch_arguments={"viz": "false"}.items()
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(odometry_launch_file),
            launch_arguments={"topic": "/transformed_lidar/ouster_top/points",
                              "my_namespace": "/kiss_top",
                              "visualize": "true"}.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(odometry_launch_file),
            launch_arguments={"topic": "/transformed_lidar/ouster_left/points",
                              "my_namespace": "/kiss_left",
                              "visualize": "false"}.items()
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(odometry_launch_file),
            launch_arguments={"topic": "/transformed_lidar/ouster_right/points",
                              "my_namespace": "/kiss_right",
                              "visualize": "false"}.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(odometry_launch_file),
            launch_arguments={"topic": "/transformed_lidar/ouster_front/points",
                              "my_namespace": "/kiss_front",
                              "visualize": "false"}.items()
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(odometry_launch_file),
            launch_arguments={"topic": "/transformed_lidar/ouster_back/points",
                              "my_namespace": "/kiss_back",
                              "visualize": "false"}.items()
        ),
        
    ])
