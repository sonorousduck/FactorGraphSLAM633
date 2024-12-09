from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
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
                ('cloud_in', '/ouster/points')
            ]
        ),
        
        Node(
                package='rf2o_laser_odometry',
                executable='rf2o_laser_odometry_node',
                name='rf2o_laser_odometry',
                output='screen',
                parameters=[{
                    'laser_scan_topic' : '/ouster/scan',
                    'odom_topic' : '/odom',
                    'publish_tf' : True,
                    'base_frame_id' : 'base_link',
                    'odom_frame_id' : 'odom',
                    'init_pose_from_topic' : '',
                    'freq' : 20.0}],
            ),
        
        
    ])

# <!-- <launch>
# 	<node pkg="octomap_server" exec="octomap_server_node" name="octomap_server">
# 		<param name="resolution" value="0.05" />

# 		<!-- fixed map frame (set to 'map' if SLAM or localization running!) -->
# 		<param name="frame_id" value="odom_combined" />

# 		<!-- maximum range to integrate (speedup!) -->
# 		<param name="sensor_model.max_range" value="5.0" />

# 		<!-- data source to integrate (PointCloud2) -->
# 		<remap from="cloud_in" to="/ouster/points" />
# 	</node>
# </launch> -->