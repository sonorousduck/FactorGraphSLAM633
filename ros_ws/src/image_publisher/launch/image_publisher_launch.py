from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare the launch argument for the image name
    image_name_arg = DeclareLaunchArgument(
        'image_name',
        default_value='my_image.png',  # Default image name
        description='Name of the image file in the images directory'
    )

    # Get the value of the launch argument
    image_name = LaunchConfiguration('image_name')

    return LaunchDescription([
        # Add the launch argument
        image_name_arg,

        # Launch the image publisher node
        Node(
            package='image_publisher',
            executable='image_node',
            name='image_publisher',
            output='screen',
            parameters=[
                {'image_path': image_name}  # Pass the image name as a parameter
            ]
        )
    ])
