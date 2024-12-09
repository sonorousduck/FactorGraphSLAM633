import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from pathlib import Path

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')

        # Declare and get the image_path parameter
        self.declare_parameter('image_path', '')
        image_path_param = self.get_parameter('image_path').get_parameter_value().string_value

        # Resolve the image path relative to the 'images' directory in the package
        package_path = Path(__file__).parent.parent  # Path to 'image_publisher' package
        images_directory = package_path / 'images'
        self.image_path = images_directory / image_path_param

        if not self.image_path.exists():
            self.get_logger().error(f"Image file does not exist: {self.image_path}")
            rclpy.shutdown()

        self.publisher_ = self.create_publisher(Image, 'image_topic', 10)
        self.timer = self.create_timer(0.5, self.publish_image)  # Publish every 0.5 seconds
        self.bridge = CvBridge()
        self.get_logger().info(f"Image Publisher Node started. Publishing: {self.image_path}")

    def publish_image(self):
        # Read the image using OpenCV
        image = cv2.imread(str(self.image_path))

        if image is None:
            self.get_logger().error(f"Could not read the image: {self.image_path}")
            return

        # Convert the OpenCV image to a ROS Image message
        image_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")

        # Publish the message
        self.publisher_.publish(image_msg)
        self.get_logger().info('Published image.')

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
