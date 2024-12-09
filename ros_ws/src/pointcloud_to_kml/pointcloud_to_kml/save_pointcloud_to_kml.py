import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import simplekml
from pyproj import Transformer
import os
import atexit

class MapToKML(Node):
    def __init__(self):
        super().__init__('map_to_kml')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/octomap_point_cloud_centers',  # Replace with your topic
            self.pointcloud_callback,
            10)
        self.points = []  # To accumulate all point cloud data
        self.get_logger().info("Node initialized. Listening to the point cloud topic.")
        atexit.register(self.save_on_exit)  # Register function to save KML on exit

    def pointcloud_callback(self, msg):
        self.get_logger().info("Received point cloud message.")
        # Accumulate all points in the map
        new_points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        self.points.extend(new_points)
        self.get_logger().info(f"Accumulated {len(new_points)} points. Total: {len(self.points)}.")

    def save_as_kml(self):
        self.get_logger().info("Saving the accumulated map as KML.")
        # Create a KML object
        kml = simplekml.Kml()

        # Define the coordinate transformation (e.g., from UTM to WGS84)
        transformer = Transformer.from_crs("epsg:32633", "epsg:4326", always_xy=True)  # Adjust CRS as needed

        for point in self.points:
            x, y, z = point
            lat, lon = transformer.transform(x, y)
            kml.newpoint(name="Point", coords=[(lon, lat, z)])

        # Save the KML file
        output_file = os.path.expanduser("/docker_SLAM/ros2_ws/maps/map_output.kml")
        kml.save(output_file)
        self.get_logger().info(f"KML file saved to {output_file}")

    def save_on_exit(self):
        """ Function to ensure KML is saved when the node exits """
        if self.points:
            self.get_logger().info("Exiting and saving accumulated points to KML...")
            self.save_as_kml()
        else:
            self.get_logger().info("No points accumulated, nothing to save.")

def main(args=None):
    rclpy.init(args=args)
    node = MapToKML()

    try:
        rclpy.spin(node)  # Keep the node running to receive point clouds
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt received. Saving KML file...")
        # Don't call shutdown here; it'll be handled by atexit
        rclpy.shutdown()

if __name__ == '__main__':
    main()
