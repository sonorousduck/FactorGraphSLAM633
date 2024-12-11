import rclpy  # noqa: F401
from rclpy.node import Node
from nav_msgs.msg import Odometry
# from matplotlib.animation import FuncAnimation
# import matplotlib.pyplot as plt
import gtsam
# import math
import numpy as np


# TODO: Make sure to change the topic names and try in incorporate the full quaternion
'''
self.lidar_topics = [
            '/kiss_top/odometry',
            '/kiss_left/odometry',
            '/kiss_right/odometry',
            '/kiss_front/odometry',
            '/kiss_back/odometry'
        ]
'''

class FactorGraphSLAM(Node):

    def __init__(self):
        super().__init__('factorgraph_node')

        # Subscribe to five LiDAR odometry topics
        self.lidar_topics = [
            '/kiss_top/odometry',
            # '/kiss_left/odometry',
            # '/kiss_right/odometry',
            # '/kiss_front/odometry',
            # '/kiss_back/odometry'
        ]

        self.my_subscriptions = []
        for idx, topic in enumerate(self.lidar_topics):
            self.my_subscriptions.append(
                self.create_subscription(Odometry, topic, lambda msg, idx=idx: self.main_callback(msg,idx), 10)
                
            )

        self.publisher = self.create_publisher(Odometry, '/optimized_odom', 10)
        self.graph = gtsam.NonlinearFactorGraph()
        self.initial_estimate = gtsam.Values()
        self.previous_poses = {i: None for i in range(5)}  # Track previous poses for each LiDAR
        self.key_counters = {i: 0 for i in range(5)}  # Separate key counters for each LiDAR

        # Add prior factor for the first LiDAR
        prior_pose = gtsam.Pose3(gtsam.Rot3(), np.array([0.0, 0.0, 0.0]))
        prior_noise = gtsam.noiseModel.Diagonal.Sigmas([0.2, 0.2, 0.2, 0.1, 0.1, 0.1])
        self.graph.add(gtsam.PriorFactorPose3(0, prior_pose, prior_noise))
        self.initial_estimate.insert(0, prior_pose)
        self.previous_poses[0] = prior_pose

    def quaternion_to_rot3(self, q):
        rotation_matrix = gtsam.Rot3.Quaternion(q.w, q.x, q.y, q.z)
        return rotation_matrix

    def main_callback(self, incoming, idx):
        # Determine which LiDAR the message is from
        topic_name = self.lidar_topics[idx]  # Topic name the message came from
        lidar_index = self.lidar_topics.index(topic_name)

        # Extract pose
        x = incoming.pose.pose.position.x
        y = incoming.pose.pose.position.y
        z = incoming.pose.pose.position.z
        position = np.array([x, y, z])
        orientation = incoming.pose.pose.orientation
        rotation = self.quaternion_to_rot3(orientation)

        new_pose = gtsam.Pose3(rotation, position)
        self.key_counters[lidar_index] += 1
        current_key = self.key_counters[lidar_index]

        # Add odometry factor to the graph
        if self.previous_poses[lidar_index] is not None:
            odometry = self.previous_poses[lidar_index].between(new_pose)
            odom_noise = gtsam.noiseModel.Diagonal.Sigmas([0.2, 0.2, 0.2, 0.1, 0.1, 0.1])
            self.graph.add(gtsam.BetweenFactorPose3(
                current_key - 1, current_key, odometry, odom_noise))

        self.previous_poses[lidar_index] = new_pose
        if not self.initial_estimate.exists(current_key):
            self.initial_estimate.insert(current_key, new_pose)
        else:
            self.get_logger().warn(f"Key {current_key} already exists in the initial estimate. Skipping insertion.")


        # Optimize the graph
        optimizer = gtsam.LevenbergMarquardtOptimizer(self.graph, self.initial_estimate)
        result = optimizer.optimize()

        optimized_pose = result.atPose3(current_key)
        self.get_logger().debug(
            f"LiDAR {lidar_index} Optimized Pose: x={optimized_pose.x()}, "
            f"y={optimized_pose.y()}, z={optimized_pose.z()}"
        )
        self.publish_optimized_odometry(optimized_pose)

    def publish_optimized_odometry(self, optimized_pose):
        # Publish the optimized points
        position = optimized_pose.translation()
        rotation = optimized_pose.rotation().toQuaternion()

        out = Odometry()
        out.header.frame_id = 'odom_lidar'
        out.pose.pose.position.x = position[0]
        out.pose.pose.position.y = position[1]
        out.pose.pose.position.z = position[2]
        out.pose.pose.orientation.x = rotation.x()
        out.pose.pose.orientation.y = rotation.y()
        out.pose.pose.orientation.z = rotation.z()
        out.pose.pose.orientation.w = rotation.w()
        self.publisher.publish(out)



def main(args=None):
    rclpy.init(args=args)
    node = FactorGraphSLAM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # node.plot_trajectory()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
