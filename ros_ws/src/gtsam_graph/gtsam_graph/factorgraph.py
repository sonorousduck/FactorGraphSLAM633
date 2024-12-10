import rclpy  # noqa: F401
from rclpy.node import Node
from nav_msgs.msg import Odometry
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
import gtsam
import math
import numpy as np


# TODO: Make sure to change the topic names and try in incorporate the full quaternion


class FactorGraphSLAM(Node):

    def __init__(self):
        super().__init__('factorgraph_node')
        self.subscription = self.create_subscription(Odometry, '/kiss/odometry', self.main_callback, 10)
        self.subscription  # Prevent warning for unused variable
        self.publisher = self.create_publisher(Odometry, '/optimized_odom', 10)
        self.graph = gtsam.NonlinearFactorGraph()
        self.initial_estimate = gtsam.Values()
        self.previous_pose = None
        self.key_counter = 0

        prior_pose = gtsam.Pose3(gtsam.Rot3(),np.array([0.0,0.0,0.0]))
        prior_noise = gtsam.noiseModel.Diagonal.Sigmas([0.2,0.2,0.2,0.1,0.1,0.1])
        self.graph.add(gtsam.PriorFactorPose3(self.key_counter,prior_pose,prior_noise))
        self.initial_estimate.insert(self.key_counter,prior_pose)
        self.previous_pose = prior_pose
        # self.animation = FuncAnimation(self.fig, self.update_plot, interval=500)

    
    def quaternion_to_rot3(self,q):
        rotation_matrix = gtsam.Rot3.Quaternion(q.w,q.x,q.y,q.z)
        return rotation_matrix

    def main_callback(self, incoming):
        x = incoming.pose.pose.position.x
        y = incoming.pose.pose.position.y
        z = incoming.pose.pose.position.z
        position = np.array([x,y,z])
        orientation = incoming.pose.pose.orientation
        rotation = self.quaternion_to_rot3(orientation)

        new_pose = gtsam.Pose3(rotation,position)
        self.key_counter += 1

        if self.previous_pose is not None:
            odometry = self.previous_pose.between(new_pose)
            odom_noise = gtsam.noiseModel.Diagonal.Sigmas([0.2,0.2,0.2,0.1,0.1,0.1])
            self.graph.add(gtsam.BetweenFactorPose3(self.key_counter - 1, self.key_counter,odometry,odom_noise))

        self.previous_pose = new_pose
        self.initial_estimate.insert(self.key_counter, new_pose)

        # Optimize the graph
        optimizer = gtsam.LevenbergMarquardtOptimizer(self.graph,self.initial_estimate)
        result = optimizer.optimize()

        optimized_pose = result.atPose3(self.key_counter)
        self.get_logger().info(f"Optimized Pose: x={optimized_pose.x()}, y={optimized_pose.y()}, z={optimized_pose.z()}")
        self.publish_optimized_odometry(optimized_pose)

    def publish_optimized_odometry(self, optimized_pose):
        # Publish the optimized points
        
        position = optimized_pose.translation()
        rotation = optimized_pose.rotation().toQuaternion()

        out = Odometry()
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
