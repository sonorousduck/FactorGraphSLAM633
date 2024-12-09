import rclpy  # noqa: F401
from rclpy.node import Node
from nav_msgs.msg import Odometry
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
import gtsam


class FactorGraphSLAM(Node):


    def __init__(self):
        super().__init__('factorgraph_node')
        self.subscription = self.create_subscription(Odometry, '/odom', self.main_callback, 10)
        self.subscription  # Prevent warning for unused variable
        self.publisher = self.create_publisher(Odometry, '/optimized_odom', 10)
        self.graph = gtsam.NonlinearFactorGraph()
        self.initial_estimate = gtsam.Values()
        self.previous_pose = None
        self.key_counter = 0

        self.initial_trajectory = []
        self.optimized_trajectory = []

        prior_pose = gtsam.Pose2(0.0,0.0,0.0)
        prior_noise = gtsam.noiseModel.Diagonal.Sigmas([0.1,0.1,0.05])
        self.graph.add(gtsam.PriorFactorPose2(self.key_counter,prior_pose,prior_noise))
        self.initial_estimate.insert(self.key_counter,prior_pose)
        self.initial_trajectory.append((0.0,0.0))

        self.fig, self.ax = plt.subplots()
        # self.animation = FuncAnimation(self.fig, self.update_plot, interval=500)
        plt.ion()
        plt.show()

    def quaternion_to_yaw(orientation):
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w
        yaw = math.atan2(2.0 * (w*z + x*y), 1.0 - 2.0*(y * y + z * z))
        return yaw

    def yaw_to_quaternion(yaw):
        qz = math.sin(yaw/2.0)
        qw = math.cos(yaw/2.0)
        return (0.0,0.0,qz,qw)

    def main_callback(self, incoming):
        x = incoming.pose.pose.position.x
        y = incoming.pose.pose.position.y

        orientation = incoming.pose.pose.orientation
        theta = quaternion_to_yaw(orientation)

        self.key_counter += 1

        new_pose = gtsam.Pose2(x,y,theta)
        if self.previous_pose is not None:
            odometry = new_pose.between(self.previous_pose)
            odom_noise = gtsam.noiseModel.Diagonal.Sigmas([0.2,0.2,0.1])
            self.graph.add(gtsam.BetweenFactorPose2(self.key_counter - 1, self.key_counter,odometry,odom_noise))

        self.previous_pose = new_pose
        self.initial_estimate.insert(self.key_counter, new_pose)
        self.initial_trajectory.append((x,y))

        # Optimize the graph
        optimizer = gtsam.LevenbergMarquardtOptimizer(self.graph,self.initial_estimate)
        result = optimizer.optimize()

        optimized_pose = result.atPose2(self.key_counter)
        self.optimized_trajectory = [(result.atPose2(k).x(), result.atPose2(k).y()) for k in range(self.key_counter + 1)]
        self.get_logger().info(f"Optimized Pose: x={optimized_pose.x()}, y={optimized_pose.y()}")

        # Publish the optimized points
        out = Odometry()
        out.pose.pose.position.x = optimized_pose.x()
        out.pose.pose.position.y = optimized_pose.y()
        qx,qy,qz,qw = self.quaternion_to_yaw(optimized_pose.theta())
        out.pose.pose.orientation.x = qx
        out.pose.pose.orientation.y = qy
        out.pose.pose.orientation.z = qz
        out.pose.pose.orientation.w = qw
        self.publisher.publish(out)

        # self.plot_trajectory()

    # def plot_trajectory(self):
    #     fig, ax = plt.subplots()
    #     if self.initial_trajectory:
    #         x_init, y_init = zip(*self.initial_trajectory)
    #         ax.plot(x_init, y_init, 'r--', label='Initial Estimate')
    #
    #
    #     if self.optimized_trajectory:                                    
    #         x_opt, y_opt = zip(*self.optimized_trajectory)               
    #         ax.plot(x_opt,y_opt, 'b-', label='Optimized Trajectory')
    #
    #     ax.set_title("GTSAM Trajectory Estimation")
    #     ax.set_xlabel("X")
    #     ax.set_ylabel("Y")
    #     ax.legend()
    #     ax.grid()
    #     plt.show()


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

