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
        self.animation = FuncAnimation(self.fig, self.update_plot, interval=500)
        plt.ion()
        plt.show()


    def main_callback(self, incoming):
        x = incoming.pose.pose.position.x
        y = incoming.pose.pose.position.y

        self.key_counter += 1

        new_pose = gtsam.Pose2(x,y,0.0)
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


    def update_plot(self, frame):
        self.ax.clear()

        if self.initial_trajectory:
            x_init, y_init = zip(*self.initial_trajectory)
            self.ax.plot(x_init, y_init, 'r--', label='Initial Estimate')

        if self.optimized_trajectory:
            x_opt, y_opt = zip(*self.optimized_trajectory)
            self.ax.plot(x_opt,y_opt, 'b-', label='Optimized Trajectory')

        self.ax.set_title("GTSAM trajectory estimation")
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.legend()
        self.ax.grid()
        plt.pause(0.1)
        


def main(args=None):
    rclpy.init(args=args)
    node = FactorGraphSLAM()
    rclpy.spin(node)
    plt.show()
    

    node.destroy_node()
    rclpy.shutdown()
    plt.close()

if __name__ == '__main__':
    main()

