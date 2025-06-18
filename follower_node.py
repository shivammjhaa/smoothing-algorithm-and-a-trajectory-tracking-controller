import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math

from trajectory_follower.smooth_path import smooth_path
from trajectory_follower.generate_trajectory import generate_trajectory


class TrajectoryFollower(Node):
    """
    A ROS 2 node that follows a time-parameterized 2D trajectory using odometry feedback.
    Publishes velocity commands to `/cmd_vel` and subscribes to `/odom`.
    """

    def __init__(self):
        super().__init__('trajectory_follower')

        # Publishers and Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Generate a smooth trajectory from waypoints
        waypoints = [(0, 0), (1, 2), (3, 4), (6, 1)]
        smoothed_path = smooth_path(waypoints)
        self.trajectory = generate_trajectory(smoothed_path)

        # Robot state
        self.current_pose = None  # (x, y, yaw)
        self.current_index = 0

        # Timer to periodically control the robot
        self.timer = self.create_timer(0.1, self.follow_trajectory)

    def odom_callback(self, msg):
        """
        Callback to update the robot's current pose from odometry.
        """
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        self.current_pose = (pos.x, pos.y, yaw)

    def follow_trajectory(self):
        """
        Timer callback that drives the robot to follow the trajectory.
        """
        if self.current_pose is None:
            self.get_logger().info("Waiting for odometry...")
            return

        if self.current_index >= len(self.trajectory):
            self.get_logger().info("Trajectory complete. Stopping robot.")
            self.cmd_pub.publish(Twist())
            return

        # Get current and target positions
        target_x, target_y, _ = self.trajectory[self.current_index]
        current_x, current_y, current_yaw = self.current_pose

        # Compute control commands
        dx = target_x - current_x
        dy = target_y - current_y
        distance = math.hypot(dx, dy)

        # If close enough, move to the next waypoint
        if distance < 0.1:
            self.current_index += 1
            return

        # Compute steering
        target_angle = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(target_angle - current_yaw)

        twist = Twist()
        twist.linear.x = min(0.2, distance)
        twist.angular.z = angle_diff

        # Publish the velocity command
        self.cmd_pub.publish(twist)
        self.get_logger().info(f"[cmd_vel] linear: {twist.linear.x:.2f}, angular: {twist.angular.z:.2f}")

    @staticmethod
    def normalize_angle(angle):
        """
        Normalize angle to be between [-π, π].
        """
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main(args=None):
    """
    Main function to initialize the ROS 2 node.
    """
    rclpy.init(args=args)
    node = TrajectoryFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
