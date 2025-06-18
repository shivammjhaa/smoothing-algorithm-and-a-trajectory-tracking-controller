import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math
import time

# Load the trajectory
import sys
sys.path.append('/home/shiva')
from generate_trajectory import generate_trajectory, smooth_path

waypoints = [(0, 0), (2, 3), (5, 4), (7, 1)]
smoothed = smooth_path(waypoints)
trajectory = generate_trajectory(smoothed)

class TrajectoryFollower(Node):
    def __init__(self):
        super().__init__('trajectory_follower')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.current_pose = None
        self.trajectory = trajectory
        self.start_time = time.time()
        self.timer = self.create_timer(0.1, self.follow_trajectory)

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        self.current_pose = (pos.x, pos.y, yaw)

    def follow_trajectory(self):
        if self.current_pose is None:
            return

        current_time = time.time() - self.start_time
        closest = min(self.trajectory, key=lambda pt: abs(pt[2] - current_time))

        target_x, target_y, _ = closest
        curr_x, curr_y, curr_theta = self.current_pose

        dx = target_x - curr_x
        dy = target_y - curr_y

        target_theta = math.atan2(dy, dx)
        angle_error = target_theta - curr_theta
        distance = math.sqrt(dx ** 2 + dy ** 2)

        twist = Twist()
        twist.linear.x = min(0.2, distance)
        twist.angular.z = angle_error

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
