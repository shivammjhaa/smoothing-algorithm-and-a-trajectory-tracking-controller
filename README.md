A ROS 2-based trajectory-following module for differential-drive robots (e.g., TurtleBot3), capable of generating smooth trajectories from waypoints and executing them in Gazebo simulation.

📦 Requirements
ROS 2 Humble

turtlebot3_gazebo package

Python libraries: numpy, matplotlib, scipy, tf_transformations

Install dependencies:

bash
sudo apt update
sudo apt install ros-humble-turtlebot3-gazebo python3-tf-transformations
🛠️ Workspace Setup
bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Place your package here (trajectory_follower)

cd ~/ros2_ws
colcon build --packages-select trajectory_follower
source install/setup.bash

# Add to bashrc for convenience
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
▶️ Running the Simulation
Terminal 1: Launch Gazebo
bash
source /opt/ros/humble/setup.bash
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
Terminal 2: Run the trajectory follower
bash
cd ~/ros2_ws
source install/setup.bash
ros2 run trajectory_follower trajectory_follower
You should see your TurtleBot3 begin following the smooth trajectory in Gazebo.

🧠 Design & Architecture
Design Choices
Modular structure: code split into smooth_path.py, generate_trajectory.py, and follower_node.py.

ROS 2 native: uses rclpy, publishers, subscribers, and timers.

Simple constant-velocity planning to allow deterministic behavior.

Algorithms Used
🌀 B-spline interpolation to smooth discrete waypoints.

📐 Uniform velocity profile to time-stamp trajectory points.

🔁 The feedback controller calculates linear and angular velocities based on the robot’s position error using odometry.

🤖 Real Robot Extension
To deploy this to a physical TurtleBot:

Replace /odom topic with data from real encoders or sensor fusion (e.g., EKF using IMU/GPS).

Test first in controlled indoor space.

Deploy on Raspberry Pi or Jetson Nano with topic remapping.

Integrate safety checks: LIDAR-based collision detection or bumper stops.

🤖 AI Tools Used
🧠 ChatGPT (for modular design, code review, and documentation support)

🧪 VS Code with Pylance (code linting and Python integration)

🧪 Gazebo: Simulation platform for visual debugging of the robot’s trajectory

🧰 rqt_graph and ros2 topic echo for inspecting and verifying topic communication

🚧 Extra Credit: Obstacle Avoidance Plan
To extend this with dynamic obstacle avoidance:

Subscribe to /scan topic from LIDAR sensor.

Detect proximity violations using simple thresholding or segmentation.

Replan a new, smoothed trajectory on the fly by updating waypoints around obstacles.

Integrate with Nav2 or use potential fields/vector field histograms as reactive layers.

Combine with costmaps and SLAM for autonomous navigation.
