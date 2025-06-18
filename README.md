Path Smoothing and Trajectory Control in 2D Space
This project implements a complete ROS 2-based trajectory tracking pipeline for a differential-drive robot. It features smooth path generation from waypoints, trajectory parameterization, and velocity-based tracking in simulation using the TurtleBot3 and Gazebo.

🚀 Overview
The robot follows a continuous trajectory derived from coarse 2D waypoints using:

B-spline interpolation for smooth path generation

Time-parameterized trajectory sampling

Proportional controller for path following

The system is built with modular Python scripts under ROS 2 Humble and simulates the TurtleBot3 Burger model in Gazebo.

⚙️ Prerequisites
ROS 2 Humble

Gazebo Classic

Python ≥ 3.10

ROS 2 packages:

turtlebot3_gazebo

gazebo_ros

tf_transformations

Python libraries:

numpy

scipy

matplotlib

🛠️ Setup Instructions
Install dependencies:

bash
sudo apt update
sudo apt install ros-humble-turtlebot3-gazebo \
                 python3-tf-transformations \
                 python3-numpy python3-scipy python3-matplotlib
Setup workspace:

bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
# clone or copy this repo here
cd ~/ros2_ws
colcon build --packages-select trajectory_follower
source install/setup.bash
Export TurtleBot3 model:

bash
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
▶️ Running the Simulation
Open 3 terminals:

🖥️ Terminal 1: Launch Gazebo

bash
source /opt/ros/humble/setup.bash
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
🤖 Terminal 2: Start the trajectory follower

bash
source ~/ros2_ws/install/setup.bash
ros2 run trajectory_follower trajectory_follower
📡 Terminal 3 (optional): Echo velocity topic

bash
ros2 topic echo /cmd_vel
🔍 Architecture Overview
📐 Path Smoothing (smooth_path.py)
Input: Waypoints

Method: B-spline interpolation (scipy)

Output: Smooth continuous path
![image](https://github.com/user-attachments/assets/3f60d5d5-e52b-449c-9382-29c45a6e130c)

🧭 Trajectory Generation (generate_trajectory.py)
Converts smoothed path to (x, y, t)

Constant velocity assumed (default: 0.2 m/s)

🎮 Trajectory Following (follower_node.py)
Reads current pose from /odom

Calculates linear and angular velocity to next point

Stops when trajectory is complete

🧪 Real-World Extension
Real sensors: Replace Gazebo odometry with encoder + IMU

Use robot_localization for data fusion

Adjust controller to hardware constraints

Add safety checks, soft failsafe, RViz debugging

🧠 AI Tools Used
ChatGPT: Logic design, documentation, debugging

Replit / VS Code: Iterative development

ROS CLI Tools: Testing, validation

🛡️ Obstacle Avoidance (Bonus)
Reactive strategy using /scan data

Stops robot when obstacle < 0.4m

Resumes once path is clear

Ideal for light dynamic environments

📸 Demo
You can include a demo video or screenshot here once uploaded. Example:


📁 Folder Structure
bash
Copy
Edit
trajectory_follower/
├── follower_node.py
├── generate_trajectory.py
├── smooth_path.py
├── setup.py
├── package.xml
└── README.md
