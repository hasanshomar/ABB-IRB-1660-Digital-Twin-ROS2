# 1. Setting Up a New ROS2 Workspace

For the sake of example, the workspace used in this documentation will be called `ros2_irb1660`

---
1. **Create a new workspace directory:**
   - Open a terminal and run:
     ```bash
     mkdir -p ~/ros2_irb1660/src
     cd ~/ros2_irb1660
     ```
   - This creates a new ROS 2 workspace directory named `ros2_irb1660` with a `src` subdirectory where your packages will be placed.

2. **Initialize the workspace:**
   - Run the following command to initialize your workspace:
     ```bash
     colcon build
     ```
   - This command builds the workspace and creates the necessary build and install directories.

3. **Source the workspace:**
   - You’ll need to source the workspace so that ROS 2 can find your packages:
     ```bash
     source install/setup.bash
     ```
     To make this automatic for every new terminal session, add the source command to your `.bashrc` file:
     ```bash
     echo "source ~/ros2_ws_test/install/setup.bash" >> ~/.bashrc
     source ~/.bashrc
     ```

# 2. Setting Up the ROS Package

1. **Create a new ROS 2 package:**
   - Navigate to the `src` directory:
     ```bash
     cd src
     ```
   - Create a new package with the necessary dependencies:
     ```bash
     ros2 pkg create --build-type ament_cmake irb1660_sim --dependencies rclcpp std_msgs sensor_msgs urdf xacro gazebo_ros control_msgs trajectory_msgs
     ```
   - This creates a new package named `irb16600_sim` with dependencies like `rclcpp`, `urdf`, `xacro`, and `gazebo_ros`, which are essential for your simulation.

# 3. Create the necessary directories

1. **URDF and Xacro files:**
   - Create a `urdf` directory inside your package:
     ```bash
     mkdir -p ~/ros2_irb1660/src/irb1660_sim/urdf
     ```
   - This is where we'll add the URDF files later.

2. **Launch files:**
   - Create a `launch` directory inside your package:
     ```bash
     mkdir -p ~/ros2_irb1660/src/irb1660_sim/launch
     ```
   - This is where we'll add the launch files later.

3. **Meshes and Models:**
   - Create a `meshes` directory inside your package:
     ```bash
     mkdir -p ~/ros2_irb1660/src/irb1660_sim/meshes
     ```
   - This is where we'll save the necessary 3D models for the robot.

# 4. Build and Source the Workspace

1. **Build the workspace:**
   - Go back to the root of the workspace and build it:
     ```bash
     cd ~/ros2_irb1660
     colcon build
     ```
     
2. **Source the workspace:**
   - Don’t forget to source the workspace after building:
     ```bash
     source install/setup.bash
     ```
     
