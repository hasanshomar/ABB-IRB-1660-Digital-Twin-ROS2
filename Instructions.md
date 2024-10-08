# Setting Up a New ROS2 Workspace

For the sake of example, the workspace used in this documentation will be called `ros2_irb1660`

1. **Create a new workspace directory:**
   - Open a terminal and run:
     ```bash
     mkdir -p ~/ros2_ws/src
     cd ~/ros2_ws
     ```
   - This creates a new ROS 2 workspace directory named `ros2_ws` with a `src` subdirectory where your packages will be placed.

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
     echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
     source ~/.bashrc
     ```

# Setting Up the ROS Package

1. **Create a new ROS 2 package:**
   - Navigate to the `src` directory:
     ```bash
     cd src
     ```
   - Create a new package with the necessary dependencies:
     ```bash
     ros2 pkg create --build-type ament_cmake irb1660 --dependencies rclcpp std_msgs sensor_msgs urdf xacro gazebo_ros control_msgs trajectory_msgs
     ```
   - This creates a new package named `irb16600` with dependencies like `rclcpp`, `urdf`, `xacro`, and `gazebo_ros`, which are essential for your simulation.

# Create the necessary directories

1. **URDF and Xacro files:**
   - Create a `urdf` directory inside your package:
     ```bash
     mkdir -p ~/ros2_ws/src/irb1660/urdf
     ```
   - This is where we'll add the URDF files later.

2. **Launch files:**
   - Create a `launch` directory inside your package:
     ```bash
     mkdir -p ~/ros2_ws/src/irb1660/launch
     ```
   - This is where we'll add the launch files later.

3. **Meshes and Models:**
   - Create a `meshes` directory inside your package:
     ```bash
     mkdir -p ~/ros2_ws/src/irb1660/meshes
     ```
   - This is where we'll save the necessary 3D models for the robot.

# Build and Source the Workspace

1. **Build the workspace:**
   - Go back to the root of the workspace and build it:
     ```bash
     cd ~/ros2_ws
     colcon build
     ```
     
2. **Source the workspace:**
   - Don’t forget to source the workspace after building:
     ```bash
     source install/setup.bash
     ```

Your current workspace structure should look something like this now:

#### Workspace Structure
- **`ros2_ws/`** (Root workspace directory)
  - **`build/`**: Directory where the build artifacts are stored.
  - **`install/`**: Directory containing the installed packages after building.
  - **`log/`**: Directory where logs are stored.
  - **`src/`**: Source directory for your ROS 2 packages.
    - **`irb1660/`**: Your ROS 2 package directory.
      - **`include/irb1660/`**: This is where header files (if any) would be placed.
      - **`launch/`**: Directory for launch files that start the simulation or other ROS nodes.
      - **`meshes/`**: Directory for 3D model files (likely `.stl` or `.dae`).
      - **`urdf/`**: Directory for URDF and Xacro files that describe the robot model.
      - **`src/`**: Directory for source files (e.g., `.cpp` files for nodes).
      - **`package.xml`**: ROS package manifest, specifying dependencies and package metadata.
      - **`CMakeLists.txt`**: Build configuration file for compiling the package.

---

# Understanding URDF Files

#### **What is URDF?**
URDF stands for **Unified Robot Description Format**. It is an XML format used in ROS (Robot Operating System) to describe the physical configuration of a robot. A URDF file contains all the necessary information to model a robot, including its links (bodies), joints, sensors, actuators, and other physical properties like mass and inertia.

#### **Key Concepts in URDF:**
1. **Links:**
   - A **link** represents a rigid body in the robot. It could be a simple shape like a box or cylinder or a complex 3D model imported from a mesh file.
   - **Example:** A robot's arm, base, or gripper could each be modeled as a separate link.

2. **Joints:**
   - A **joint** connects two links and defines how they move relative to each other. Joints can be of different types, such as revolute (rotating around an axis), prismatic (sliding along an axis), fixed (no movement), and more.
   - **Example:** The elbow joint in a robotic arm, allowing the lower arm to rotate relative to the upper arm.

3. **Inertia and Mass:**
   - URDF allows specifying the **mass** of each link and its **inertia**, which defines how it resists rotational motion. This is crucial for accurate physics simulation in environments like Gazebo.

4. **Visual and Collision Properties:**
   - **Visual:** Defines how the robot looks in a simulation or visualization tool like RViz. It can be a simple geometric shape or a complex mesh.
   - **Collision:** Defines the shapes used for collision detection. This can be simpler than the visual model to reduce computational load.

5. **Sensors and Actuators:**
   - URDF files can include descriptions of sensors (e.g., cameras, LiDAR) and actuators (e.g., motors) to simulate them in the robot model.

6. **Xacro:**
   - Often, URDF files are written in **Xacro** (XML Macros), a more flexible format that allows the use of variables, macros, and includes. Xacro files are processed to generate standard URDF files. This approach reduces redundancy and makes complex robot models easier to manage.

#### **What Does a URDF File Do?**
- **Robot Modeling:**
  - The primary purpose of a URDF file is to model the robot in simulation environments like Gazebo and visualization tools like RViz. This model includes the robot's geometry, kinematics, and dynamics.

- **Simulation:**
  - In simulation, the URDF provides all the necessary data to simulate the robot's behavior, including how it moves, interacts with the environment, and responds to forces.

- **Control:**
  - The URDF is used by controllers in ROS to apply commands to the robot’s joints. This includes everything from simple joint position controllers to complex trajectory controllers.

- **Visualization:**
  - Tools like RViz use the URDF to render a 3D model of the robot, which is crucial for understanding the robot's state during operation.

### **How URDF Fits into ROS:**
- **ROS Nodes:** The URDF is used by various ROS nodes to understand the robot's structure and simulate its behavior. For example, a node can use URDF to publish the robot's state (e.g., joint positions) to other nodes.
- **RViz Visualization:** RViz loads the URDF to visualize the robot's current state, including joint positions and sensor data.
- **Gazebo Simulation:** Gazebo uses the URDF to create a physics-based simulation of the robot, allowing you to test the robot's interactions with the environment.

**So in summary:**

The URDF file is essentially a detailed blueprint of the robot, describing its physical shape, properties, and how its parts are connected. While the URDF doesn't "do" anything on its own as it's just a descriptive file, it is crucial for enabling other parts of the ROS ecosystem to interact with, simulate, and visualize the robot correctly.

---
# Creating the URDF file

Let's create a new URDF file in your `urdf` folder.

1. **Create the File:**
   - Navigate to your `urdf` directory in the terminal:
     ```bash
     cd ~/ros2_ws/src/irb1660/urdf
     ```
   - Create a new file named `irb1660_robot.urdf`:
     ```bash
     touch irb1660_robot.urdf
     ```

2. **Write the URDF code (See next section for detailed explanation of components of the URDF):**
   - Open the `irb1660_robot.urdf` file with your preferred text editor (e.g., `nano`, `vim`, or a graphical editor like `gedit` or VS Code):
```bash
gedit irb1660_robot.urdf
```
- Copy the following code:

```xml
<?xml version="1.0"?>
<robot name="irb1660_robot">

  <!-- Base Link -->
  <link name="base_link">
    <visual name="visual">
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <mesh filename="package://irb1660/meshes/base_link.dae"/>
      </geometry>
    </visual>
  </link>

  <!-- Joint 1 -->
  <joint name="joint_1" type="revolute">
    <origin xyz="0 0 0.1255" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <parent link="base_link"/>
    <child link="link_1"/>
    <limit effort="10.0" lower="-2.967" upper="2.967" velocity="1.7453"/>
  </joint>

  <!-- Link 1 -->
  <link name="link_1">
    <visual name="visual">
      <origin xyz="0 0 -0.1255" rpy="0 0 0" />
      <geometry>
        <mesh filename="package://irb1660/meshes/link_1.dae"/>
      </geometry>
    </visual>
  </link>

  <!-- Joint 2 -->
  <joint name="joint_2" type="revolute">
    <origin xyz="0.15 0.137 0.4865" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <parent link="link_1"/>
    <child link="link_2"/>
    <limit effort="10.0" lower="-1.134" upper="1.4855" velocity="1.5707"/>
  </joint>

  <!-- Link 2 -->
  <link name="link_2">
    <visual name="visual">
      <origin xyz="-0.15 -0.137 -0.612" rpy="0 0 0" />
      <geometry>
        <mesh filename="package://irb1660/meshes/link_2.dae"/>
      </geometry>
    </visual>
  </link>

  <!-- Joint 3 -->
  <joint name="joint_3" type="revolute">
    <origin xyz="0.15 0.107 1.1865" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <parent link="link_2"/>
    <child link="link_3"/>
    <limit effort="10.0" lower="-3.142" upper="1.222" velocity="1.5707"/>
  </joint>

  <!-- Link 3 -->
  <link name="link_3">
    <visual name="visual">
      <origin xyz="-0.3 -0.244 -1.7985" rpy="0 0 0" />
      <geometry>
        <mesh filename="package://irb1660/meshes/link_3.dae"/>
      </geometry>
    </visual>
  </link>

  <!-- Joint 4 -->
  <joint name="joint_4" type="revolute">
    <origin xyz="0.464 0 1.1865" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <parent link="link_3"/>
    <child link="link_4"/>
    <limit effort="10.0" lower="-5.236" upper="5.236" velocity="2.9671"/>
  </joint>

  <!-- Link 4 -->
  <link name="link_4">
    <visual name="visual">
      <origin xyz="-0.764 -0.244 -2.985" rpy="0 0 0" />
      <geometry>
        <mesh filename="package://irb1660/meshes/link_4.dae"/>
      </geometry>
    </visual>
  </link>

  <!-- Joint 5 -->
  <joint name="joint_5" type="revolute">
    <origin xyz="0.75 0.00005 1.1865" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <parent link="link_4"/>
    <child link="link_5"/>
    <limit effort="10.0" lower="-2.094" upper="2.094" velocity="2.4435"/>
  </joint>

  <!-- Link 5 -->
  <link name="link_5">
    <visual name="visual">
      <origin xyz="-1.514 -0.24405 -4.1715" rpy="0 0 0" />
      <geometry>
        <mesh filename="package://irb1660/meshes/link_5.dae"/>
      </geometry>
    </visual>
  </link>

  <!-- Joint 6 -->
  <joint name="joint_6" type="revolute">
    <origin xyz="0.717 0.00005 1.1865" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <parent link="link_5"/>
    <child link="link_6"/>
    <limit effort="10.0" lower="-6.283" upper="6.283" velocity="3.3161"/>
  </joint>

  <!-- Link 6 -->
  <link name="link_6">
    <visual name="visual">
      <origin xyz="-2.231 -0.24410 -5.358" rpy="0 0 0" />
      <geometry>
        <mesh filename="package://irb1660/meshes/link_6.dae"/>
      </geometry>
    </visual>
  </link>

</robot>
```
     
3. **Save and Close**
---
# Explanation of URDF File
Let's walk through the `irb1660_robot.urdf` file step by step, explaining each part and how the information for each line was obtained.

### Header
```xml
<?xml version="1.0"?>
<robot name="irb1660_robot">
```
- **Explanation:** 
  - `<?xml version="1.0"?>` is a standard XML declaration indicating the version of XML being used.
  - `<robot name="irb1660_robot">` defines the root element of the URDF and assigns a name to the robot model. This name is used by tools like RViz and Gazebo to identify the robot. It’s a good practice to name it after the specific robot model, in this case, "irb1660_robot".

### Base Link
```xml
  <!-- Base Link -->
  <link name="base_link">
    <visual name="visual">
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <mesh filename="package://irb1660/meshes/base_link.dae"/>
      </geometry>
    </visual>
  </link>
```
- **Explanation:** 
  - `<link name="base_link">` defines a rigid body in the robot, named "base_link". This is the base of the robot from which all other parts are connected.
  - `<visual name="visual">` specifies the visual representation of this link. It includes:
    - `<origin xyz="0 0 0" rpy="0 0 0" />`: Defines the position (`xyz`) and orientation (`rpy` – roll, pitch, yaw) of the visual geometry relative to the link’s origin.
    - `<geometry>`: Describes the shape of the link. Here, it references a 3D mesh file (`base_link.dae`) located in the `meshes` directory of the package.
  - **How Information Was Obtained:** 
    - The mesh file is typically created using CAD software and exported in a format like `.dae` (Collada). The origin and orientation are set to zero because the mesh file is usually aligned with the base link’s frame by default.

### Joint 1 and Link 1
```xml
  <!-- Joint 1 -->
  <joint name="joint_1" type="revolute">
    <origin xyz="0 0 0.1255" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <parent link="base_link"/>
    <child link="link_1"/>
    <limit effort="10.0" lower="-2.967" upper="2.967" velocity="1.7453"/>
  </joint>

  <!-- Link 1 -->
  <link name="link_1">
    <visual name="visual">
      <origin xyz="0 0 -0.1255" rpy="0 0 0" />
      <geometry>
        <mesh filename="package://irb1660/meshes/link_1.dae"/>
      </geometry>
    </visual>
  </link>
```
- **Explanation:** 
  - `<joint name="joint_1" type="revolute">` defines a revolute joint (a hinge allowing rotation) named "joint_1".
    - `<origin xyz="0 0 0.1255" rpy="0 0 0"/>`: Specifies the position and orientation of the joint relative to the parent link’s frame. Here, the joint is slightly offset in the z-axis.
    - `<axis xyz="0 0 1"/>`: Indicates that the joint rotates around the z-axis.
    - `<parent link="base_link"/>` and `<child link="link_1"/>`: Define the parent and child links connected by this joint.
    - `<limit>`: Specifies constraints for the joint:
      - `effort="10.0"`: Maximum torque or force the joint can apply.
      - `lower` and `upper`: Angular limits for the joint in radians.
      - `velocity`: Maximum speed of the joint.
  - `<link name="link_1">`: Defines the next link connected to "base_link" through "joint_1".
    - The `<visual>` element describes its visual appearance, similar to "base_link".
  - **How Information Was Obtained:**
    - The joint’s position and orientation (`origin`) are determined based on the robot’s mechanical design. Limits like `effort`, `lower`, `upper`, and `velocity` are derived from the robot’s physical specifications, typically provided by the manufacturer.

### Subsequent Joints and Links
- The file continues in a similar pattern, defining joints and links for the rest of the robot's arm.

  For example:
```xml
  <!-- Joint 2 -->
  <joint name="joint_2" type="revolute">
    <origin xyz="0.15 0.137 0.4865" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <parent link="link_1"/>
    <child link="link_2"/>
    <limit effort="10.0" lower="-1.134" upper="1.4855" velocity="1.5707"/>
  </joint>

  <!-- Link 2 -->
  <link name="link_2">
    <visual name="visual">
      <origin xyz="-0.15 -0.137 -0.612" rpy="0 0 0" />
      <geometry>
        <mesh filename="package://irb1660/meshes/link_2.dae"/>
      </geometry>
    </visual>
  </link>
```
- **Explanation:** 
  - The process for defining each joint and link is similar to the first one:
    - `origin`, `axis`, `parent`, `child`, `limit` for joints.
    - `origin`, `geometry`, and `mesh` for links.
  - **How Information Was Obtained:** 
    - Again, the positions, orientations, and limits come from the robot’s mechanical design and specifications.

### Closing Tags
```xml
</robot>
```
- **Explanation:** 
  - This closing tag ends the robot description in the URDF file.

### **Summary:**
- The URDF file is structured to describe each part of the robot’s kinematic chain, starting from the base link and moving out through each joint and link.
- **Information Sources:**
  - The data for joint limits, axes, and origins typically comes from the robot’s technical documentation or CAD models.
  - Mesh files are created using 3D modeling software and linked in the URDF for visualization.
---


