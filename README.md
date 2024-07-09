## RandomNav2Controller

RandomNav2Controller is a ROS 2 package that utilizes the Navigation 2 (Nav2) stack to control a TurtleBot3 robot in a simulated environment. The robot is programmed to navigate to random positions within a specified area.

### Features
- Set initial pose of the robot
- Navigate to a given pose
- Navigate to random poses within a specified area

### Requirements
- Ubuntu Linux - Jammy Jellyfish (22.04)

### Installation

#### Install Dependent ROS 2 Packages

1. Install the ROS 2 binary packages as described in the [official ROS 2 docs](https://docs.ros.org/en/humble/Installation.html).
2. Open a terminal with `Ctrl+Alt+T`.
3. Install Gazebo:
    ```sh
    sudo apt install ros-humble-gazebo-*
    ```
4. Install Cartographer:
    ```sh
    sudo apt install ros-humble-cartographer
    sudo apt install ros-humble-cartographer-ros
    ```
5. Install the Nav2 packages:
    ```sh
    sudo apt install ros-humble-navigation2
    sudo apt install ros-humble-nav2-bringup
    ```
6. Install the TurtleBot3 packages:
    ```sh
    sudo apt install ros-humble-turtlebot3-gazebo
    ```

#### Install TurtleBot3 Packages

1. Source your ROS 2 environment:
    ```sh
    source /opt/ros/humble/setup.bash
    ```
2. Install TurtleBot3 packages via Debian:
    ```sh
    sudo apt install ros-humble-dynamixel-sdk
    sudo apt install ros-humble-turtlebot3-msgs
    sudo apt install ros-humble-turtlebot3
    ```

### Execution

1. Start a terminal in your GUI.
2. Set key environment variables:
    ```sh
    source /opt/ros/humble/setup.bash
    export TURTLEBOT3_MODEL=waffle
    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
    ```
3. In the same terminal, launch the TurtleBot3 simulation:
    ```sh
    ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False
    ```
4. Open another terminal and clone the RandomNav2Controller repository:
    ```sh
    cd ~/
    git clone https://github.com/Juanpalai/RandomNav2Controller.git
    cd RandomNav2Controller
    colcon build
    source install/setup.bash
    ```
5. Run the `nav2_controller` node:
    ```sh
    ros2 run turtlebot3_nav2 nav2_controller
    ```

Now you can watch the TurtleBot3 move to different areas within the simulation.
