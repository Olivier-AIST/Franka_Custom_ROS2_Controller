<h1 style="font-size: 3em;">ROS 2 custom controller</h1>



#### Table of Contents
- [About](#about)
- [Setup](#setup)
  - [Local Machine Installation](#local-machine-installation)
- [Test the Setup](#test-the-setup)
- [Contributing](#contributing)
- [License](#license)
- [Contact](#contact)

# About
This project is based on the original Franka ROS 2 repository, with a custom controller implemented to extend its capabilities. The added functionalities include:

- ROS 2 custom topics controller  
- Modular interpolation of trajectories  
- Trajectory replay  
- Motion smoothing  
- ROS 2 grasping control
- Trajectory visualization
- 
# Franka ROS 2 Dependencies Setup

This repository contains a `.repos` file that helps you clone the required dependencies for Franka ROS 2. You also need to build ruckig on the top.

## Prerequisites

- Franka ros2 [https://github.com/frankarobotics/franka_ros2]
- ruckig [https://github.com/pantor/ruckig]
- Python (3.13 tested)
- ubuntu 22.04
- ROS humble

## Local Machine Installation
1. **Install ROS 2 Development environment**

    _**franka_ros2**_ is built upon _**ROS 2 Humble**_.

    To set up your ROS 2 environment, follow the official _**humble**_ installation instructions provided [**here**](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).
    The guide discusses two main installation options: **Desktop** and **Bare Bones**.

    #### Choose **one** of the following:
    - **ROS 2 "Desktop Install"** (`ros-humble-desktop`)
      Includes a full ROS 2 installation with GUI tools and visualization packages (e.g., Rviz and Gazebo).
      **Recommended** for users who need simulation or visualization capabilities.

    - **"ROS-Base Install (Bare Bones)"** (`ros-humble-ros-base`)
      A minimal installation that includes only the core ROS 2 libraries.
      Suitable for resource-constrained environments or headless systems.

    ```bash
    # replace <YOUR CHOICE> with either ros-humble-desktop or ros-humble-ros-base
    sudo apt install <YOUR CHOICE>
    ```
    ---
    Also install the **Development Tools** package:
    ```bash
    sudo apt install ros-dev-tools
    ```
    Installing the **Desktop** or **Bare Bones** should automatically source the **ROS 2** environment but, under some circumstances you may need to do this again:
    ```bash
    source /opt/ros/humble/setup.sh
    ```

2. **Create a ROS 2 Workspace:**
   ```bash
   mkdir -p ~/franka_ros2_ws/src
   cd ~/franka_ros2_ws  # not into src
   ```
3. **Clone the Repositories:**
   ```bash
    git clone https://github.com/frankarobotics/franka_ros2.git src
    ```
4. **Install the dependencies**
    ```bash
    vcs import src < src/franka.repos --recursive --skip-existing
    ```
5. **Detect and install project dependencies**
   ```bash
   rosdep install --from-paths src --ignore-src --rosdistro humble -y
   ```
6. **Build**
   ```bash
   # use the --symlinks option to reduce disk usage, and facilitate development.
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```
7. **Adjust Enviroment**
   ```bash
   # Adjust environment to recognize packages and dependencies in your newly built ROS 2 workspace.
   source install/setup.sh
   ```


# Test the build
   ```bash
   colcon test
   ```
> Warnings can be expected.

# Run a sample ROS 2 application

To verify that your franka ros2 lib works correctly without a robot, you can run the following command to use dummy hardware:

```bash
ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=dont-care use_fake_hardware:=true
```

# Install ruckig
You need to install ruckig in the folder of your choice or in a global install. But I recommand to build it in the _src/libfranka/include_ folder.

After that you can put the right path in the **arm_sim2real.cpp** includes and compile. 
Check this repo : https://github.com/pantor/ruckig


# Run the ROS2 custom controller

Navigate to the folder:
```bash
cd src/libfranka/build
```
Compile:
```bash
make arm_sim2real
```
Execute :
```bash
./arm_sim2real --ip 192.168.1.299 --filter 0.15
```

The program will record joint states from the topic **/joint_states_isaac** during 10 sec. After that it will filter the trajectory and make a quintic interpolation to the right control cycle. You can just press enter and the robot will play the interpolated trajectory

> **Note:** The filtering parameter can be tuned depending of your use case. It is an important parameter as it directly impact the smoothness of the motion. A high value will make the trajectory less accurate but smoother as it will have less intermediate positions. A low value will respect more the inputed trajectory but can conduct to some jiggle depending how good is the inputed trajectory.

# Run the ROS2 gripper boolean controller

Navigate to the folder:
```bash
cd src/libfranka/build
```
Compile:
```bash
make grasp_object
```
Execute :
```bash
./grasp_object 
```

I also designed a controller for the gripper . It will listen until  command is activated on the topic /gripper command. It will activate a grasp commandat max speed. You can change the width in the code.

> **Note:** As the gripper and the arm of the franka are independent we can connect to the twice at the same time. You just need to open two terminals and execute them in parralel!


# Contributing

Contributions are welcome!

## License

All packages of this project are licensed under the Apache 2.0 license.

## Contact



