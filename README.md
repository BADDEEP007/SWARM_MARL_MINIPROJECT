# ROS 2 Workspace

This is a ROS 2 workspace containing custom packages. Follow the instructions below to set up and run the workspace on your system.

## Prerequisites
- **Operating System:** Ubuntu 22.04 (or compatible with ROS 2 Humble)
- **ROS 2 Version:** Humble
- **Dependencies:** Ensure you have ROS 2 installed. If not, install it using:
  ```bash
  sudo apt update && sudo apt install ros-humble-desktop
  ```

## Clone the Repository
To get started, clone this repository into your home directory:
```bash
cd ~
git clone <your-repository-url> ros2_ws
cd ros2_ws
```

## Install Dependencies
Before building the workspace, install required dependencies using `rosdep`:
```bash
rosdep install --from-paths src --ignore-src -r -y
```

## Build the Workspace
After installing dependencies, build the workspace using `colcon`:
```bash
colcon build
```

## Source the Workspace
Once the build is complete, source the setup file:
```bash
source install/setup.bash
```
To make this change permanent, add it to your `~/.bashrc`:
```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Running Your Package
To run a specific package, use the following command:
```bash
ros2 run <package_name> <executable_name>
```
Replace `<package_name>` and `<executable_name>` with the actual package and executable names.

## Troubleshooting
### Common Issues & Fixes
1. **Package not found error**
   - Ensure the workspace is built and sourced:
     ```bash
     colcon build
     source install/setup.bash
     ```
2. **Missing dependencies**
   - Run `rosdep install` again:
     ```bash
     rosdep install --from-paths src --ignore-src -r -y
     ```
3. **Wrong ROS version**
   - Ensure you're using ROS 2 Humble:
     ```bash
     echo $ROS_DISTRO
     ```

## Contributing
If you want to contribute, fork the repository, make changes, and submit a pull request.

## License
This project is licensed under the MIT License.

