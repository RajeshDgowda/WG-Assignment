# Assignment: Order Optimizer for AMR Systems

This project involves the development of a ROS node named ‘OrderOptimizer’ for autonomous mobile robot (AMR) systems. The node is designed to optimize the path for AMR based on given orders and product configurations. 

## Key Features:
- Subscribing to topics: ‘currentPosition’ and ‘nextOrder’.
- Parsing YAML files for orders and product configurations.
- Calculating the geometrically shortest path for AMR to collect all parts.
- Visualizing the AMR position and part pickup locations.

## Technical Details:
The solution is developed in C++ and is intended to run on Ubuntu 20.04 LTS using ROS2 Foxy Fitzroy. The code should be platform-independent and is hosted in a public Gitlab repository.


## AMR Systems ROS2 Package

This document will guide you through the process of building, running, and testing the package.

## **Building the Workspace**

Before diving into running or testing the package, it's essential to ensure that the workspace is correctly built.

- **Navigating to the Workspace**: 
  Start by moving to the root directory of your ROS 2 workspace. This directory is often named `ros_ws` or `workspace`, but it might have a different name based on your setup.
   ```bash
   cd ~/ros_ws
   ```

- **Setting Up the ROS 2 Environment**: 
  Ensure that your terminal is aware of the ROS 2 environment by sourcing the setup file. Replace `<ros2_distro>` with your ROS 2 distribution's name, like `foxy` or `galactic`.
   ```bash
   source /opt/ros/<ros2_distro>/setup.bash
   ```

- **Building**: 
  Now, initiate the build process using `colcon`.
   ```bash
   colcon build --symlink-install
   ```

## **Running the Code**

Once the build is successful, you're all set to run the node.

- **Sourcing the Workspace**: 
  This step makes sure that your terminal recognizes the newly built packages and nodes.
   ```bash
   source ~/ros_ws/install/setup.bash
   ```

- **Executing the Node**: 
  Use the following command to run the `order_optimizer_node` from the `amr_systems` package.
   ```bash
   ros2 run amr_systems order_optimizer_node
   ```

## **Testing the Package**

If the package contains tests configured in the `CMakeLists.txt`, you can execute them as follows:

- **Building the Tests**: 
  This command specifically targets the tests in the `amr_systems` package.
   ```bash
   colcon build --symlink-install --packages-select amr_systems --cmake-target tests
   ```

- **Executing the Tests**: 
  Run the tests using `colcon`.
   ```bash
   colcon test --packages-select amr_systems
   ```

- **Reviewing Test Results**: 
  After running the tests, you can view a summary of the results.
   ```bash
   colcon test-result --all
   ```

## **Additional Notes**

- Ensure you have a properly set up ROS 2 environment on your machine.
  
- Ensure all dependencies are correctly installed and available in your workspace.

- If you face any issues, the terminal's output can be a valuable resource. It often provides detailed error messages that can help diagnose problems.

- This guide assumes the package's name is `amr_systems` and the primary node is `order_optimizer_node`. If your setup uses different names, adjust the commands accordingly.
