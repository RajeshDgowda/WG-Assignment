# Assignment: Order Optimizer for AMR Systems

This project involves the development of a ROS node named ‘OrderOptimizer’ for autonomous mobile robot (AMR) systems. The node is designed to optimize the path for AMR based on given orders and product configurations. 

## Key Features:
- Subscribing to topics: ‘currentPosition’ and ‘nextOrder’.
- Parsing YAML files for orders and product configurations.
- Calculating the geometrically shortest path for AMR to collect all parts.
- Visualizing the AMR position and part pickup locations.

## Technical Details:
The solution is developed in C++ and is intended to run on Ubuntu 20.04 LTS using ROS2 Foxy Fitzroy. The code should be platform-independent and is hosted in a public Gitlab repository.


## AMR Systems

The `amr_systems` project is designed to optimize the order processing for Autonomous Mobile Robots (AMRs). It subscribes to a custom message `order.msg` to receive order details and calculates the optimal path for the robot to fetch products.

## Directory Structure

```
amr_systems/
│
├── .git/                 # Git configuration and source directory
├── .gitignore            # Specifies intentionally untracked files to ignore
│
├── CMakeLists.txt        # CMake configuration file
│
├── include/
│   └── amr_systems/
│       └── order_optimizer.hpp   # Header file for the order optimizer
│
├── src/
│   └── order_optimizer.cpp      # Source file for the order optimizer
│
├── test/
│   └── test_order_optimizer.cpp # Test file for the order optimizer
│
└── package.xml           # Package information for ROS2
```

## Dependencies

- ROS2
- `order_msg` package: This package contains the custom message `order.msg` which is used to communicate order details.

## How to Use

1. **Setup ROS2 Environment**: Ensure you have ROS2 installed and sourced.

2. **Clone the Repository**:
   ```bash
   git clone [repository_url]
   ```

3. **Build the Workspace**:
   ```bash
   colcon build --packages-select amr_systems
   ```

4. **Run the `order_optimizer_node`**:
   ```bash
   ros2 run amr_systems order_optimizer_node
   ```

5. **Publish a Custom Message**:
   You can manually publish an `order.msg` message using the console. Here's an example:
   ```bash
   ros2 topic pub /nextOrder order_msg/Order "{order_id: 1, description: 'Sample Order'}"
   ```

## Contributing

If you'd like to contribute to this project, please fork the repository, make your changes, and submit a pull request. We'll review your contributions and integrate them into the main branch.

---

This `README.md` provides an overview of the project, its directory structure, dependencies, and basic usage instructions. You can further customize and expand upon this content based on the specific details and requirements of your project.
