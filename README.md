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

## order_msg Structure
```markdown
order_msg/
│
├── CMakeLists.txt        # CMake configuration file for the package
│
├── msg/
│   └── Order.msg         # Custom ROS message definition for Order
│
└── package.xml           # Package information for ROS2
```

## How to Use

#Note: As per the instruction given in the guide there are absolute paths to parse the `.yaml` files to test the code please change the path in ` order_optimizer.cpp`to the absolute path of your ROS2 workspace. 

1. **Setup ROS2 Environment**:
   Ensure you have ROS2 installed and sourced.

3. **Clone the Repository**:
   ```bash
   git clone https://github.com/RajeshDgowda/WG-Assignment.git
   ```

4. **Build the Workspace**:
   ```bash
   colcon build --packages-select amr_systems
   ```

5. **Run the `order_optimizer_node`**:
   ```bash
   ros2 run amr_systems order_optimizer_node
   ```

6. **Publish a Custom Message**:
   
   You can manually publish an `order.msg` message using the console. Here's an example:
   ```bash
   ros2 topic pub /nextOrder order_msg/Order "{order_id: 1000002, description: 'Sample Order'}"
   ```

