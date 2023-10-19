/*
 * Copyright (c) [2023] [Rajesh Doddegowda/Student Research Assistant @ university paderborn]
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * ... [Other licensing information]
 *
 */

#include "amr_systems/order_optimizer.hpp"

// Constructor initializes the ROS node, subscriptions, publishers, and loads configuration.
OrderOptimizer::OrderOptimizer() : Node("OrderOptimizer") {
    // Declare and initialize directory path parameter
    this->declare_parameter<std::string>("directory_path", "/home/rajesh/ros2_ws/src/amr_systems");
    directoryPath_ = this->get_parameter("directory_path").as_string();

    // Set up subscriptions
    currentPositionSub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "currentPosition", 10, std::bind(&OrderOptimizer::currentPositionCallback, this, std::placeholders::_1));
    nextOrderSub_ = this->create_subscription<order_msg::msg::Order>(
        "nextOrder", 10, std::bind(&OrderOptimizer::nextOrderCallback, this, std::placeholders::_1));
    
    // Set up publisher
    orderPathPub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("order_path", 10);
    
    // Load configuration data
    loadConfiguration();
}

// Callback that updates the current position of the robot
void OrderOptimizer::currentPositionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    currentPos_ = *msg;
}

// Callback to handle a new order based on the incoming message
void OrderOptimizer::nextOrderCallback(const order_msg::msg::Order::SharedPtr msg) {
    handleOrder(msg->order_id, msg->description);  
}

// Processes the received order
void OrderOptimizer::handleOrder(uint32_t order_id, const std::string& description) {
    std::vector<Order> orders;  // Stores orders read from configuration
    std::vector<std::thread> threads;  // Threads for parsing order files (if multi-threading was implemented)
    std::mutex orders_mutex;  // Mutex for thread-safe access to orders vector

    // Check if the directory path exists
    if (!std::filesystem::exists(directoryPath_)) {
        RCLCPP_ERROR(this->get_logger(), "Directory not found: %s", directoryPath_.c_str());
        return;
    }

    // Define the directory containing order configurations
    std::filesystem::path orders_directory = directoryPath_ / "orders";

    // Iterate over files in the orders directory
    for (const auto& entry : std::filesystem::directory_iterator(orders_directory)) {
        if (entry.path().extension() == ".yaml") {  // Only process YAML files
            try {
                YAML::Node config = YAML::LoadFile(entry.path());
                std::vector<Order> file_orders;

                // Parse order configurations from the file
                for (const auto& orderNode : config) {
                    Order order;
                    order.order_id = orderNode["order"].as<uint32_t>();
                    order.cx = orderNode["cx"].as<float>();
                    order.cy = orderNode["cy"].as<float>();
                    order.products = orderNode["products"].as<std::vector<int>>();
                    file_orders.push_back(order);
                }

                // Append parsed orders to the main orders list
                orders.insert(orders.end(), file_orders.begin(), file_orders.end());
            }
            catch (const YAML::Exception& e) {
                std::cerr << "Error parsing YAML file " << entry.path() << ": " << e.what() << std::endl;
            }
        }
    }

    // Join all threads (would be used if multi-threaded order parsing was implemented)
    for (auto& thread : threads) {
        thread.join();
    }

    // Find and process the requested order
    bool orderFound = false;
    Order currentOrder;
    for (const auto& order : orders) {
        if (order.order_id == order_id) {
            currentOrder = order;
            orderFound = true;
            break;
        }
    }
    if (!orderFound) {
        RCLCPP_ERROR(this->get_logger(), "No order found with ID: %u", order_id);
        return;
    }

    // Calculate and log the path for the order
    std::vector<Product::Part> path = calculateShortestPath(currentOrder);
    std::stringstream output;
    output << "Working on order " << order_id << " (" << description << ")\n";
    int count = 1;
    std::string lastProductName = ""; // To keep track of the last product name

    for (const auto& part : path) {
        for (const auto& product_id : currentOrder.products) {
            for (const auto& product : products_) {
                if (product.id == product_id) {
                    // Check if the current product name is different from the last product name
                    if (product.product != lastProductName) {
                        output << count++ << ". Fetching " << part.part_name << " for " << product.product << " at x: " << part.cx << ", y: " << part.cy << "\n";
                        lastProductName = product.product; // Update the last product name
                    }
                }
            }
        }
    }


    output << count << ". Delivering to destination x: " << currentOrder.cx << ", y: " << currentOrder.cy;
    RCLCPP_INFO(this->get_logger(), output.str().c_str());
}

// Load product configurations from the specified YAML file
void OrderOptimizer::loadConfiguration() {
    std::filesystem::path configPath = directoryPath_ / "configuration" / "products.yaml";
    if (!std::filesystem::exists(configPath)) {
        RCLCPP_ERROR(this->get_logger(), "Configuration file not found at %s", configPath.c_str());
        return;
    }

    // Parse product configuration from the YAML file
    YAML::Node config = YAML::LoadFile(configPath);
    products_.clear();
    for (const auto& productNode : config) {
        Product product;
        product.id = productNode["id"].as<int>();
        product.product = productNode["product"].as<std::string>();
        for (const auto& partNode : productNode["parts"]) {
            Product::Part part;
            part.part_name = partNode["part"].as<std::string>();
            part.cx = partNode["cx"].as<double>();
            part.cy = partNode["cy"].as<double>();
            product.parts.push_back(part);
        }
        products_.push_back(product);
    }
}

// Calculate the path to collect all parts for a given order
std::vector<Product::Part> OrderOptimizer::calculateShortestPath(const Order& order) {
    std::vector<Product::Part> path;
    std::vector<Product::Part> allParts;

    // Gather all parts for the given order
    for (const int product_id : order.products) {
        for (const Product& product : products_) {
            if (product.id == product_id) {
                for (const Product::Part& part : product.parts) {
                    allParts.push_back(part);
                }
                break;
            }
        }
    }

    // Current position
    double currentX = currentPos_.pose.position.x;
    double currentY = currentPos_.pose.position.y;

    while (!allParts.empty()) {
        double minDistance = std::numeric_limits<double>::max();
        size_t nearestPartIndex = 0;

        // Find the nearest part
        for (size_t i = 0; i < allParts.size(); i++) {
            double distance = std::sqrt(std::pow(allParts[i].cx - currentX, 2) + std::pow(allParts[i].cy - currentY, 2));
            if (distance < minDistance) {
                minDistance = distance;
                nearestPartIndex = i;
            }
        }

        // Add the nearest part to the path and update the current position
        path.push_back(allParts[nearestPartIndex]);
        currentX = allParts[nearestPartIndex].cx;
        currentY = allParts[nearestPartIndex].cy;

        // Remove the nearest part from allParts
        allParts.erase(allParts.begin() + nearestPartIndex);
    }

    return path;
}


// Main function to initialize and run the ROS node
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OrderOptimizer>());
    rclcpp::shutdown();
    return 0;
}
