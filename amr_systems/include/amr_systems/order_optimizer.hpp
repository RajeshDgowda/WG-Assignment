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

#ifndef ORDER_OPTIMIZER_HPP
#define ORDER_OPTIMIZER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "order_msg/msg/order.hpp"
#include <yaml-cpp/yaml.h>
#include <vector>
#include <filesystem>
#include <thread>
#include <queue>
#include <mutex>

// Represents a Product with its ID, name and associated parts
class Product {
public:
    int id;
    std::string product;

    // Represents a part of a product with its name and coordinates
    struct Part {
        std::string part_name;
        double cx, cy;
    };

    std::vector<Part> parts;
};

class TestOrderOptimizer; // Forward declaration for friend class

// Class OrderOptimizer is responsible for optimizing order handling
// based on current position and products configuration.
class OrderOptimizer : public rclcpp::Node {
public:
    OrderOptimizer(); // Constructor

    friend class TestOrderOptimizer; // For unit testing purposes

    // Checks if products are loaded or not
    bool productsEmpty() const {
        return products_.empty();
    }

    // Returns the number of products loaded
    size_t productsSize() const {
        return products_.size();
    }

    // Provides access to the loaded products
    const std::vector<Product>& getProducts() const { return products_; }

    // Callback for handling current robot position updates
    void currentPositionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    // Callback for handling new orders
    void nextOrderCallback(const order_msg::msg::Order::SharedPtr msg);

    // Processes the provided order details
    void handleOrder(uint32_t order_id, const std::string& description);

    // Represents an order with its ID, target position, and associated product IDs
    struct Order {
        uint32_t order_id;
        float cx, cy;
        std::vector<int> products;
    };

private:
    // Loads the products configuration
    void loadConfiguration();

    // Calculates the shortest path for handling a given order
    std::vector<Product::Part> calculateShortestPath(const Order& order);

    // ROS2 Subscriptions and Publishers
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr currentPositionSub_;
    rclcpp::Subscription<order_msg::msg::Order>::SharedPtr nextOrderSub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr orderPathPub_;
    
    geometry_msgs::msg::PoseStamped currentPos_; // Current position of the robot
    std::vector<Product> products_;              // List of loaded products
    std::filesystem::path directoryPath_;        // Path for the products configuration directory
};

#endif // ORDER_OPTIMIZER_HPP
