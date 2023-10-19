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

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "amr_systems/order_optimizer.hpp"
#include <yaml-cpp/yaml.h>
#include <set>

// Define a testing fixture for OrderOptimizer
class TestOrderOptimizer : public ::testing::Test {
protected:
    // Set up rclcpp initialization before each test
    void SetUp() override {
        rclcpp::init(0, nullptr);
    }

    // Shut down rclcpp after each test
    void TearDown() override {
        rclcpp::shutdown();
    }
};

// Test case to validate that the product configurations are loaded properly
TEST_F(TestOrderOptimizer, LoadConfigurationTest) {
    auto optimizer = std::make_shared<OrderOptimizer>();

    // Validate if the optimizer successfully loaded the products configuration
    ASSERT_FALSE(optimizer->productsEmpty()) << "Failed to load products configuration";

    // Load products from the YAML file for a comparative check
    YAML::Node products = YAML::LoadFile("/home/rajesh/ros2_ws/src/amr_systems/configuration/products.yaml");

    // Assert if the number of products loaded by the optimizer matches the count in the YAML file
    ASSERT_EQ(optimizer->productsSize(), products.size()) << "Mismatch in number of products loaded";
}

// Test case to check if the order processing mechanism works
TEST_F(TestOrderOptimizer, OrderProcessingTest) {
    auto optimizer = std::make_shared<OrderOptimizer>();

    // Define a sample order for testing
    order_msg::msg::Order test_order;
    test_order.order_id = 1000002;  
    test_order.description = "Test Order";

    // Call the order processing callback
    optimizer->nextOrderCallback(std::make_shared<order_msg::msg::Order>(test_order));

    // Here, we're only ensuring that there were no exceptions during the order processing.
    // Extended validations can be incorporated based on desired functionality.
}

// Test case to validate that all product IDs in the orders are valid
TEST_F(TestOrderOptimizer, ValidateOrderProductsTest) {
    auto optimizer = std::make_shared<OrderOptimizer>();
    
    // Load product and order configurations
    YAML::Node products = YAML::LoadFile("/home/rajesh/ros2_ws/src/amr_systems/configuration/products.yaml");
    YAML::Node orders = YAML::LoadFile("/home/rajesh/ros2_ws/src/amr_systems/orders/orders_20201202.yaml");

    // Store all product IDs for validation
    std::set<int> productIds;
    for (const auto& product : products) {
        productIds.insert(product["id"].as<int>());
    }

    // Validate that all product IDs in the orders are valid
    for (const auto& order : orders) {
        for (const auto& productId : order["products"]) {
            ASSERT_TRUE(productIds.find(productId.as<int>()) != productIds.end()) << "Order " << order["order"].as<int>() << " has invalid product ID: " << productId.as<int>();
        }
    }
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);

    // Set filter to exclude tests that match the patterns "copyright.*", "uncrustify.*", and "cpplint.*"
    testing::GTEST_FLAG(filter) = "-copyright.*:-uncrustify.*:-cpplint.*";
    return RUN_ALL_TESTS();
}
