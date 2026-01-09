#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>


namespace test_publisher {



/**
 * @brief TestPublisher class
 */
class TestPublisher : public rclcpp::Node {

 public:

  TestPublisher();

 private:

  int msg2_data_;

  /**
   * @brief Sets up subscribers, publishers, etc. to configure the node
   */
  void setup();

  /**
   * @brief Processes messages received by a subscriber
   *
   * @param msg message
   */
  void topicCallback(const std_msgs::msg::Int32::ConstSharedPtr& msg);
  void topicCallback2(const std_msgs::msg::Int32::ConstSharedPtr& msg);

 private:

  /**
   * @brief Subscriber
   */
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscriber_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscriber2_;

  /**
   * @brief Publisher
   */
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
};


}
