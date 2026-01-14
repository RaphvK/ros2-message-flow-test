#include <functional>

#include <test_publisher/test_publisher.hpp>
#include <tracetools/tracetools.h>


namespace test_publisher {


TestPublisher::TestPublisher() : Node("test_publisher") {

  this->setup();
}


void TestPublisher::setup() {

  // subscriber for handling incoming messages
  subscriber_ = this->create_subscription<std_msgs::msg::Int32>("~/input", 10, std::bind(&TestPublisher::topicCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", subscriber_->get_topic_name());
  subscriber2_ = this->create_subscription<std_msgs::msg::Int32>("~/input2", 10, std::bind(&TestPublisher::topicCallback2, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", subscriber2_->get_topic_name());

  // publisher for publishing outgoing messages
  publisher_ = this->create_publisher<std_msgs::msg::Int32>("~/output", 10);
  RCLCPP_INFO(this->get_logger(), "Publishing to '%s'", publisher_->get_topic_name());

  std::vector<const void *> link_subs;
  std::vector<const void *> link_pubs;
  link_subs.push_back(static_cast<const void *>(subscriber_->get_subscription_handle().get()));
  link_subs.push_back(static_cast<const void *>(subscriber2_->get_subscription_handle().get()));
  link_pubs.push_back(static_cast<const void *>(publisher_->get_publisher_handle().get()));
  TRACETOOLS_TRACEPOINT(message_link_partial_sync, link_subs.data(), link_subs.size(), link_pubs.data(), link_pubs.size());
}


void TestPublisher::topicCallback(const std_msgs::msg::Int32::ConstSharedPtr& msg) {

  RCLCPP_INFO(this->get_logger(), "Message received: '%d'", msg->data);

  // publish message
  if(msg2_data_ == msg->data) {
    std_msgs::msg::Int32 out_msg;
    out_msg.data = msg->data;
    publisher_->publish(out_msg);
    RCLCPP_INFO(this->get_logger(), "Message published: '%d' including '%d'", out_msg.data, msg2_data_);
  }
}

void TestPublisher::topicCallback2(const std_msgs::msg::Int32::ConstSharedPtr& msg) {

  RCLCPP_INFO(this->get_logger(), "Message (2) received: '%d'", msg->data);

  msg2_data_ = msg->data;
}


}


int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  auto node = std::make_shared<test_publisher::TestPublisher>();
  rclcpp::executors::SingleThreadedExecutor executor;
  RCLCPP_INFO(node->get_logger(), "Spinning node '%s' with %s", node->get_fully_qualified_name(), "SingleThreadedExecutor");
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();

  return 0;
}
