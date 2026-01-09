#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

using namespace std::chrono_literals;

namespace test_publisher {

class DummyPublisher : public rclcpp::Node {
public:
  DummyPublisher() : Node("dummy_publisher") {
    publisher_ = this->create_publisher<std_msgs::msg::Int32>("~/topic", 10);
    publisher2_ = this->create_publisher<std_msgs::msg::Int32>("~/topic2", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&DummyPublisher::timerCallback, this));
    RCLCPP_INFO(this->get_logger(), "Publishing to '%s'", publisher_->get_topic_name());
  }

private:
  void timerCallback() {
    auto message = std_msgs::msg::Int32();
    message.data = count_++;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.data);
    publisher2_->publish(message);
    std::this_thread::sleep_for(1ms);
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher2_;
  int count_ = 0;
};

} // namespace test_publisher

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<test_publisher::DummyPublisher>());
  rclcpp::shutdown();
  return 0;
}
