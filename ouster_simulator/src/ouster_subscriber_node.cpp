#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class OusterSubscriber : public rclcpp::Node
{
public:
  OusterSubscriber()
  : Node("ouster_subscriber"), first_message_received_(false)
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/data", 10, std::bind(&OusterSubscriber::topicCallback, this, std::placeholders::_1));
  }

private:
  void topicCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    rclcpp::Time current_time = this->now();

    if (first_message_received_) {
      double delta_seconds = (current_time - prev_time_).seconds();
      if (delta_seconds > 0.0) {
        double frequency = 1.0 / delta_seconds;
        RCLCPP_INFO(this->get_logger(), "Got data at frequency: %.2f Hz", frequency);
      }
    } else {
      first_message_received_ = true;
    }

    prev_time_ = current_time;
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Time prev_time_;
  bool first_message_received_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OusterSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

