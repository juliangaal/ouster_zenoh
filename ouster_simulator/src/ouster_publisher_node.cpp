#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

// ROS 2
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

using namespace std::chrono_literals;

/**
 * @brief A ROS 2 node that publishes a 128×1024 point cloud using
 *        a custom layout matching the 'ouster_ros::Point' fields:
 * 
 *        x (float), y (float), z (float), 
 *        intensity (float), 
 *        t (uint32_t), 
 *        reflectivity (uint16_t), 
 *        ring (uint16_t), 
 *        ambient (uint16_t), 
 *        range (uint32_t).
 *
 *        see https://github.com/ouster-lidar/ouster-ros/blob/ros2/ouster-ros/include/ouster_ros/os_point.h
 */
class OusterPublisher : public rclcpp::Node
{
public:
  OusterPublisher()
  : Node("ouster_publisher")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/data", 10);

    timer_ = this->create_wall_timer(
      50ms, std::bind(&OusterPublisher::onTimer, this));

    RCLCPP_INFO(this->get_logger(), "OusterPublisher node started.");
  }

private:
  void onTimer()
  {
    auto cloud_msg = generatePointCloud();
    publisher_->publish(cloud_msg);
    RCLCPP_INFO(this->get_logger(), "Published.");
  }

  sensor_msgs::msg::PointCloud2 generatePointCloud()
  {
    // Dimensions
    static constexpr uint32_t HEIGHT = 128;
    static constexpr uint32_t WIDTH  = 1024;

    // We have 9 fields, totalling 32 bytes per point:
    //   offset | name          | type      | bytes
    //   -------+---------------+-----------+------
    //       0  | x            | float32   | 4
    //       4  | y            | float32   | 4
    //       8  | z            | float32   | 4
    //      12  | intensity    | float32   | 4
    //      16  | t            | uint32    | 4
    //      20  | reflectivity | uint16    | 2
    //      22  | ring         | uint16    | 2
    //      24  | ambient      | uint16    | 2
    //      26  | (padding)    | uint16    | 2  (for alignment)
    //      28  | range        | uint32    | 4
    //
    //     total per point: 32 bytes
    //
    // Because we need alignment for the 'range' field, we introduce 2 bytes
    // of padding after 'ambient' to make 'range' start at a multiple of 4.

    static float total_iterations = 0.0;

    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.stamp = now();
    cloud_msg.header.frame_id = "os_lidar";
    cloud_msg.height = HEIGHT;
    cloud_msg.width  = WIDTH;
    cloud_msg.is_bigendian = false;
    cloud_msg.is_dense = false;

    cloud_msg.fields.resize(9);

    cloud_msg.fields[0].name     = "x";
    cloud_msg.fields[0].offset   = 0;
    cloud_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[0].count    = 1;

    cloud_msg.fields[1].name     = "y";
    cloud_msg.fields[1].offset   = 4;
    cloud_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[1].count    = 1;

    cloud_msg.fields[2].name     = "z";
    cloud_msg.fields[2].offset   = 8;
    cloud_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[2].count    = 1;

    cloud_msg.fields[3].name     = "intensity";
    cloud_msg.fields[3].offset   = 12;
    cloud_msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[3].count    = 1;

    cloud_msg.fields[4].name     = "t";
    cloud_msg.fields[4].offset   = 16;
    cloud_msg.fields[4].datatype = sensor_msgs::msg::PointField::UINT32;
    cloud_msg.fields[4].count    = 1;

    cloud_msg.fields[5].name     = "reflectivity";
    cloud_msg.fields[5].offset   = 20;
    cloud_msg.fields[5].datatype = sensor_msgs::msg::PointField::UINT16;
    cloud_msg.fields[5].count    = 1;

    cloud_msg.fields[6].name     = "ring";
    cloud_msg.fields[6].offset   = 22;
    cloud_msg.fields[6].datatype = sensor_msgs::msg::PointField::UINT16;
    cloud_msg.fields[6].count    = 1;

    cloud_msg.fields[7].name     = "ambient";
    cloud_msg.fields[7].offset   = 24;
    cloud_msg.fields[7].datatype = sensor_msgs::msg::PointField::UINT16;
    cloud_msg.fields[7].count    = 1;

    cloud_msg.fields[8].name     = "range";
    cloud_msg.fields[8].offset   = 28;
    cloud_msg.fields[8].datatype = sensor_msgs::msg::PointField::UINT32;
    cloud_msg.fields[8].count    = 1;

    cloud_msg.point_step = 32;
    cloud_msg.row_step   = cloud_msg.point_step * cloud_msg.width;
    cloud_msg.data.resize(cloud_msg.row_step * cloud_msg.height);

    sensor_msgs::PointCloud2Iterator<float>   iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float>   iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float>   iter_z(cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<float>   iter_intensity(cloud_msg, "intensity");
    sensor_msgs::PointCloud2Iterator<uint32_t> iter_t(cloud_msg, "t");
    sensor_msgs::PointCloud2Iterator<uint16_t> iter_reflectivity(cloud_msg, "reflectivity");
    sensor_msgs::PointCloud2Iterator<uint16_t> iter_ring(cloud_msg, "ring");
    sensor_msgs::PointCloud2Iterator<uint16_t> iter_ambient(cloud_msg, "ambient");
    sensor_msgs::PointCloud2Iterator<uint32_t> iter_range(cloud_msg, "range");

    // Fill the point cloud in row-major order
    // Simple pattern:
    //   x = (float) col
    //   y = sin(col / 50.0 + row)
    //   z = (float) row
    //   intensity = row + col
    //   t = synthetic timestamp
    //   reflectivity = row
    //   ring = col
    //   ambient = row*col mod 65535
    //   range = (col * 100 + row)
    for (uint32_t row = 0; row < HEIGHT; ++row)
    {
      for (uint32_t col = 0; col < WIDTH; ++col)
      {
        *iter_x = static_cast<float>(col);
        *iter_y = std::sin(static_cast<float>(col) / 50.0f + row);
        *iter_z = static_cast<float>(row); // static offset to slowly move cloud towards the sky

        *iter_intensity = static_cast<float>(row + col);

        *iter_t = static_cast<uint32_t>((row * 100000) + col); // pseudorandom

        *iter_reflectivity = static_cast<uint16_t>(row);
        *iter_ring         = static_cast<uint16_t>(col % 65535);
        *iter_ambient      = static_cast<uint16_t>((row * col) % 65535);

        *iter_range = static_cast<uint32_t>((col * 100) + row);

        ++iter_x; ++iter_y; ++iter_z; ++iter_intensity;
        ++iter_t;
        ++iter_reflectivity; ++iter_ring; ++iter_ambient;
        ++iter_range;
	total_iterations++;
      }
    }

    return cloud_msg;
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OusterPublisher>());
  rclcpp::shutdown();
  return 0;
}
