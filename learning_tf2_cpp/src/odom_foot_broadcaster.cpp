#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

class OdomToMapAndBaseScanPublisher : public rclcpp::Node
{
public:
  OdomToMapAndBaseScanPublisher()
  : Node("odom_to_map_and_footprint_publisher")
  {
    // Khởi tạo TransformBroadcaster cho odom -> map và map -> base_footprint
    tf_broadcaster_odom_to_map = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    tf_broadcaster_map_to_base_footprint = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Khởi tạo timer để phát biến đổi định kỳ (10 Hz)
    timer_ = this->create_wall_timer(
      100ms, std::bind(&OdomToMapAndBaseScanPublisher::publish_transform, this));

    // Biến lưu trữ vị trí và góc quay cho các phép biến đổi
    x_ = 0.0;
    y_ = 0.0;
    theta_ = 0.0;
  }

private:
  void publish_transform()
  {
    // Phát biến đổi từ odom -> map
    geometry_msgs::msg::TransformStamped odom_to_map;
    odom_to_map.header.stamp = this->get_clock()->now();
    odom_to_map.header.frame_id = "map";
    odom_to_map.child_frame_id = "odom";

    // Gán vị trí (translation) cho odom -> map
    odom_to_map.transform.translation.x = x_;
    odom_to_map.transform.translation.y = y_;
    odom_to_map.transform.translation.z = 0.0;

    // Tạo quaternion từ góc yaw (theta) cho odom -> map
    tf2::Quaternion q_odom_to_map;
    q_odom_to_map.setRPY(0, 0, theta_);
    odom_to_map.transform.rotation.x = q_odom_to_map.x();
    odom_to_map.transform.rotation.y = q_odom_to_map.y();
    odom_to_map.transform.rotation.z = q_odom_to_map.z();
    odom_to_map.transform.rotation.w = q_odom_to_map.w();

    // Phát biến đổi
    tf_broadcaster_odom_to_map->sendTransform(odom_to_map);

    // Phát biến đổi từ map -> base_footprint
    geometry_msgs::msg::TransformStamped map_to_base_footprint;
    map_to_base_footprint.header.stamp = this->get_clock()->now();
    map_to_base_footprint.header.frame_id = "odom";
    map_to_base_footprint.child_frame_id = "base_footprint";

    // Gán vị trí (translation) cho map -> base_footprint
    map_to_base_footprint.transform.translation.x = -1.0;
    map_to_base_footprint.transform.translation.y = -1.0;
    map_to_base_footprint.transform.translation.z = 0.0;

    // Tạo quaternion từ góc yaw (theta) cho map -> base_footprint
    tf2::Quaternion q_map_to_base_footprint;
    q_map_to_base_footprint.setRPY(0, 0, 0.5236);  // Không xoay
    map_to_base_footprint.transform.rotation.x = q_map_to_base_footprint.x();
    map_to_base_footprint.transform.rotation.y = q_map_to_base_footprint.y();
    map_to_base_footprint.transform.rotation.z = q_map_to_base_footprint.z();
    map_to_base_footprint.transform.rotation.w = q_map_to_base_footprint.w();

    // Phát biến đổi
    tf_broadcaster_map_to_base_footprint->sendTransform(map_to_base_footprint);
  }

  // Các biến của class
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_odom_to_map;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_map_to_base_footprint;
  rclcpp::TimerBase::SharedPtr timer_;
  double x_, y_, theta_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomToMapAndBaseScanPublisher>());
  rclcpp::shutdown();
  return 0;
}
