#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

class MapToOdomPublisher : public rclcpp::Node
{
public:
  MapToOdomPublisher()
  : Node("map_to_odom_publisher")
  {
    // Khởi tạo TransformBroadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Khởi tạo timer để phát biến đổi định kỳ (10 Hz)
    timer_ = this->create_wall_timer(
      100ms, std::bind(&MapToOdomPublisher::publish_transform, this));

    // Biến lưu trữ vị trí và góc quay
    x_ = 0.0;
    y_ = 0.0;
    theta_ = 0.0;
  }

private:
  void publish_transform()
  {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "map";
    t.child_frame_id = "odom";

    // Gán vị trí (translation)
    t.transform.translation.x = x_;
    t.transform.translation.y = y_;
    t.transform.translation.z = 0.0;

    // Tạo quaternion từ góc yaw (theta)
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t);
  }

  // Các biến của class
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  double x_, y_, theta_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapToOdomPublisher>());
  rclcpp::shutdown();
  return 0;
}
