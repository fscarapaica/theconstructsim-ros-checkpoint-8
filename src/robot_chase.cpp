#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;

class RobotChase : public rclcpp::Node {
public:
  RobotChase() : Node("robot_chase_node") {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("rick/cmd_vel", 10);

    timer_ = this->create_wall_timer(
        100ms, std::bind(&RobotChase::timer_callback, this));
  }

private:
  void timer_callback() {

    try {
      transformStamped =
          tf_buffer_->lookupTransform(fromFrame, toFrame, tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
                  toFrame.c_str(), fromFrame.c_str(), ex.what());
      return;
    }

    static const double kp_yaw = 0.5;
    static const double kp_distance = 0.5;

    twist_msg.angular.z =
        kp_yaw * atan2(transformStamped.transform.translation.x,
                       transformStamped.transform.translation.y);

    twist_msg.linear.x =
        kp_distance *
        sqrt(pow(transformStamped.transform.translation.x, 2) +
             pow(transformStamped.transform.translation.y, 2)) *
        -1;

    publisher_->publish(twist_msg);
  }

  // cmd_vel control
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  geometry_msgs::msg::Twist twist_msg;
  geometry_msgs::msg::TransformStamped transformStamped;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string fromFrame = "rick/base_link";
  std::string toFrame = "morty/base_link";
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotChase>());
  rclcpp::shutdown();
  return 0;
}