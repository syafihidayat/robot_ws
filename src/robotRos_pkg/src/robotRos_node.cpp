#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>

class Movement : public rclcpp::Node
{
public:
  Movement() : Node("Movement_Point")
  {
    cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("/omni_cont/cmd_vel", 10);

    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10,
      std::bind(&Movement::odom_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&Movement::control_loop, this));

    target_distance = 1.0;
    start_received = false;
  }

  double startX,startY;
  double currentX,currentY;
  double target_distance;
  bool start_received;

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    currentX = msg->pose.pose.position.x;
    currentY = msg->pose.pose.position.y;

    if (!start_received)
    {
      startX = currentX;
      startY = currentY;
      start_received = true;
    }
  }

  void control_loop()
  {
    if (!start_received)
      return;

    double dx = currentX - startX;
    double dy = currentY - startY;

    double distance_traveled = std::sqrt(dx * dx + dy * dy);

    double error = target_distance - distance_traveled;

    geometry_msgs::msg::Twist cmd;

    if (error > 0.01)
    {
      cmd.linear.x = 0.2;
      cmd.linear.y = 0.0;
      cmd.angular.z = 0.0;
      RCLCPP_INFO(this->get_logger(), "move forward");
    }
    else
    {
      cmd.linear.x = 0.0;
      cmd.linear.y = 0.0;
      cmd.angular.z = 0.0;
      timer_->cancel();
    }

    cmd_pub->publish(cmd);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub; 
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char**argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Movement>());
  rclcpp::shutdown();
  return 0;
}

