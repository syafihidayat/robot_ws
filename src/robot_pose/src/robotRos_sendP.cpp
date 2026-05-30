#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>

class waypointPublish : public rclcpp::Node
{
public:
  waypointPublish() : Node("send_pose")
  {
    pose_pub = this->create_publisher<geometry_msgs::msg::Point>("/pose", 10);

    reached_sub = this->create_subscription<std_msgs::msg::Bool>(
      "/target_reached", 10,
      std::bind(&waypointPublish::reached_callback, this, std::placeholders::_1));

    ir_sub = this->create_subscription<std_msgs::msg::Bool>(
      "/infraReceive", 10,
      std::bind(&waypointPublish::ir_callback, this, std::placeholders::_1));

    lifter_pub = this->create_publisher<std_msgs::msg::Bool>("lifter_control", 10);

    limit_sub = this->create_subscription<std_msgs::msg::Bool>("limitSlideData", 10,
      std::bind(&waypointPublish::limit_callback, this, std::placeholders::_1));

    exit_pos_sub = this->create_subscription<geometry_msgs::msg::Point>("/stage2_exit_pos", 10,
      std::bind(&waypointPublish::exit_pos_callback, this, std::placeholders::_1));

    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg){
        current_robot_x = msg->pose.pose.position.x;
        current_robot_y = msg->pose.pose.position.y;
      });

    // toStage2_pub = this->create_publisher<std_msgs::msg::Bool>("toStage2", 10);



    waypoint = {

      {0.2, 0.0}
      // {0.0, 1.0}

      // {0.0, 1.0},
      // {1.0, 1.0},
      // {1.0, -1.5},
      // {1.5, -1.5}
    };

    current_waypoint_index = 0;
    waiting_ir = false;
    reached_latched = false;
    all_completed = false;

    RCLCPP_INFO(this->get_logger(), "Waypoint Publisher Start");
    send_waypoint();
  }

private:

  enum class WP_STATE
  {
    MOVING,
    WAIT_IR,
    COMPLETE
  };

  WP_STATE state = WP_STATE::MOVING;

  bool waiting_ir = false;
  bool reached_latched = false;
  bool all_completed = false;
  double exit_x = 0.0,exit_y = 0.0;
  double current_robot_x = 0.0 , current_robot_y = 0.0;
  bool limit_triggered = false;
  bool exit_pos_received = false;
  bool stage2_done = false;
  bool exit_pos_pushed = false;
  bool stage1_done = false;

  size_t current_waypoint_index;

  // ===================== SEND WAYPOINT =====================
  void send_waypoint()
  {
    if (current_waypoint_index >= waypoint.size())
    {
      all_completed = true;
      state = WP_STATE::COMPLETE;
      RCLCPP_INFO(this->get_logger(), "ALL WAYPOINTS COMPLETE");
      return;
    }

    geometry_msgs::msg::Point point;
    point.x = waypoint[current_waypoint_index].first;
    point.y = waypoint[current_waypoint_index].second;
    point.z = 0.0;

    pose_pub->publish(point);

    reached_latched = false;

    RCLCPP_INFO(this->get_logger(),"Sending waypoint %ld (%.2f, %.2f)",current_waypoint_index,point.x, point.y);
  }

  // ===================== ADVANCE (ONLY ONE ENTRY POINT) =====================
  void advance_waypoint()
  {
    current_waypoint_index++;
    waiting_ir = false;
    state = WP_STATE::MOVING;

    send_waypoint();
  }

  // ===================== REACHED CALLBACK =====================
  void reached_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (all_completed) return;
    if (!msg->data) return;
    if (reached_latched) return;   // anti double trigger

    reached_latched = true;

    RCLCPP_INFO(this->get_logger(), "WAYPOINT REACHED: %ld", current_waypoint_index);

    // WP0 action
    if (current_waypoint_index == 0)
    {
      std_msgs::msg::Bool lifter_msg;
      lifter_msg.data = true;
      lifter_pub->publish(lifter_msg);

      RCLCPP_INFO(this->get_logger(), "Trigger lifter turun");
    }

    // WP1 → WAIT IR
    if (current_waypoint_index == 1 && !stage1_done)
    {
      state = WP_STATE::WAIT_IR;
      waiting_ir = true;

      RCLCPP_INFO(this->get_logger(), "WAITING IR...");
      return; 
    }

    // WP lainnya langsung lanjut
    advance_waypoint();
  }

  void limit_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if(all_completed) return;
    if(!msg->data) return;
    if(limit_triggered) return;
    if(reached_latched) return;
    if(state != WP_STATE::MOVING) return;

    if(current_waypoint_index != 0) return;

    limit_triggered = true;
    reached_latched = true;

    RCLCPP_INFO(this->get_logger(), "LIMIT HIT WP0 → Anggap Reached + Lifter turun");

    std_msgs::msg::Bool lifter_msg;
    lifter_msg.data = true;
    lifter_pub->publish(lifter_msg);

    waypoint[1].second = current_robot_y;
    RCLCPP_INFO(this->get_logger(),"WP1 di-snap ke Y robot: %.3f", current_robot_y);

    advance_waypoint();

  }
  
  void exit_pos_callback(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    exit_x = msg->x;
    exit_y = msg->y;

    exit_pos_received = true;
    stage2_done = true;
    all_completed = false;
    stage1_done = true;

    reached_latched = false;
    waiting_ir = false;
    limit_triggered = false;

    state = WP_STATE::MOVING;

    waypoint.push_back({exit_x, exit_y + -1.1});
    waypoint.push_back({exit_x + 2.0 , exit_y + -1.0});

    current_waypoint_index = waypoint.size() - 2;
    
    // RCLCPP_INFO(this->get_logger(), "STAGE 3 WAYPOINTS: geser=(%.2f,%.2f) maju=(%.2f,%.2f)",
    // exit_x, exit_y - 1.0, exit_x + 1.0, exit_y - 1.0);

    RCLCPP_INFO(this->get_logger(),
    "STAGE 3 WAYPOINTS: geser=(%.2f,%.2f) maju=(%.2f,%.2f)",
    waypoint[waypoint.size()-2].first,
    waypoint[waypoint.size()-2].second,
    waypoint[waypoint.size()-1].first,
    waypoint[waypoint.size()-1].second);


    send_waypoint();
    // RCLCPP_INFO(this->get_logger(), "EXIT POS RECEIVED: (%.2f, %.2f)", exit_x, exit_y);

  }

  void ir_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (all_completed) return;
    if (!msg->data) return;

    if (state != WP_STATE::WAIT_IR)
      return;

    RCLCPP_INFO(this->get_logger(), "IR RECEIVED → CONTINUE");

    advance_waypoint();
  }

  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pose_pub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reached_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ir_sub;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr exit_pos_sub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lifter_pub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr limit_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  // rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr toStage2_pub;

  std::vector<std::pair<double, double>> waypoint;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<waypointPublish>());
  rclcpp::shutdown();
  return 0;
}