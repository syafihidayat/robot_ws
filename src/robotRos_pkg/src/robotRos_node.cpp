#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/bool.hpp>
// #include <visualization_msgs/msg/marker.hpp>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2/LinearMath/Matrix3x3.h>
#include <pid.hpp>
#include <convertion.hpp>
#include <cmath>

#define PWM_MAX 2.0
#define PWM_MIN -2.0

#define kp 0.5
#define ki 0.0
#define kd 0.0

#define kpT 1.0
#define kiT 0.0
#define kdT 0.0

PID omni_distance(PWM_MIN, PWM_MAX, kp, ki, kd);
PID omni_angular(PWM_MIN, PWM_MAX, kpT, kiT, kdT);

class Movement : public rclcpp::Node
{

public:
  Movement() : Node("Movement_Point")
  //  omni_distance(PWM_MIN, PWM_MAX, kp, ki, kd),
  //  omni_angular(PWM_MIN, PWM_MAX, kpT,kiT, kdT)
  {

    cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("/omni_cont/cmd_vel", 10);

    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10,
                                                                  std::bind(&Movement::odom_callback, this, std::placeholders::_1));

    pose_sub = this->create_subscription<geometry_msgs::msg::Point>("/pose", 10,
                                                                    std::bind(&Movement::pose_callback, this, std::placeholders::_1));

    // pose2_sub = this->create_subscription<geometry_msgs::msg::Point>("/pose_steps", 10,
    //                                                                  std::bind(&Movement::pose2_callback, this, std::placeholders::_1));

    reached_pub = this->create_publisher<std_msgs::msg::Bool>("/target_reached", 10);

    next_step_pub = this->create_publisher<std_msgs::msg::Bool>("/next_step", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&Movement::control_loop, this));

    start_received = false;
    target_received = false;
    current_state = WAITING_FOR_TARGET;
    prevT = this->now().seconds();

    RCLCPP_INFO(this->get_logger(), "waiting for target");
  }

private:
  enum State
  {
    WAITING_FOR_TARGET,
    MOVING_TO_TARGET,
    TARGET_REACHED,
    STAGE2_MOVE_STEPbSTEP
  };

  int stage1_target_count = 0;
  int stage1_targets_total = 2;
  bool stage1_completed = false;

  double startX, startY;
  double targetX, targetY;
  double currentX, currentY;
  bool start_received;
  bool target_received;
  State current_state;
  double prevT;

  bool stage2_initiallized = false;
  double stage2_posX = 0;
  double stage2_posY = 0;
  double stage2_yaw = 0;
  double stage2_target_distance = 0.0;
  int move_count = 0;

  double convertation(const nav_msgs::msg::Odometry &odom_robot)
  {

    Convertion convert;

    Convertion::Quaternion robot_quat = {
        odom_robot.pose.pose.orientation.w,
        odom_robot.pose.pose.orientation.x,
        odom_robot.pose.pose.orientation.y,
        odom_robot.pose.pose.orientation.z};
    double odom_robot_yaw, odom_robot_pitch, odom_robot_roll;
    convert.quat_to_eular(robot_quat, odom_robot_yaw, odom_robot_pitch, odom_robot_roll);

    return odom_robot_yaw;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {

    odom_robot_msg = *msg;
    odom_robot_step = *msg;

    currentX = msg->pose.pose.position.x;
    currentY = msg->pose.pose.position.y;

    if (!start_received)
    {
      startX = currentX;
      startY = currentY;
      start_received = true;
    }
  }

  void pose_callback(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    if (current_state == WAITING_FOR_TARGET || current_state == TARGET_REACHED)
    {
      targetX = msg->x;
      targetY = msg->y;
      target_received = true;
      current_state = MOVING_TO_TARGET;

      RCLCPP_INFO(this->get_logger(), "New target STAGE1 received: (%.2f, %.2f)", targetX, targetY);
      RCLCPP_INFO(this->get_logger(), "Current position: (%.2f, %.2f)", currentX, currentY);
    }
  }

  void control_loop()
  {
    if (!start_received)
      return;

    // if (current_state == MOVING_TO_TARGET && !target_received)
    //   return;

    if ((current_state == WAITING_FOR_TARGET || current_state == MOVING_TO_TARGET) && !target_received)
      return;

    double currT = this->now().seconds();
    float deltaT = currT - prevT;

    geometry_msgs::msg::Twist cmd;

    switch (current_state)
    {
    case WAITING_FOR_TARGET:
      cmd.linear.x = 0.0;
      cmd.linear.y = 0.0;
      cmd.angular.z = 0.0;

      if (target_received)
      {
        current_state = MOVING_TO_TARGET;
        RCLCPP_INFO(this->get_logger(), "🎯 Starting movement to target %d/%d",
                    stage1_target_count + 1, stage1_targets_total);
      }
      break;

    case MOVING_TO_TARGET:
    {
      double dx = targetX - currentX;
      double dy = targetY - currentY;

      double odom_robot_yaw = convertation(odom_robot_msg);
      double theta = 0 - odom_robot_yaw;

      double distance = std::sqrt(dx * dx + dy * dy);
      double angle = std::atan2(dy, dx);

      float control_distance = omni_distance.control_base(distance, deltaT);
      float control_angle = omni_angular.control_base_rotation(theta, deltaT);

      if (distance > 0.05)
      {

        cmd.linear.x = control_distance * std::cos(angle);
        cmd.linear.y = control_distance * std::sin(angle);
        cmd.angular.z = control_angle;
        RCLCPP_INFO(this->get_logger(), "move robot");
      }
      else
      {
        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        cmd.angular.z = 0.0;

        current_state = TARGET_REACHED;
        stage1_target_count++;
        // current_state = STAGE2_MOVE_STEPbSTEP;
        // target_received = false;

        std_msgs::msg::Bool reached_msg;
        reached_msg.data = true;
        reached_pub->publish(reached_msg);

        RCLCPP_INFO(this->get_logger(), "✅ Target %d/%d reached!", stage1_target_count, stage1_targets_total);

        if (stage1_target_count >= stage1_targets_total)
        {
          stage1_completed = true;
          RCLCPP_INFO(this->get_logger(), "target reached");
          RCLCPP_INFO(this->get_logger(), "STAGE1 COMPLATED");
        }
        else
        {

          RCLCPP_INFO(this->get_logger(), "wait for next target");
        }
        // break;
      }
      break;
    }

    case STAGE2_MOVE_STEPbSTEP:
    {

      if (!stage2_initiallized)
      {

        stage2_posX = currentX;
        stage2_posY = currentY;
        stage2_target_distance = 0.5;
        stage2_yaw = convertation(odom_robot_msg);
        stage2_initiallized = true;
        move_count++;

        RCLCPP_INFO(this->get_logger(), "..................................................");
        RCLCPP_INFO(this->get_logger(), "stage 2 starting move step by step");
      }

      // double stage2_target_distance = current_block * block_distance_meter;
      double stage2_targetX = stage2_posX + stage2_target_distance * std::cos(stage2_yaw);
      double stage2_targetY = stage2_posY + stage2_target_distance * std::sin(stage2_yaw);

      double stage2_dx = stage2_targetX - currentX;
      double stage2_dy = stage2_targetY - currentY;
      double stage2_distance_error = std::sqrt(stage2_dx * stage2_dx + stage2_dy * stage2_dy);
      double stage2_angle = std::atan2(stage2_dy, stage2_dx);

      // double theta = 0 - stage2_yaw;

      if (stage2_distance_error > 0.03)
      {
        float control_distance = omni_distance.control_base(stage2_distance_error, deltaT);
        // float control_angle = omni_angular.control_base_rotation(theta,deltaT);

        cmd.linear.x = control_distance * std::cos(stage2_angle);
        cmd.linear.y = control_distance * std::sin(stage2_angle);
        cmd.angular.z = 0;
        // cmd.angular.z = control_angle;
      }
      else
      {

        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        cmd.angular.z = 0.0;

        stage2_initiallized = false;

        RCLCPP_INFO(this->get_logger(), "✅ Stage2: movement completed! Preparing next move...");
      }

      break;
    }
    case TARGET_REACHED:
      cmd.linear.x = 0.0;
      cmd.linear.y = 0.0;
      cmd.angular.z = 0.0;

      if (stage1_completed)
      {
        current_state = STAGE2_MOVE_STEPbSTEP;
        stage1_target_count = 0;
        stage1_completed = false;

        RCLCPP_INFO(this->get_logger(), " start moving stage2");
        RCLCPP_INFO(this->get_logger(), "==============================================================");
      }
      else if (target_received)
      {
        current_state = MOVING_TO_TARGET;
        RCLCPP_INFO(this->get_logger(), "🎯 Starting movement to target %d/%d", stage1_target_count + 1, stage1_targets_total);
      }
      break;
    }
    prevT = currT;
    cmd_pub->publish(cmd);
  }
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr pose_sub;
  // rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr pose2_sub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reached_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr next_step_pub;
  nav_msgs::msg::Odometry odom_robot_msg;
  nav_msgs::msg::Odometry odom_robot_step;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Movement>());
  rclcpp::shutdown();
  return 0;
}