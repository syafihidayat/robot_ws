#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include "gui_kfs_msgs/msg/kfs_decision.hpp"
#include <pid.hpp>
#include <convertion.hpp>
#include <cmath>
#include "grid_planner.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// #define PWM_MAX 3.0  // 1.5
// #define PWM_MIN -3.0 // 1.5

// #define kp 1.0   //0.8 // 1.0
// #define ki 0.001
// #define kd 0.0

// #define kpT 1.5
// #define kiT 0.0
// #define kdT 0.0

// PID omni_distance(PWM_MIN, PWM_MAX, kp, ki, kd);
// PID omni_angular(PWM_MIN, PWM_MAX, kpT, kiT, kdT);

GridPlanner planner;
PID omni_distance;
PID omni_angular;

class Movement : public rclcpp::Node
{

public:
  Movement() : Node("Movement_Point")
  {

    cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("/omni_cont/cmd_vel", 10);

    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10,
    std::bind(&Movement::odom_callback, this, std::placeholders::_1));

    pose_sub = this->create_subscription<geometry_msgs::msg::Point>("/pose", 10,
    std::bind(&Movement::pose_callback, this, std::placeholders::_1));

    proxy_sub = this->create_subscription<std_msgs::msg::Bool>("proxydata", 10,
    std::bind(&Movement::proxy_callback, this, std::placeholders::_1));

    waypoint_backend_sub = this->create_subscription<geometry_msgs::msg::Point>("/planner/waypoint", 10,
    std::bind(&Movement::waypoint_backend_callback, this, std::placeholders::_1));

    gui_sub = this->create_subscription<gui_kfs_msgs::msg::KFSDecision>("kfs_decision", 10,
    std::bind(&Movement::gui_callback, this, std::placeholders::_1));

    toStage2_sub = this->create_subscription<std_msgs::msg::Bool>("toStage2", 10,
    std::bind(&Movement::toStage2_callback, this, std::placeholders::_1));

    // boundingBox_sub = this->create_subscription<geometry_msgs::msg::Point>("coordinate_boundingBox", 10,
    //                                                                        std::bind(&Movement::coordinate_callback, this, std::placeholders::_1));

    tof_sub = this->create_subscription<std_msgs::msg::UInt16>("tof_distance", 10,
    std::bind(&Movement::tof_callback, this, std::placeholders::_1));

    tof_send_pub = this->create_publisher<std_msgs::msg::UInt16>("tof_send_back", 10);

    lifter2_sub = this->create_subscription<std_msgs::msg::Bool>("/lifter_down2", 10,
    std::bind(&Movement::lifter2_callback, this, std::placeholders::_1));

    reached_pub = this->create_publisher<std_msgs::msg::Bool>("/target_reached", 10);

    infraReceive_sub = this->create_subscription<std_msgs::msg::Bool>("infraReceive", 10,
      std::bind(&Movement::infra_callback, this, std::placeholders::_1));

    proxy_true_pub = this->create_publisher<std_msgs::msg::Bool>("/true_sensor_proxy", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&Movement::control_loop, this));

    start_received = false;
    target_received = false;
    current_state = WAITING_FOR_TARGET;
    prevT = this->now().seconds();

    float kp, ki, kd;
    float kpT, kiT, kdT;
    // float desired_linear_vel, max_angular_vel;

    this->declare_parameter("kp", 1.0);
    this->declare_parameter("ki", 0.0);
    this->declare_parameter("kd", 0.0);

    // this->declare_parameter("kp", 1.0);
    // this->declare_parameter("ki", 0.0001);
    // this->declare_parameter("kd", 0.0);

    this->declare_parameter("kpT", 3.0);
    this->declare_parameter("kiT", 0.0);
    this->declare_parameter("kdT", 0.0);

    this->get_parameter("kp", kp);
    this->get_parameter("ki", ki);
    this->get_parameter("kd", kd);

    this->get_parameter("kpT", kpT);
    this->get_parameter("kiT", kiT);
    this->get_parameter("kdT", kdT);

    this->declare_parameter("desired_linear_vel", 3.0);
    this->declare_parameter("max_angular_vel", 3.0);

    this->get_parameter("desired_linear_vel", desired_linear_vel);
    this->get_parameter("max_angular_vel", max_angular_vel);

    omni_distance.setBaseParam(kp, ki, kd);
    omni_angular.setHeadingParam(kpT, kiT, kdT);

    RCLCPP_INFO(this->get_logger(), "waiting for target");
  }

private:
  static bool swing_reset_done;

  enum State
  {
    WAITING_FOR_TARGET,
    MOVING_TO_TARGET,
    TARGET_REACHED,
    PAUSED_STAGE1,
    STAGE2_MOVE_STEPbSTEP
  };

  // enum Stage2state
  // {
  //   MOVE_FORWARD,
  //   ROTATE,
  //   SEARCH_TARGET,
  //   ALIGN_BY_YOLO,
  //   APPROACH_TARGET,
  //   COMPLETE
  // };

  enum Stage2Substate
  {
    ST2_PLAN,
    ST2_ROTATE,
    ST2_MOVE,
    // ST2_CHECK_YOLO,
    // ST2_APPROACH,
    ST2_DONE
  };

  float desired_linear_vel, max_angular_vel;

  int stage1_target_count = 0;
  int stage1_targets_total = 3;
  bool stage1_completed = false;

  bool camera_active = false;
  bool stage1_complated_for_camera = false;

  double startX, startY;
  double targetX, targetY;
  double currentX, currentY;
  bool start_received;
  bool target_received;
  bool sensor_obstacle = false;
  bool sensor_obstacle2 = false;
  bool tof_valid = false;
  bool was_obstacle = false;
  State current_state;
  double prevT;
  bool ignore_obstacle = false;
  bool target_reached_flag = false;
  bool paused = false;
  bool just_resumed = false;



  bool stage2_initiallized = false;
  double target_Astar_X, target_Astar_Y;
  bool receive_waypoint_backend = false;
  bool rotating = false;
  double stage2_start_y = 0;
  double stage2_start_x = 0;
  double stage2_yaw = 0;
  // int current_block = 1;
  Stage2Substate stage2_substate = ST2_MOVE;
  // Stage2state stage2_substate = MOVE_FORWARD;
  State previous_state_before_pause;
  double rotation_direction = 1;
  double rotate_target_angle = 0.0;
  bool rotation_completed = false;
  bool yolo_valid = false;
  float yolo_errorX = 0.0;
  float yolo_depth = -1.0;
  float target = 0.0;

  int last_known_target_cell = -1;

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

    odom_robot_yaw = -odom_robot_yaw;

    RCLCPP_INFO(this->get_logger(), "heading %.2f", odom_robot_yaw);

    return odom_robot_yaw;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {

    Convertion cnvrt;

    Convertion::Quaternion rbt_q = {
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
    };
    double yaw, roll, pitch;
    cnvrt.quat_to_eular(rbt_q, yaw, pitch, roll);

    odom_robot_msg = *msg;

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
    // if(!target_received)
    // {
      targetX = msg->x;
      targetY = msg->y;
      target_received = true;
      target_reached_flag = false;
      // paused = false;
      
      // std_msgs::msg::Bool reached_msg;
      // reached_msg.data = false;
      // reached_pub->publish(reached_msg);
      
      current_state = MOVING_TO_TARGET;

      RCLCPP_INFO(this->get_logger(), "New target STAGE1 received: (%.2f, %.2f)", targetX, targetY);
      RCLCPP_INFO(this->get_logger(), "Current position: (%.2f, %.2f)", currentX, currentY);
    }
  }

  bool last_detected = false;
  bool obstacle_lock = false;

  void proxy_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {

    bool detected = msg->data;
    bool new_obstacle = !detected;
    

    if(new_obstacle != sensor_obstacle)
    {
      RCLCPP_INFO(this->get_logger(), "OBSTACLE FLAG: %s", new_obstacle ? "STOP" : "GO");
    }
    
    sensor_obstacle = !detected;
    
    if( detected != last_detected)
    {
      RCLCPP_INFO(this->get_logger(), "SENSOR RAW: %s", detected ? "TRUE" : "FALSE");
      
    }

    if(!detected)
    {
      obstacle_lock = true;
    }

    last_detected = detected;

    std_msgs::msg::Bool out_msg;
    out_msg.data = msg->data;
    proxy_true_pub->publish(out_msg);

    if (sensor_obstacle && !ignore_obstacle)
    {

      if (current_state == MOVING_TO_TARGET)
      {
        previous_state_before_pause = current_state;
        current_state = PAUSED_STAGE1;

        if (previous_state_before_pause == MOVING_TO_TARGET)
        {
          RCLCPP_INFO(this->get_logger(), "Stage 1 PAUSED");
        }
      }
    }

    was_obstacle = sensor_obstacle;
  }

  double tof_distance = 0;
  #define TOF_THRESHOLD 100

  void tof_callback(const std_msgs::msg::UInt16::SharedPtr msg)
  {
    uint16_t distance = msg->data;

    if(distance > 0)
    {
      tof_distance = distance;
      tof_valid = true;
      RCLCPP_DEBUG(this->get_logger(), "ToF distance: %u mm", distance);

      std_msgs::msg::UInt16 send_back_msg;
      send_back_msg.data = msg->data;
      tof_send_pub->publish(send_back_msg);
    } 

    if(distance > 0 && distance < TOF_THRESHOLD && !ignore_obstacle)
    {
      if(current_state == MOVING_TO_TARGET)
      {
        previous_state_before_pause = current_state;
        current_state = PAUSED_STAGE1;

        RCLCPP_INFO(this->get_logger(), "stage 1 PAUSED by Tof");
      }
    }
  }

  // void coordinate_callback(const geometry_msgs::msg::Point::SharedPtr msg)
  // {

  //   if (!camera_active)
  //     return;
  //   yolo_errorX = msg->x;
  //   yolo_depth = msg->y;
  //   target = msg->z;

  //   yolo_valid = (yolo_depth > 0.0);
  // }

  void infra_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data)
    {
      RCLCPP_INFO(this->get_logger(), "IR DETECTED");
    }
  }


  bool stage2_flag = false;
  void toStage2_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {

    if(msg->data)
    {
      stage2_flag = true;
      // RCLCPP_INFO(this->get_logger(), "LIMIT TOUCHED");
    }
  }

  bool lifter_down2_flag = false;

  void lifter2_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if(msg->data)
    {
      lifter_down2_flag = true;
      ignore_obstacle = true;
      RCLCPP_INFO(this->get_logger(), "LIFTER DOWN 2 DETECTED");
    }
  }


  bool has_active_target = false;

  void waypoint_backend_callback(const geometry_msgs::msg::Point::SharedPtr msg)
  {

    if (!has_active_target)
    // if ((current_state == WAITING_FOR_TARGET || current_state == STAGE2_MOVE_STEPbSTEP) && !has_active_target)
    {

      target_Astar_X = msg->x;
      target_Astar_Y = msg->y;
      receive_waypoint_backend = true;
      has_active_target = true;
    }
  }

  bool decision_receive = false;

  void gui_callback(const gui_kfs_msgs::msg::KFSDecision::SharedPtr msg)
  {
    decision_receive = true;
    RCLCPP_INFO(this->get_logger(), "Decision received -> robot boleh jalan");

  }

  void control_loop()
  {
    if (!start_received)
      return;

    if ((current_state == WAITING_FOR_TARGET || current_state == MOVING_TO_TARGET) && !target_received)
      return;
    // if (!target_received || current_state == MOVING_TO_TARGET)
    //   return;

    double currT = this->now().seconds();
    float deltaT = currT - prevT;
    prevT = currT;

    geometry_msgs::msg::Twist cmd;

    double dx = targetX - currentX;
    double dy = targetY - currentY;

    double odom_robot_yaw = convertation(odom_robot_msg);
    double theta = 0 - odom_robot_yaw;
    theta = atan2(sin(theta), cos(theta));

    double distance = std::sqrt(dx * dx + dy * dy);
    double angle = std::atan2(dy, dx);
    float controlled_distance = omni_distance.control_base(distance, desired_linear_vel);
    float controlled_angle = omni_angular.control_base_rotation(theta, max_angular_vel);

    
    if(!decision_receive)
    {
      cmd.linear.x = 0;
      cmd.linear.y = 0;
      cmd.angular.z = 0;
      RCLCPP_INFO(this->get_logger()," Waiting target receive from gui......");
      RCLCPP_INFO(this->get_logger(), "posisi x %f", currentX);
      RCLCPP_INFO(this->get_logger(), "posisi y %f\n", currentY);
      return;
    }

    switch (current_state)
    {
    case WAITING_FOR_TARGET:
      cmd.linear.x = 0.0;
      cmd.linear.y = 0.0;
      cmd.angular.z = 0.0;

      if (target_received)
      {
        current_state = MOVING_TO_TARGET;
        RCLCPP_INFO(this->get_logger(), " Starting movement to target %d/%d",
                    stage1_target_count + 1, stage1_targets_total);
      }
      break;

    case MOVING_TO_TARGET:
    {
      
      
      if (( tof_valid && distance < TOF_THRESHOLD|| sensor_obstacle || obstacle_lock) && !ignore_obstacle)
      {
        RCLCPP_INFO(this->get_logger(), " Stage 1: BLOCKED by obstacle");
        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        cmd.angular.z = 0.0;
        
        break;
      }

      RCLCPP_INFO(this->get_logger(), "posisi x %f", currentX);
      RCLCPP_INFO(this->get_logger(), "posisi y %f\n", currentY);
      RCLCPP_INFO(this->get_logger(), "theta: %f", theta);
      RCLCPP_INFO(this->get_logger(), "angular pid: %f", controlled_angle);
      RCLCPP_INFO(this->get_logger(), "dx: %f dy: %f", dx, dy);
      // RCLCPP_INFO(this->get_logger(), "posisi y", currentY);


      if (distance > 0.04)
      {
          cmd.linear.x = 0.2 * std::cos(angle);
          cmd.linear.y = controlled_distance * std::sin(angle);
          cmd.angular.z = -controlled_angle;
          RCLCPP_INFO(this->get_logger(), "move robot");

        // }
      }
      else
      {
        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        cmd.angular.z = -controlled_angle;

        RCLCPP_INFO(this->get_logger(), "holding position");

        if (!target_reached_flag && !paused)
        // if (!target_reached_flag && !paused && !just_resumed && target_received && distance < 0.03)
        {

          target_reached_flag = true;


          current_state = TARGET_REACHED;
          stage1_target_count++;

          std_msgs::msg::Bool reached_msg;
          reached_msg.data = true;
          reached_pub->publish(reached_msg);

          RCLCPP_INFO(this->get_logger(), "✅ Target %d/%d reached!", stage1_target_count, stage1_targets_total);

          if (stage1_target_count >= stage1_targets_total)
          {
            stage1_completed = true;
            stage1_complated_for_camera = true;
            camera_active = true;

            stage1_target_count = 0;
            target_received = false;
            RCLCPP_INFO(this->get_logger(), "target reached");
            RCLCPP_INFO(this->get_logger(), "STAGE1 COMPLATED - Moving to Stage 2");
          }
          else
          {

            RCLCPP_INFO(this->get_logger(), "wait for next target");
          }
        }
      }

      break;
    }

    case PAUSED_STAGE1:
    {

      paused = true;

      cmd.linear.x = 0.0;
      cmd.linear.y = 0.0;
      cmd.angular.z = 0.0;
      // cmd.angular.z = -controlled_angle;

      bool obstacle_cleared = !((tof_valid && distance < TOF_THRESHOLD) || sensor_obstacle || obstacle_lock);

      // if (lifter_down2_flag && obstacle_cleared)
      if (!obstacle_cleared && lifter_down2_flag)
      // if (!obstacle_cleared || lifter_down2_flag)
      {

        paused = false;
        lifter_down2_flag = false;

        just_resumed = true;

        current_state = previous_state_before_pause;
        // current_state = MOVING_TO_TARGET;
        target_reached_flag = false;
        RCLCPP_INFO(this->get_logger(), "Resuming from pause");
        // swing_reset_done = false;
        // ignore_obstacle = false;
      }
      else
      {

        RCLCPP_INFO(this->get_logger(), " PAUSED - Waiting for obstacle to clear...");
      }
    }
      break;

      // ================================================STAGE 2====================================================

    case STAGE2_MOVE_STEPbSTEP:
    {
      constexpr double CELL_SIZE = 1.0;

      if (!stage2_initiallized)
      {

        planner.reset();

        stage2_substate = ST2_PLAN;
        stage2_initiallized = true;
        RCLCPP_INFO(this->get_logger(), "STAGE2 STARTED (GRID PLANNER/WAYPOINT MODE)");
      }

      double dx2 = target_Astar_X - currentX;
      double dy2 = target_Astar_Y - currentY;
      double dist = std::sqrt(dx2 * dx2 + dy2 * dy2);
      double current_yaw = convertation(odom_robot_msg);

      // float control_distance2 = omni_distance.control_base(dist, deltaT);

      switch (stage2_substate)
      {

      case ST2_PLAN:
      {
        if (!receive_waypoint_backend)
          break;

        int current_cell = planner.worldToGrid(currentX, currentY);
        int target_cell = planner.worldToGrid(target_Astar_X, target_Astar_Y);

        int next = planner.chooseNextCell(current_cell, target_cell);

        if (next < 0 || next == target_cell)
        {
          stage2_substate = ST2_DONE;
          receive_waypoint_backend = false;
          break;
        }

        planner.last_cell = current_cell;
        planner.next_cell = next;

        stage2_yaw = planner.cellToYaw(current_cell, next);
        stage2_substate = ST2_ROTATE;

        RCLCPP_INFO(this->get_logger(), "[STAGE2][PLAN] current_cell=%d | next_cell=%d | target_cell=%d",
                    current_cell, next, target_cell);

        break;
      }

      case ST2_ROTATE:
      {
        double current_yaw = convertation(odom_robot_msg);
        double yaw_error = tf2NormalizeAngle(stage2_yaw - current_yaw);

        if (std::fabs(yaw_error) > 0.05)
        {
          cmd.linear.x = 0.0;
          cmd.linear.y = 0.0;
          cmd.angular.z = 0.6 * yaw_error;
        }
        else
        {
          cmd.angular.z = 0.0;

          stage2_start_x = currentX;
          stage2_start_y = currentY;

          stage2_substate = ST2_MOVE;
        }

        break;
      }

      case ST2_MOVE:
      {
        // double dist = hypot(currentX - stage2_start_x,currentY - stage2_start_y);
        float control_distance2 = omni_distance.control_base((CELL_SIZE - dist), deltaT);

        if (dist < CELL_SIZE - 0.05)
        {
          cmd.linear.x = control_distance2 * cos(stage2_yaw);
          cmd.linear.y = control_distance2 * sin(stage2_yaw);
          cmd.angular.z = 0.0;
        }
        else
        {
          cmd.linear.x = 0.0;
          cmd.linear.y = 0.0;

          stage2_substate = ST2_PLAN;
        }

        break;
      }

      case ST2_DONE:
      {
        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        cmd.angular.z = 0.0;

        stage2_initiallized = false;
        receive_waypoint_backend = false;
        current_state = TARGET_REACHED;

        RCLCPP_INFO(this->get_logger(), "STAGE2 COMPLETE");
        break;
      }
      }

      break;
    }
    case TARGET_REACHED:
      cmd.linear.x = 0.0;
      cmd.linear.y = 0.0;
      cmd.angular.z = -controlled_angle;

      if (stage1_completed)
      {
        // current_state = STAGE2_MOVE_STEPbSTEP;
        stage1_completed = false; // ✅ RESET FLAG
        // current_block = 1;
        RCLCPP_INFO(this->get_logger(), "STARTING STAGE 2 - Step by Step Movement");
        RCLCPP_INFO(this->get_logger(), "=============================================");
      }

      else if (target_received)
      {

        current_state = MOVING_TO_TARGET;
        RCLCPP_INFO(this->get_logger(), "Starting movement to target %d/%d", stage1_target_count + 1, stage1_targets_total);
      }
      else
      {

        RCLCPP_INFO(this->get_logger(), "waiting to next target........");
      }
      break;
    }
    cmd_pub->publish(cmd);
  }
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr pose_sub;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr boundingBox_sub;
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr tof_sub;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr tof_send_pub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lifter2_sub;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr waypoint_backend_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr infraReceive_sub;
  rclcpp::Subscription<gui_kfs_msgs::msg::KFSDecision>::SharedPtr gui_sub;
  // rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr pose2_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr proxy_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr toStage2_sub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr proxy_true_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reached_pub;
  nav_msgs::msg::Odometry odom_robot_msg;

  rclcpp::TimerBase::SharedPtr timer_;
};

bool Movement::swing_reset_done = false;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Movement>());
  rclcpp::shutdown();
  return 0;
}