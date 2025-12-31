#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <pid.hpp>
#include <convertion.hpp>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#define PWM_MAX 1.0  //1.5
#define PWM_MIN -1.0 //1.5

#define kp 0.8 //1.0
#define ki 0.0
#define kd 0.0

#define kpT 1.5
#define kiT 0.0
#define kdT 0.0

PID omni_distance(PWM_MIN, PWM_MAX, kp, ki, kd);
PID omni_angular(PWM_MIN, PWM_MAX, kpT, kiT, kdT);


class Movement : public rclcpp::Node
{

public:
  Movement() : Node("Movement_Point")
  {

    // cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("/omni_cont/cmd_vel", 10);

    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10,
                                                                  std::bind(&Movement::odom_callback, this, std::placeholders::_1));

    pose_sub = this->create_subscription<geometry_msgs::msg::Point>("/pose", 10,
                                                                  std::bind(&Movement::pose_callback, this, std::placeholders::_1));

    proxy_sub = this->create_subscription<std_msgs::msg::Bool>("proxydata", 10,
                                                                  std::bind(&Movement::proxy_callback, this, std::placeholders::_1));

    proxy2_sub = this->create_subscription<std_msgs::msg::Bool>("proxydata2", 10,
                                                                  std::bind(&Movement::proxy2_callback, this, std::placeholders::_1));



    boundingBox_sub = this->create_subscription<geometry_msgs::msg::Point>("coordinate_boundingBox", 10,
                                                                  std::bind(&Movement::coordinate_callback, this, std::placeholders::_1)); 
                                                                  
                  
    tof_sub = this->create_subscription<std_msgs::msg::UInt16>("tof_distance", 10,
                                                                  std::bind(&Movement::tof_callback, this, std::placeholders::_1));
                                                                  
                                                                  





    // pose2_sub = this->create_subscription<geometry_msgs::msg::Point>("/pose_steps", 10,
    //                                                                  std::bind(&Movement::pose2_callback, this, std::placeholders::_1));

    reached_pub = this->create_publisher<std_msgs::msg::Bool>("/target_reached", 10);

    proxy_true_pub = this->create_publisher<std_msgs::msg::Bool>("/true_sensor", 10);

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
    PAUSED_STAGE1,
    STAGE2_MOVE_STEPbSTEP
  };

  enum Stage2state
  {
    MOVE_FORWARD,
    ROTATE,
    SEARCH_TARGET,
    ALIGN_BY_YOLO,
    APPROACH_TARGET,
    COMPLETE
  };

  int stage1_target_count = 0;
  int stage1_targets_total = 2;
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



  bool stage2_initiallized = false;
  bool rotating = false;
  double stage2_posX = 0;
  double stage2_posY = 0;
  double stage2_yaw = 0;
  int current_block = 1;
  Stage2state stage2_substate = MOVE_FORWARD;
  State previous_state_before_pause;
  double rotation_direction = 1;
  double rotate_target_angle = 0.0;
  bool rotation_completed = false;
  bool yolo_valid = false;
  float yolo_errorX = 0.0;
  float yolo_depth = -1.0;

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
      targetX = msg->x;
      targetY = msg->y;
      target_received = true;
      current_state = MOVING_TO_TARGET;

      RCLCPP_INFO(this->get_logger(), "New target STAGE1 received: (%.2f, %.2f)", targetX, targetY);
      RCLCPP_INFO(this->get_logger(), "Current position: (%.2f, %.2f)", currentX, currentY);
    }
  }

  void proxy_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {

      bool detected = msg->data;
      // proxy_true_pub->publish(msg);


      RCLCPP_INFO(this->get_logger(), "SENSOR RAW: %s", detected ? "TRUE" : "FALSE");

      sensor_obstacle = !detected;

      RCLCPP_INFO(this->get_logger(), "OBSTACLE FLAG: %s", sensor_obstacle ? "STOP" : "GO");
      
      std_msgs::msg::Bool out_msg;
      out_msg.data = msg->data;
      proxy_true_pub->publish(out_msg);
      
      if(sensor_obstacle){
        

        if(current_state == MOVING_TO_TARGET){
          previous_state_before_pause = current_state;
          current_state = PAUSED_STAGE1;

          if(previous_state_before_pause == MOVING_TO_TARGET){
            RCLCPP_INFO(this->get_logger(), "Stage 1 PAUSED");
          }
        }
      }

    was_obstacle = sensor_obstacle;
  }

  void proxy2_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    bool detected2 = msg->data;

    RCLCPP_INFO(this->get_logger(), "SENSOR2 RAW: %s", detected2 ? "TRUE" : "FALSE");

    sensor_obstacle2 = !detected2;

    RCLCPP_INFO(this->get_logger(), "OBSTACLE FLAG: %s", sensor_obstacle2 ? "NOT GRIPP" : "GRIPP");

    std_msgs::msg::Bool out_msg;
    out_msg.data = msg->data;
    proxy_true_pub->publish(out_msg);
    
  }

  double tof_distance = 0;

  void tof_callback(const std_msgs::msg::UInt16::SharedPtr msg)
  {
    uint16_t distance = msg->data;

    tof_distance = distance;
    tof_valid = true;

    RCLCPP_DEBUG(this->get_logger(),"ToF distance: %u mm", distance);

  }

  void coordinate_callback(const geometry_msgs::msg::Point::SharedPtr msg)
  {

    if(!camera_active) return;
    yolo_errorX = msg->x;
    yolo_depth = msg->y;
    float target = msg->z;

    yolo_valid = (yolo_depth > 0.0);

    // if(!camera_active) {

    //   static bool first_time = true;
    //   if(first_time){
    //     RCLCPP_INFO(this->get_logger(),  "Camera data received but NOT ACTIVE (waiting Stage 1)");

    //     first_time = false;
    //   }

    //   return;

    // }
    // float errorX = msg->x;
    // float depth =  msg->y;
    // float target = msg->z;

    // if(depth < 0)
    // {
    //   RCLCPP_WARN(this->get_logger(), "NO VALID TARGET FROM CAMERA");
    //   return;
    // }

    // RCLCPP_INFO(this->get_logger(), "📷 Camera: errorX=%.3f, depth=%.3f, target=%.3f", errorX, depth, target);

  }

  void control_loop()
  {
    if (!start_received)
      return;

    if ((current_state == WAITING_FOR_TARGET || current_state == MOVING_TO_TARGET) && !target_received)
      return;

    double currT = this->now().seconds();
    float deltaT = currT - prevT;
    prevT = currT;

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
        RCLCPP_INFO(this->get_logger(), " Starting movement to target %d/%d",
                    stage1_target_count + 1, stage1_targets_total);
      }
      break;

    case MOVING_TO_TARGET:
    {

      if(sensor_obstacle){
        RCLCPP_INFO(this->get_logger(), " Stage 1: BLOCKED by obstacle");
        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        cmd.angular.z = 0.0;
        break;

      }


      double dx = targetX - currentX;
      double dy = targetY - currentY;

      // double yaw = convertation(odom_robot_msg);
      // double theta = 0 - yaw;

      double odom_robot_yaw = convertation(odom_robot_msg);
      double theta = 0 - odom_robot_yaw;
      double distance = std::sqrt(dx * dx + dy * dy);
      double angle = std::atan2(dy, dx);

      float control_distance = omni_distance.control_base(distance, deltaT);
      float control_angle = omni_angular.control_base_rotation(theta, deltaT);

      if (distance > 0.03)
      {
        float speed1 = 0.2;

        // float kp_distance = 0.5;  // gain, adjust sesuai kebutuhan
        // speed1 = kp_distance * distance;
        // speed1 = std::min(speed1, 0.3f);  // Limit max speed
        // speed1 = std::max(speed1, 0.05f); // Minimum speed

        float vx = std::cos(angle);
        float vy = std::sin(angle);
        float magnitude = std::sqrt(vx*vx + vy*vy);
        if(magnitude > 0.001){

          vx = vx / magnitude;
          vy = vy / magnitude;
        }

        cmd.linear.x = speed1 *  vx;
        cmd.linear.y = speed1 *  vy;
        // cmd.linear.x = control_distance *  std::cos(angle);
        // cmd.linear.y = control_distance *  std::sin(angle);
        cmd.angular.z = 0.0;
        // cmd.angular.z = control_angle;
        RCLCPP_INFO(this->get_logger(), "move robot");
      }
      else
      {
        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        cmd.angular.z = 0.0;

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
      break;
    }

    case PAUSED_STAGE1:
      cmd.linear.x = 0.0;
      cmd.linear.y = 0.0;
      cmd.angular.z = 0.0;

      if(!sensor_obstacle){

        current_state = previous_state_before_pause;
        RCLCPP_INFO(this->get_logger(), "Resuming from pause");

      }else{

        RCLCPP_INFO(this->get_logger(), " PAUSED - Waiting for obstacle to clear...");
      }
      break;

    case STAGE2_MOVE_STEPbSTEP:
    {

      const double BLOCK_DISTANCE_METER = 1.2;

      if (!stage2_initiallized)
      {
        stage2_posX = currentX;
        stage2_posY = currentY;
        stage2_yaw = convertation(odom_robot_msg);
        
        stage2_substate = MOVE_FORWARD;
        rotation_direction = -1;
        rotation_completed = false;
        stage2_initiallized = true;

        RCLCPP_INFO(this->get_logger(), "==========================================");
        RCLCPP_INFO(this->get_logger(), "START: 2 Blocks with Turn in Middle");
        RCLCPP_INFO(this->get_logger(), "Block 1: Move 120cm → Turn → Block 2: Move 120cm");
        RCLCPP_INFO(this->get_logger(), "Turn direction: %s",
                    (rotation_direction == 1) ? "RIGHT" : "LEFT");
      }

      double current_yaw = convertation(odom_robot_msg);

      switch (stage2_substate)
      {
      case MOVE_FORWARD:
      {
        double stage2_targetX = stage2_posX + BLOCK_DISTANCE_METER * std::cos(stage2_yaw);
        double stage2_targetY = stage2_posY + BLOCK_DISTANCE_METER * std::sin(stage2_yaw);

        double stage2_dx = stage2_targetX - currentX;
        double stage2_dy = stage2_targetY - currentY;
        double stage2_distance_error = std::sqrt(stage2_dx * stage2_dx + stage2_dy * stage2_dy);

        double stage2_angle = std::atan2(stage2_dy, stage2_dx);

        double real_traveled = std::sqrt(
            (currentX - stage2_posX) * (currentX - stage2_posX) +
            (currentY - stage2_posY) * (currentY - stage2_posY));

        // double remaining_distance = std::max(0.0, BLOCK_DISTANCE_METER - real_traveled);
          
        RCLCPP_INFO(this->get_logger(), "Block %d: %.3f/1.20m, Yaw=%.1f°",
                current_block, real_traveled, stage2_yaw * 180/M_PI);

        RCLCPP_INFO(this->get_logger(), "Block %d: %.3f/1.20m (Error: %.3fm)",
                    current_block, real_traveled, stage2_distance_error);

        if (real_traveled < 1.17)
        // if (stage2_distance_error > 0.03)
        {
          // float control_distance = omni_distance.control_base(remaining_distance, deltaT);
          // float control_distance = omni_distance.control_base(stage2_distance_error, deltaT);

          double speed = 0.50;

          // cmd.linear.x = control_distance * std::cos(stage2_angle);
          // cmd.linear.y = control_distance * std::sin(stage2_angle);
          cmd.linear.x = speed * std::cos(stage2_yaw);
          cmd.linear.y = speed * std::sin(stage2_yaw);
          cmd.angular.z = 0.0;
        }
        else
        {
          
          cmd.linear.x = 0.0;
          cmd.linear.y = 0.0;
          cmd.angular.z = 0.0;

          if (current_block == 1)
          {
            if (rotation_direction == 1)
            {

              rotate_target_angle = stage2_yaw - (M_PI / 2);
            }
            else
            {

              rotate_target_angle = stage2_yaw + (M_PI / 2);
            }

            while (rotate_target_angle > M_PI)
              rotate_target_angle -= 2 * M_PI;
            while (rotate_target_angle < -M_PI)
              rotate_target_angle += 2 * M_PI;

            stage2_substate = ROTATE;
            rotation_completed = false;

            RCLCPP_INFO(this->get_logger(), "✅ Block 1 DONE! Turning %s 90°...",
                        (rotation_direction == 1) ? "RIGHT" : "LEFT");
          }
          else if (current_block == 2)
          {
            stage2_substate = COMPLETE;
            RCLCPP_INFO(this->get_logger(), "✅ Block 2 DONE! Mission complete.");
          }
        }

        break;
      }

      case ROTATE:
      {
        double angle_error = rotate_target_angle - current_yaw;

        while (angle_error > M_PI) angle_error -= 2 * M_PI;
        while (angle_error < -M_PI)angle_error += 2 * M_PI;

         RCLCPP_INFO(this->get_logger(), "Turning: %.1f° error (%.3f rad)",angle_error * 180 / M_PI, angle_error);

        if (std::fabs(angle_error) > 0.03)
        {
          // float control_angle = omni_angular.control_base_rotation(angle_error, deltaT);
          double speed_angle = 0.25;

          cmd.linear.x = 0.0;
          cmd.linear.y = 0.0;
          cmd.angular.z = speed_angle;
          // cmd.angular.z = control_angle;
        }
        else
        {
          cmd.linear.x = 0.0;
          cmd.linear.y = 0.0;
          cmd.angular.z = 0.0;

          stage2_yaw = rotate_target_angle;

          stage2_posX = currentX;
          stage2_posY = currentY;
          stage2_substate = MOVE_FORWARD;
          current_block = 2;

          RCLCPP_INFO(this->get_logger(), "Turn complete! Ready for Block 2 (120cm)...");
        }

        break;
      }

      case COMPLETE:
      {
        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        cmd.angular.z = 0.0;

        stage2_initiallized = false;
        stage2_substate = MOVE_FORWARD;
        current_state = TARGET_REACHED;
        current_block = 1;

        RCLCPP_INFO(this->get_logger(), "==========================================");
        RCLCPP_INFO(this->get_logger(), "MISSION COMPLETE!");
        RCLCPP_INFO(this->get_logger(), "Pattern: Forward → Turn → Forward");
        RCLCPP_INFO(this->get_logger(), "Total: 2 blocks × 1.20m = 2.40m");
        RCLCPP_INFO(this->get_logger(), "==========================================");
        break;
      }
      }

      break;
    }
    case TARGET_REACHED:
      cmd.linear.x = 0.0;
      cmd.linear.y = 0.0;
      cmd.angular.z = 0.0;

      if (stage1_completed)
      {
        // current_state = STAGE2_MOVE_STEPbSTEP;
        stage1_completed = false; // ✅ RESET FLAG
        current_block = 1;
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
    // cmd_pub->publish(cmd);
  }
  // rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr pose_sub;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr boundingBox_sub;
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr tof_sub;
  // rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr pose2_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr proxy_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr proxy2_sub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr proxy_true_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reached_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr next_step_pub;
  nav_msgs::msg::Odometry odom_robot_msg;
  // nav_msgs::msg::Odometry odom_robot_step;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Movement>());
  rclcpp::shutdown();
  return 0;
}