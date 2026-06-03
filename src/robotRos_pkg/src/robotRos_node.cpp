#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include "gui_kfs_msgs/msg/kfs_decision.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>
#include <pid.hpp>
#include <convertion.hpp>
#include <cmath>
#include "grid_planner.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

GridPlanner planner;
PID omni_distance;
PID omni_angular;

class Movement : public rclcpp::Node
{
  
  public:
  Movement() : Node("Movement_Point")
  {
    auto qos = rclcpp::QoS(10).transient_local();

    cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("/omni_cont/cmd_vel", 10);

    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10,
                                                                  std::bind(&Movement::odom_callback, this, std::placeholders::_1));

    pose_sub = this->create_subscription<geometry_msgs::msg::Point>("/pose", 10,
                                                                    std::bind(&Movement::pose_callback, this, std::placeholders::_1));

    proxy_sub = this->create_subscription<std_msgs::msg::Bool>("/proxydata", 10,
                                                               std::bind(&Movement::proxy_callback, this, std::placeholders::_1));

    waypoint_backend_sub = this->create_subscription<geometry_msgs::msg::Point>("/planner/waypoint", 10,
                                                                                std::bind(&Movement::waypoint_backend_callback, this, std::placeholders::_1));

    gui_sub = this->create_subscription<std_msgs::msg::Bool>("/kfs_decision", qos,
                                                                        std::bind(&Movement::gui_callback, this, std::placeholders::_1));

    buttonStage3_sub = this->create_subscription<std_msgs::msg::Bool>("/button_stage3", 10,
                                    std::bind(&Movement::toStage3_callback, this, std::placeholders::_1));

    after_climb_sub = this->create_subscription<std_msgs::msg::Bool>("afterClimb", 10,
                                                                     std::bind(&Movement::after_climb_callback, this, std::placeholders::_1));

    // boundingBox_sub = this->create_subscription<geometry_msgs::msg::Point>("coordinate_boundingBox", 10,
    //                                                                        std::bind(&Movement::coordinate_callback, this, std::placeholders::_1));

    limit_stage2_sub = this->create_subscription<std_msgs::msg::Bool>("limitdata", 10,
                                  std::bind(&Movement::limit_stage2_callback, this, std::placeholders::_1));

    tof_sub = this->create_subscription<std_msgs::msg::UInt16>("tof_distance", 10,
                                std::bind(&Movement::tof_callback, this, std::placeholders::_1));

    tof_send_pub = this->create_publisher<std_msgs::msg::UInt16>("tof_send_back", 10);

    lifter2_sub = this->create_subscription<std_msgs::msg::Bool>("/lifter_down2", 10,
                                std::bind(&Movement::lifter2_callback, this, std::placeholders::_1));

    reached_pub = this->create_publisher<std_msgs::msg::Bool>("/target_reached", 10);

    infraReceive_sub = this->create_subscription<std_msgs::msg::Bool>("infraReceive", 10,
                                std::bind(&Movement::infra_callback, this, std::placeholders::_1));

    reached_Astar_sub = this->create_subscription<std_msgs::msg::Bool>("/target_Astar_Done", 10,
                                std::bind(&Movement::reached_Astar_callback, this, std::placeholders::_1));

    proxy_true_pub = this->create_publisher<std_msgs::msg::Bool>("/true_sensor_proxy", 10);

    wp_reached2_pub = this->create_publisher<std_msgs::msg::Bool>("/meihua/wp_reached", 10);

    climb_done_pub = this->create_publisher<std_msgs::msg::Bool>("/meihua/r2_arrived", 10);

    wait_lifter_sub = this->create_subscription<std_msgs::msg::Bool>("/wait_lifter", 10,
                              std::bind(&Movement::wait_lifter_callback, this, std::placeholders::_1));

    wp_done_pub = this->create_publisher<std_msgs::msg::Bool>("/wp_done", 10);

    path_list_sub = this->create_subscription<std_msgs::msg::String>("/meihua/path_steps", 10,
                              std::bind(&Movement::path_list_callback, this, std::placeholders::_1));

    next_step_sub = this->create_subscription<std_msgs::msg::String>("/meihua/next_step", 10,
                              std::bind(&Movement::next_step_callback, this, std::placeholders::_1));

    descend_pub = this->create_publisher<std_msgs::msg::Bool>("/start_descend", 10);


    descend_lifter_up_sub = this->create_subscription<std_msgs::msg::Bool>("/descend_lifter_up_after_down", 10,
                              std::bind(&Movement::descend_lifter_up_callback, this, std::placeholders::_1));

    checking_input_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>("/checking_input", 10,
                              std::bind(&Movement::checking_input_callback, this, std::placeholders::_1));

    allow_lifter_up_pub = this->create_publisher<std_msgs::msg::Bool>("/allow_lifter_up", 10);

    stage2_exit_pub = this->create_publisher<geometry_msgs::msg::Point>("/stage2_exit_pos", 10);

    // tambah subscriber
    // ir_code_sub = this->create_subscription<std_msgs::msg::UInt32>("ir_raw_code", 10,
    // [this](const std_msgs::msg::UInt32::SharedPtr msg) {
    //     RCLCPP_INFO(this->get_logger(), "IR RAW CODE: 0x%08X", msg->data);
    // });



    timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&Movement::control_loop, this));


    start_received = false;
    target_received = false;
    current_state = WAITING_FOR_TARGET;
    prevT = this->now().seconds();

    float kp, ki, kd;
    float kpT, kiT, kdT;

    this->declare_parameter("kp", 0.75);
    this->declare_parameter("ki", 0.0);
    this->declare_parameter("kd", 0.0);

    // this->declare_parameter("kp", 1.0);
    // this->declare_parameter("ki", 0.0);
    // this->declare_parameter("kd", 0.0);

    this->declare_parameter("kpT", 1.2);
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
  enum State
  {
    WAITING_FOR_TARGET,
    MOVING_TO_TARGET,
    TARGET_REACHED,
    PAUSED_STAGE1,
    STRAIGHT_FOR_CLIMB,
    AFTER_CLIMB,
    STOP_SENSOR,
    POST_CLIMB_MOVE,
    STAGE2_MOVE_STEPbSTEP,
    MOVING_TO_TARGET_STAGE3,
    WAIT_LIFTER,
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

  // enum Stage2Substate

  enum Stage2SubState
  {
    ST2_IDLE,
    // ST2_HOLD,
    ST2_STRAIGHT_CLIMB,
    ST2_AFTER_CLIMB,
    ST2_ROTATE,
    ST2_STOP_SENSOR,
    ST2_POST_CLIMB,
    ST2_WAIT_LIFTER,
    ST2_WAIT_LIFTER_UP_CONFIRM,
    ST2_REVERSE_AFTER_LIFTER,
    ST2_DONE,
    ST2_EXIT_ROTATE
  };

  float desired_linear_vel, max_angular_vel;

  int stage1_target_count = 0;
  int stage1_targets_total = 4;
  bool stage1_completed = false;

  bool camera_active = false;
  bool stage1_complated_for_camera = false;

  double startX, startY;
  double targetX, targetY;
  double currentX, currentY;
  double start_x_after_climb;
  double start_y_after_climb;
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
  // bool just_resumed = false;
  bool start_saved = false;
  double stop_time = 0;
  bool stop_initialize = false;
  double heading = 0.0;
  bool waiting_lifter = false;

  bool stage2_initiallized = false;
  bool stage2_triggred = false;
  double target_Astar_X, target_Astar_Y;
  bool receive_waypoint_backend = false;
  bool rotating = false;
  double stage2_start_y = 0;
  double stage2_start_x = 0;
  double stage2_yaw = 0;
  // int current_block = 1;
  // Stage2Substate stage2_substate = ST2_MOVE;
  // Stage2state stage2_substate = MOVE_FORWARD;
  State previous_state_before_pause;
  double rotation_direction = 1;
  double rotate_target_angle = 0.0;
  bool rotation_completed = false;
  bool yolo_valid = false;
  float yolo_errorX = 0.0;
  float yolo_depth = -1.0;
  float target = 0.0;
  double wp_x, wp_y = 0;
  bool has_wp = false;
  bool need_climb = false;
  double currentYaw = 0.0;
  double heading_offset = 0.0;
  double last_angle_error = 0.0;
  bool stage2_entry_locked = false;
  double targetX_world = 0.0;
  double targetY_world = 0.0;
  double start_x_st2 = 0.0;
  double start_y_st2 = 0.0;
  bool climb_finished = false;
  bool is_stage2 = false;
  bool wp_processing = false;
  bool limit_triggered = false;
  bool wp_done_sent = false;
  double odom_offset_x = 0.0;
  double odom_offset_y = 0.0;

  bool stage3_mode = false;
  bool stage1_climb_done = false;

  Stage2SubState st2_state = ST2_IDLE;

  // double st2_timer_start = 0.0;
  // int st2_target_dir = 0;                             // 0=kanan,1=atas,2=kiri,3=bawah
  // int st2_current_grid_x = 1, st2_current_grid_y = 0; // posisi awal setelah climb
  // double st2_short_start_x, st2_short_start_y;
  // bool st2_short_odom_init = false;
  // double st2_target_yaw_rad;

  int st2_current_grid_x, st2_current_grid_y;
  int st2_current_dir; // 0=kanan, 1=atas, 2=kiri, 3=bawah
  int st2_target_dir;
  double st2_rot_duration;
  double st2_rot_start_time;
  double st2_timer_start;
  bool st2_short_odom_init;
  double st2_short_start_x, st2_short_start_y;

  // ── Stage 2 rotate & reverse ─────────────────────────────────────────────
  bool need_rotate = false;
  bool need_reverse = false;
  double rotate_target_yaw = 0.0;
  bool proxy_currently_detected = false;
  bool proxy_stop_triggered = false;
  double yaw_error = 0.0;
  int current_grid_height = 200;
  bool descend_published = false;
  bool lifter_up_confirmed = false;
  bool waiting_for_lifter_up_confirm = false;
  bool proxy_was_seen = false;
  bool has_pending_step = false;
  int pending_row = -1, pending_col = -1, pending_height = -1;
  std::string pending_dir = "";
  double st2_reverse_start_time = 0.0;
  int pitch_stable_count = 0;
  bool is_exit_step = false;
  int exit_rotate_stable_count = 0;
  bool climbing_stage3 = false;
  double wait_lifter_start_time = 0.0;
  double wait_lifter_st2_start_time = 0.0;
  bool wait_lifter_timer_init = false;
  bool wait_lifter_st2_timer_init = false;

  // ── Fall detection saat naik (ST2_STRAIGHT_CLIMB) ────────────────────────

  // float pitch_prev = 0.0;
  // bool fall_detected = false;
  // int fall_recover_phase = 0;
  // double fall_reverse_start_time = 0.0;
  // static constexpr float FALL_PITCH_DELTA_THRESHOLD = 5.0f;
  // static constexpr double FALL_REVERSE_DURATION = 1.2;

  double normalize_angle(double angle)
  {
    while (angle > M_PI)
      angle -= 2 * M_PI;
    while (angle < -M_PI)
      angle += 2 * M_PI;
    return angle;
  }

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

    heading = yaw;

    odom_robot_msg = *msg;

    currentX = msg->pose.pose.position.x + odom_offset_x;
    currentY = msg->pose.pose.position.y + odom_offset_y;
    if (!start_received)
    {
      startX = currentX;
      startY = currentY;
      start_received = true;
    }
  }

  void pose_callback(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    // if (current_state != WAITING_FOR_TARGET)
    //   return;

    if (current_state == STRAIGHT_FOR_CLIMB ||
        current_state == STAGE2_MOVE_STEPbSTEP)
    {
      RCLCPP_WARN(this->get_logger(), "pose_callback IGNORED — robot sedang stage2/climb");
      return;
    }

    targetX = msg->x;
    targetY = msg->y;
    target_received = true;
    target_reached_flag = false;

    if(current_state == MOVING_TO_TARGET_STAGE3)
    {
      RCLCPP_INFO(this->get_logger(), "STAGE3: target updated (%.2f, %.2f)", targetX, targetY);
      return;
    }

    RCLCPP_INFO(this->get_logger(), "New target STAGE1 received: (%.2f, %.2f)", targetX, targetY);
    RCLCPP_INFO(this->get_logger(), "Current position: (%.2f, %.2f)", currentX, currentY);
  }

  bool last_detected = false;
  bool obstacle_lock = false;

  void proxy_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {

    bool detected = msg->data;

    if (detected != last_detected)
    {
      RCLCPP_INFO(this->get_logger(), "SENSOR RAW: %s", detected ? "TRUE" : "FALSE");
    }

    last_detected = detected;
    proxy_currently_detected = detected;
  }

  double tof_distance = 0;
#define TOF_THRESHOLD 100

  void tof_callback(const std_msgs::msg::UInt16::SharedPtr msg)
  {
    uint16_t distance = msg->data;

    if (distance > 0)
    {
      tof_distance = distance;
      tof_valid = true;
      RCLCPP_DEBUG(this->get_logger(), "ToF distance: %u mm", distance);

      std_msgs::msg::UInt16 send_back_msg;
      send_back_msg.data = msg->data;
      tof_send_pub->publish(send_back_msg);
    }

    if (distance > 0 && distance < TOF_THRESHOLD && !ignore_obstacle)
    {
      if (current_state == MOVING_TO_TARGET)
      {
        previous_state_before_pause = current_state;
        current_state = PAUSED_STAGE1;

        RCLCPP_INFO(this->get_logger(), "stage 1 PAUSED by Tof");
      }
    }
  }

  void infra_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data)
    {
      RCLCPP_INFO(this->get_logger(), "IR DETECTED");
    }
  }

  float hardware_pitch = 0.0;
  float lifter_pos_snapshot = 0.0;
  float hardware_lifter_pos = 0.0;

  void checking_input_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() > 6)
    {
      hardware_pitch = msg->data[5];
      hardware_lifter_pos = msg->data[6];

      // RCLCPP_INFO(this->get_logger()," pos lifter", msg->data[6]);

      RCLCPP_INFO(this->get_logger(),
            "Pos Lifter: %.2f",
            // msg->data[0],  // rps1
            // msg->data[1],  // rps2
            // msg->data[2],  // rps3
            // msg->data[3], // rps4
            // msg->data[5],  // pitch
            msg->data[6]); // pos lifter
    }
  }

  bool lifter_down2_flag = false;

  void lifter2_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data)
    {
      lifter_down2_flag = true;
      ignore_obstacle = true;
      RCLCPP_INFO(this->get_logger(), "LIFTER DOWN 2 DETECTED");
    }
  }

  bool decision_receive = false;

  void gui_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    decision_receive = true;
    RCLCPP_INFO(this->get_logger(), "Decision received -> robot boleh jalan");
  }


  bool button3 = false; 
  
  void toStage3_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {

    bool button3 = true;
    
    RCLCPP_INFO(this->get_logger(), "button stage 3 received -> robot lanjut jalan di stage3");
  }

  void waypoint_backend_callback(const geometry_msgs::msg::Point::SharedPtr msg)
  {

    // wp_x = msg->x;
    // wp_y = msg->y;

    // double CELL_SIZE = 1.2;

    // targetX_world = (wp_x * CELL_SIZE) + (CELL_SIZE / 2.0);
    // targetY_world = (wp_y * CELL_SIZE) + (CELL_SIZE / 2.0);

    // need_climb = (msg->z == 1.0);

    // has_wp = true;
    // wp_done_sent = false;
    // goal_done = false;
    // climb_finished = false;
    // wp_processing = false;

    // st2_state = ST2_IDLE;

    // RCLCPP_INFO(this->get_logger(), "New WP: %.2f %.2f", wp_x, wp_y);

    // RCLCPP_INFO(this->get_logger(), "New WP grid(%.0f,%.0f) need_climb=%d", wp_x, wp_y, (int)need_climb);

    // RCLCPP_INFO(this->get_logger(), "Target World: %.2f %.2f", targetX_world, targetY_world);
    // // RCLCPP_INFO(this->get_logger(), "New WP grid(%.0f,%.0f) world(%.2f,%.2f)",
    //             msg->x, msg->y, targetX_world, targetY_world);
  }

  bool limit_active = false;

  void limit_stage2_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {

    if (!msg->data)
      return;

    // ================= STAGE 2 =================
    if (is_stage2)
    {
      // hanya trigger saat lagi jalan
      if ((st2_state == ST2_STRAIGHT_CLIMB && !need_reverse) ||
          st2_state == ST2_AFTER_CLIMB)
      {
        st2_state = ST2_STOP_SENSOR;
      }
    }

    // ================= STAGE 1 =================
    else
    {
      if (current_state == STRAIGHT_FOR_CLIMB ||
          current_state == AFTER_CLIMB)
      {
        current_state = STOP_SENSOR;
      }
    }
  }

  void after_climb_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {

    if (!msg->data)
      return;

    // ===== STAGE 1 =====
    // if (current_state == STRAIGHT_FOR_CLIMB)
    // {
    //   current_state = AFTER_CLIMB;
    // }

    // if(current_state == WAIT_LIFTER && ST2_STRAIGHT_CLIMB)
    // {
    //   st2_state = ST2_AFTER_CLIMB;
    // }

    if (!is_stage2)
    {
      if (current_state == WAIT_LIFTER)
      {
        current_state = AFTER_CLIMB;
      }
    }
    else
    {
      // ===== STAGE 2 =====
      if (st2_state == ST2_WAIT_LIFTER)
      {
        st2_state = ST2_AFTER_CLIMB;
      }
    }
  }

  void wait_lifter_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {

    // if (msg->data)
    // {
    //   current_state = WAIT_LIFTER;
    // }

    // if (!msg->data)
    //   return;

    if (msg->data)
    {
      if (!is_stage2)
      {
        current_state = WAIT_LIFTER;
      }
      else
      {
        if (st2_state == ST2_REVERSE_AFTER_LIFTER || st2_state == ST2_WAIT_LIFTER_UP_CONFIRM || st2_state == ST2_WAIT_LIFTER)
        {
          RCLCPP_WARN(this->get_logger(), "wait_lifter ignored, sedang REVERSE/WAIT_CONFIRM");
          return;
        }

        if (need_reverse && !proxy_was_seen && !proxy_stop_triggered)
        {
          RCLCPP_WARN(this->get_logger(), "wait_lifter ignored: proxy belum terdeteksi");
          return;
        }

        st2_state = ST2_WAIT_LIFTER;
        RCLCPP_INFO(this->get_logger(), "PROXY → ST2_WAIT_LIFTER (mundur)");

        // RCLCPP_WARN(this->get_logger(), "wait_lifter ignored, state=%d need_reverse=%d", (int)st2_state, need_reverse);
        // return;

        // RCLCPP_INFO(this->get_logger(), "PROXY → ST2_WAIT_LIFTER");
      }
    }
  }

  void reached_Astar_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {


  }
  bool goal_done = false;

  void path_list_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    std::string path_steps = msg->data;
    RCLCPP_INFO(this->get_logger(), "Received A* path steps: %s", path_steps.c_str());
  }

  std::string latest_next_step;
  bool goal_active = false;

  void next_step_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    latest_next_step = msg->data;

    RCLCPP_INFO(this->get_logger(), "Received next step: %s", latest_next_step.c_str());

    // update_target_from_step(latest_next_step);

    // std::string dir = parse_direction_from_json(latest_next_step);

    int row = parse_int_from_json(msg->data, "row");
    int col = parse_int_from_json(msg->data, "col");
    int height = parse_int_from_json(msg->data, "height");

    std::string dir = parse_direction_from_json(msg->data);

    if (dir == "DOWN" && row >= 4)
      is_exit_step = true;
    else
      is_exit_step = false;

    if (!dir.empty() && row >= 0 && col >= 0)
    {

      if (st2_state == ST2_REVERSE_AFTER_LIFTER ||
          st2_state == ST2_WAIT_LIFTER ||
          st2_state == ST2_WAIT_LIFTER_UP_CONFIRM ||
          st2_state == ST2_ROTATE ||
          st2_state == ST2_STRAIGHT_CLIMB)
      {
        pending_row = row;
        pending_col = col;
        pending_height = height;
        pending_dir = dir;
        has_pending_step = true;
        RCLCPP_WARN(this->get_logger(), "next_step ignored, sedang proses mundur/lifter");
        return;
      }

      update_target_from_step(row, col, height, dir);
      // update_target_from_step(dir);
      has_wp = true;
      goal_active = true;
      wp_processing = false;
      wp_done_sent = false;
      climb_finished = false;
      goal_done = false;
      descend_published = false;
      waiting_lifter = false;
      proxy_was_seen = false;
      proxy_stop_triggered = false;
      st2_state = ST2_IDLE;

      RCLCPP_INFO(this->get_logger(), "Parsed direction: %s → target set", dir.c_str());
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "gagal parse direction dari JSON: %s", latest_next_step.c_str());
    }
  }

  std::string parse_direction_from_json(const std::string &json_str)
  {

    // std::string dir = "";

    auto extract = [&](const std::string &key) -> std::string
    {
      size_t pos = json_str.find(key);
      if (pos == std::string::npos)
        return "";
      pos += key.size();
      // skip spasi
      while (pos < json_str.size() && json_str[pos] == ' ')
        pos++;
      // skip quote pembuka
      if (pos < json_str.size() && json_str[pos] == '"')
        pos++;
      size_t end = json_str.find('"', pos);
      if (end == std::string::npos)
        return "";
      return json_str.substr(pos, end - pos);
    };

    std::string dir = extract("\"direction\":"); // coba "direction":
    if (dir.empty())
      dir = extract("\"direction\" :"); // coba "direction" :

    if (dir == "SOUTH")
      return "DOWN";
    if (dir == "NORTH")
      return "UP";
    if (dir == "EAST")
      return "RIGHT";
    if (dir == "WEST")
      return "LEFT";
    if (dir == "START" || dir.empty())
      return "";

    return dir;
  }

  int parse_int_from_json(const std::string &json_str, const std::string &field)
  {
    // Coba dengan spasi: "row": 1
    std::string key1 = "\"" + field + "\": ";
    // Coba tanpa spasi: "row":1
    std::string key2 = "\"" + field + "\":";

    size_t pos = json_str.find(key1);
    if (pos == std::string::npos)
      pos = json_str.find(key2);
    if (pos == std::string::npos)
      return -1;

    // Cari angkanya
    pos = json_str.find_first_of("0123456789", pos);
    if (pos == std::string::npos)
      return -1;

    return std::stoi(json_str.substr(pos));
  }

  int current_grid_row = 0;
  int current_grid_col = 1;

  const double CELL_SIZE = 1.2;
  const double GRID_ORIGIN_X = 1.35;
  const double GRID_ORIGIN_Y = -1.25;

  void update_target_from_step(int target_row, int target_col, int target_height, const std::string &dir)
  {
    // const double CELL_SIZE = 1.2; // meter per grid

    // if (dir == "RIGHT")      { targetX_world = currentX + CELL_SIZE; targetY_world = currentY; }
    // else if (dir == "LEFT")  { targetX_world = currentX - CELL_SIZE; targetY_world = currentY; }
    // else if (dir == "UP")    { targetX_world = currentX;             targetY_world = currentY + CELL_SIZE; }
    // else if (dir == "DOWN")  { targetX_world = currentX;             targetY_world = currentY - CELL_SIZE; }
    // else
    // {
    //     RCLCPP_WARN(this->get_logger(), "Unknown step received: %s", dir.c_str());
    //     return;
    // }
    // RCLCPP_INFO(this->get_logger(), "Target set → (%.2f, %.2f)", targetX_world, targetY_world);

    targetY_world = -(GRID_ORIGIN_Y + (target_col * CELL_SIZE));
    targetX_world = GRID_ORIGIN_X + (target_row * CELL_SIZE);

    int height_diff = target_height - current_grid_height;

    if (height_diff < 0)
    {
      need_reverse = true;
      need_rotate = true;

      if (dir == "DOWN")
        rotate_target_yaw = M_PI;
      else if (dir == "UP")
        rotate_target_yaw = 0.0;
      else if (dir == "RIGHT")
        rotate_target_yaw = -M_PI / 2;
      else if (dir == "LEFT")
        rotate_target_yaw = M_PI / 2;

      RCLCPP_INFO(this->get_logger(), "TURUN: rotate belakang ke target, mundur. yaw=%.2f", rotate_target_yaw);
    }
    else if (dir == "DOWN")
    {

      need_reverse = false;
      need_rotate = false;

      RCLCPP_INFO(this->get_logger(), "NAIK SOUTH: langsung maju");
    }

    else
    {
      need_reverse = false;
      need_rotate = true;

      if (dir == "UP")
        rotate_target_yaw = M_PI;
      else if (dir == "RIGHT")
        rotate_target_yaw = M_PI / 2;
      else if (dir == "LEFT")
        rotate_target_yaw = -M_PI / 2;

      RCLCPP_INFO(this->get_logger(), "NAIK %s: rotate depan ke target. yaw=%.2f", dir.c_str(), rotate_target_yaw);
    }

    current_grid_row = target_row;
    current_grid_col = target_col;
    current_grid_height = target_height;

    // RCLCPP_INFO(this->get_logger(), "Target grid(%d,%d) → world(%.2f, %.2f)", target_row, target_col, targetX_world, targetY_world);

    RCLCPP_INFO(this->get_logger(), "Target grid(%d,%d) h=%d dir=%s → world(%.2f,%.2f) reverse=%d rotate=%d target_yaw=%.2f", target_row, target_col, target_height, dir.c_str(),
                targetX_world, targetY_world, need_reverse, need_rotate, rotate_target_yaw);
  }

  bool descend_lifter_up_received = false;

  void descend_lifter_up_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data)
    {

      RCLCPP_WARN(this->get_logger(), "DESCEND LIFTER UP CALLBACK | waiting_confirm=%d lifter_confirmed=%d",
                  waiting_for_lifter_up_confirm, lifter_up_confirmed);

      if (waiting_for_lifter_up_confirm)
      {
        lifter_up_confirmed = true;
        RCLCPP_INFO(this->get_logger(), "LIFTER NAIK KONFIRMASI → WP bisa done");
      }
      else
      {
        descend_lifter_up_received = true;
        RCLCPP_INFO(this->get_logger(), "DESCEND LIFTER UP → lanjut mundur");
      }
      // descend_lifter_up_received = true;

      // if(waiting_for_lifter_up_confirm)
      // {
      //   lifter_up_confirmed = true;
      // }
    }
  }

  void control_loop()
  {
    if (!start_received)
      return;

    // if ((current_state == WAITING_FOR_TARGET || current_state == MOVING_TO_TARGET) && !target_received)
    //   return;
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

    if (!decision_receive)
    {
      cmd.linear.x = 0;
      cmd.linear.y = 0;
      cmd.angular.z = 0;
      RCLCPP_INFO(this->get_logger(), " Waiting target receive from gui......");
      RCLCPP_INFO(this->get_logger(), "posisi x %f", currentX);
      RCLCPP_INFO(this->get_logger(), "posisi y %f\n", currentY);
      // RCLCPP_INFO(this->get_logger(), "posisi y %f\n", msg->data[6]);
      
      return;
    }

    switch (current_state)
    {

    // ================================================STAGE 1==========================================================
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

      if ((tof_valid && distance < TOF_THRESHOLD || sensor_obstacle || obstacle_lock) && !ignore_obstacle)
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

      if (distance > 0.05)
      {

        cmd.linear.x = 0.2 * std::cos(angle);
        // cmd.linear.x = controlled_distance * std::cos(angle);
        cmd.linear.y = controlled_distance * std::sin(angle);
        cmd.angular.z = -controlled_angle;
        //  cmd.angular.z = 0.0;
        RCLCPP_INFO(this->get_logger(), "move robot");

        // }
      }
      else
      {
        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        cmd.angular.z = -controlled_angle;
        // cmd.angular.z = 0.0;

        RCLCPP_INFO(this->get_logger(), "holding position");

        if (!target_reached_flag && !paused)
        // if (!target_reached_flag && !paused && !just_resumed && target_received && distance < 0.03)
        {

          target_reached_flag = true;

          current_state = TARGET_REACHED;
          stage1_target_count++;

          // std_msgs::msg::Bool reached_msg;
          // reached_msg.data = true;
          // reached_pub->publish(reached_msg);

          RCLCPP_INFO(this->get_logger(), "✅ Target %d/%d reached!", stage1_target_count, stage1_targets_total);

          if (stage1_target_count >= stage1_targets_total)
          {
            stage1_target_count = 0;
            target_received = false;

            if (!stage1_climb_done)
            {
              stage1_completed = true;
              stage1_climb_done = true;
              current_state = STRAIGHT_FOR_CLIMB;
              RCLCPP_INFO(this->get_logger(), "STAGE1 COMPLATED - Moving to Stage 2");
            }
            else
            {
              current_state = TARGET_REACHED;
              RCLCPP_INFO(this->get_logger(), "STAGE3 WP reached, waiting next");
              // RCLCPP_INFO(this->get_logger(), "Stage3 WP reached, tidak trigger climb");
            }
          }
            // RCLCPP_INFO(this->get_logger(), "target reached");
          else
          {

            RCLCPP_INFO(this->get_logger(), "wait for next target");
          }

          std_msgs::msg::Bool reached_msg;
          reached_msg.data = true;
          reached_pub->publish(reached_msg);
          RCLCPP_INFO(this->get_logger(), "target reached published");
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

        // just_resumed = true;

        current_state = previous_state_before_pause;
        // current_state = MOVING_TO_TARGET;
        target_reached_flag = false;
        RCLCPP_INFO(this->get_logger(), "Resuming from pause");
        RCLCPP_INFO(this->get_logger(), "posisi x %f", currentX);
        RCLCPP_INFO(this->get_logger(), "posisi y %f\n", currentY);
        // swing_reset_done = false;
      }
      else
      {

        RCLCPP_INFO(this->get_logger(), " PAUSED - Waiting for obstacle to clear...");
        RCLCPP_INFO(this->get_logger(), "posisi x %f", currentX);
        RCLCPP_INFO(this->get_logger(), "posisi y %f\n", currentY);
      }
    }
    break;

    case STRAIGHT_FOR_CLIMB:
    {
      cmd.linear.x = 0.3;
      cmd.linear.y = 0.0;
      // cmd.angular.z = 0.0;
      cmd.angular.z = -controlled_angle;

      RCLCPP_INFO(this->get_logger(), "STRAIGHT MODE - jalan lurus");

      break;
    }

    case WAIT_LIFTER:
    {
      cmd.linear.x = 0.0;
      cmd.linear.y = 0.0;
      cmd.angular.z = 0.0;

      RCLCPP_INFO(this->get_logger(), "WAITING LIFTER");

      if (!wait_lifter_timer_init)
      {
        wait_lifter_start_time = this->now().seconds();
        wait_lifter_timer_init = true;
        RCLCPP_WARN(this->get_logger(), "WAIT_LIFTER: timer dimulai (timeout 5s)");
      }
 
      double wait_elapsed = this->now().seconds() - wait_lifter_start_time;
      RCLCPP_INFO(this->get_logger(), "WAITING LIFTER (stage1) elapsed=%.1fs", wait_elapsed);
 
      if (wait_elapsed > 5.0)
      {
        wait_lifter_timer_init = false;
        RCLCPP_WARN(this->get_logger(), "WAIT_LIFTER TIMEOUT → paksa lanjut ke AFTER_CLIMB");
        current_state = AFTER_CLIMB;
      }

      break;
    }

    case AFTER_CLIMB:
    {
      cmd.linear.x = 0.4;
      cmd.linear.y = 0.0;
      // cmd.angular.z = 0.0;
      cmd.angular.z = -controlled_angle;

      RCLCPP_INFO(this->get_logger(), "LIFTER TURUN MASUK KE MODE PELAN");

      break;
    }

    case STOP_SENSOR:
    {
      cmd.linear.x = 0.0;
      cmd.linear.y = 0.0;
      cmd.angular.z = 0.0;
      // cmd.angular.z = -controlled_angle;

      RCLCPP_INFO(this->get_logger(), "STOP 1 sensor WAITING LIFTER");

      if (!stop_initialize)
      {
        stop_time = this->now().seconds();
        stop_initialize = true;
      }

      // if(!start_saved)
      if (this->now().seconds() - stop_time > 2.0)
      {
        start_x_after_climb = currentX;
        start_y_after_climb = currentY;

        start_saved = true;

        current_state = POST_CLIMB_MOVE;
      }

      break;
    }

    case POST_CLIMB_MOVE:
    {

      double dx = currentX - start_x_after_climb;
      double dy = currentY - start_y_after_climb;
      double dist = sqrt(dx * dx + dy * dy);

      if (dist < 0.2)
      {
        cmd.linear.x = 0.3;
        cmd.linear.y = 0.0;
        cmd.angular.z = 0.0;
      }
      else
      {
        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        cmd.angular.z = -controlled_angle;

        std_msgs::msg::Bool reachedClimb_msg;
        reachedClimb_msg.data = true;
        climb_done_pub->publish(reachedClimb_msg);

        RCLCPP_INFO(this->get_logger(), "masuk stage 2");

        current_state = STAGE2_MOVE_STEPbSTEP;

        is_stage2 = true;
        start_saved = false;
        stage2_initiallized = false;
        stop_initialize = false;
        st2_state = ST2_IDLE;

        if (!latest_next_step.empty())
        {
          // update_target_from_step(latest_next_step);
          int row_ins = parse_int_from_json(latest_next_step, "row");
          int col_ins = parse_int_from_json(latest_next_step, "col");
          int height_ins = parse_int_from_json(latest_next_step, "height");
          std::string dir_ins = parse_direction_from_json(latest_next_step);

          if (row_ins >= 0 && col_ins >= 0 && !dir_ins.empty())
          {
            update_target_from_step(row_ins, col_ins, height_ins, dir_ins);
          }
          goal_active = true;
          has_wp = true;
          wp_processing = false;
        }

        RCLCPP_INFO(this->get_logger(), "ROBOT SUDAH DI TENGAH GRID");
      }

      break;
    }

      // ================================================STAGE 2====================================================

    case STAGE2_MOVE_STEPbSTEP:
    {
      if (!goal_active && st2_state != ST2_EXIT_ROTATE)
      {
        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        cmd.angular.z = 0.0;
        break;
      }

      if (!stage2_initiallized)
      {
        st2_state = ST2_IDLE;

        // has_wp = false;
        goal_done = false;
        climb_finished = false;
        wp_processing = false;

        // limit_triggered = false;

        stage2_initiallized = true;

        RCLCPP_INFO(this->get_logger(), "STAGE2 INIT");
      }

      switch (st2_state)
      {
      case ST2_IDLE:
      {
        // if (goal_done)
        // {

        //   goal_done = false;

        //   RCLCPP_INFO(this->get_logger(), "FINAL STATE READY");

        //   st2_state = ST2_AFTER_CLIMB;
        //   break;
        // }

        if (has_wp && !wp_processing)
        {
          wp_processing = true;

          double dx = targetX_world - currentX;
          double dy = targetY_world - currentY;
          double dist = sqrt(dx * dx + dy * dy);

          if (dist < 0.2) // sudah di WP ini, skip
          {
            // RCLCPP_INFO(this->get_logger(), "Skip WP, robot sudah di sini");

            std_msgs::msg::Bool wp_msg;
            wp_msg.data = true;
            wp_reached2_pub->publish(wp_msg);

            wp_processing = false;
            has_wp = false;
          }
          else if (need_rotate)
          {
            st2_state = ST2_ROTATE;
            RCLCPP_INFO(this->get_logger(), "ST2 IDLE: perlu rotate → ST2_ROTATE");
          }
          else
          {
            st2_state = ST2_STRAIGHT_CLIMB;
            RCLCPP_INFO(this->get_logger(), "ST2 IDLE: langsung maju → ST2_STRAIGHT_CLIMB");
          }
        }

        break;
      }

      case ST2_STRAIGHT_CLIMB:
      {
        double dx = targetX_world - currentX;
        double dy = targetY_world - currentY;

        double dist = std::sqrt(dx * dx + dy * dy);

        double yaw = heading; // pastikan ini dari odom

        // normalize yaw biar stabil
        yaw = std::atan2(std::sin(yaw), std::cos(yaw));

        // WORLD → ROBOT FRAME transform
        double dx_r = std::cos(yaw) * dx + std::sin(yaw) * dy;
        double dy_r = -std::sin(yaw) * dx + std::cos(yaw) * dy;

        // logging debug (penting untuk cek arah)
        RCLCPP_INFO(this->get_logger(),
                    "ST2 dx=%.2f dy=%.2f dx_r=%.2f dy_r=%.2f dist=%.2f",
                    dx, dy, dx_r, dy_r, dist);

        // === safety check ===
        if (dist > 3.0)
        {
          RCLCPP_WARN(this->get_logger(), "ST2: waypoint terlalu jauh / error drift");
          has_wp = false;
          st2_state = ST2_IDLE;
          break;
        }

        // === arrival condition ===
        if (dist < 0.25)
        {
          if (need_reverse)
          {
            st2_state = ST2_WAIT_LIFTER;
          }
          else if (need_climb && !need_reverse)
          {
            st2_state = ST2_WAIT_LIFTER;
          }
          else
          {
            std_msgs::msg::Bool wp_msg;
            wp_msg.data = true;
            wp_reached2_pub->publish(wp_msg);

            has_wp = false;
            goal_active = false;
            wp_processing = false;

            st2_state = ST2_IDLE;

            RCLCPP_INFO(this->get_logger(), "ST2 WP REACHED");
          }
          break;
        }

        if (need_reverse)
        {
          if (proxy_currently_detected)
          {
            proxy_was_seen = true;
            RCLCPP_WARN(this->get_logger(), "PROXY TRUE DETECTED saat mundur"); // ← tambah ini
          }

          if (!proxy_currently_detected && proxy_was_seen)
          {
            cmd.linear.x = 0.0;
            cmd.linear.y = 0.0;
            cmd.angular.z = 0.0;

            RCLCPP_WARN(this->get_logger(),
                        "PROXY HILANG: was_seen=%d stop_triggered=%d", // ← tambah ini
                        proxy_was_seen, proxy_stop_triggered);

            if (!proxy_stop_triggered)
            {
              proxy_stop_triggered = true;
              proxy_was_seen = false;

              std_msgs::msg::Bool descend_msg;
              descend_msg.data = true;
              descend_pub->publish(descend_msg);

              RCLCPP_INFO(this->get_logger(), "PROXY TIDAK TERDETEKSI -> ROBOT BERHENTI (saat mundur)");
              st2_state = ST2_WAIT_LIFTER;
            }
          }
          else if (!proxy_stop_triggered)
          {
            cmd.linear.x = -0.3;
          }
        }
        else
        {
          // === MOTION CONTROL (FIX UTAMA) ===
          cmd.linear.x = 0.4 * dx_r;
          cmd.linear.y = 0.4 * dy_r;
        }
        // IMPORTANT: jangan pakai angular PID di translasi
        cmd.angular.z = 0.0;

        break;
      }

      case ST2_REVERSE_AFTER_LIFTER:
      {
        cmd.linear.x = -0.3;
        cmd.angular.z = 0.0;
        // cmd.angular.z = -controlled_angle;

        RCLCPP_INFO(this->get_logger(), "MUNDUR TUNGGU PITCH > 2 | pitch=%.2f", hardware_pitch);
        RCLCPP_INFO(this->get_logger(),
                    "MUNDUR | pitch=%.2f proxy_was_seen=%d proxy_currently=%d proxy_stop=%d st2=%d",
                    hardware_pitch, proxy_was_seen, proxy_currently_detected, proxy_stop_triggered, (int)st2_state);

        if (st2_reverse_start_time == 0.0)
        {
          st2_reverse_start_time = this->now().seconds();
        }

        double elapsed = this->now().seconds() - st2_reverse_start_time;

        // static int pitch_stable_count = 0;

        if (hardware_pitch > 1.7)
          pitch_stable_count++;
        else
          pitch_stable_count = 0;

        if (pitch_stable_count >= 10 && elapsed > 1.0)
        {
          pitch_stable_count = 0;

          std_msgs::msg::Bool allow_msg;
          allow_msg.data = true;
          allow_lifter_up_pub->publish(allow_msg);

          targetX_world = currentX;
          targetY_world = currentY;

          proxy_was_seen = false;
          // proxy_stop_triggered = false;

          waiting_for_lifter_up_confirm = true;
          lifter_up_confirmed = false;

          st2_state = ST2_WAIT_LIFTER_UP_CONFIRM;

          // st2_state = ST2_STRAIGHT_CLIMB;
          RCLCPP_INFO(this->get_logger(), "PITCH > 2 -> IZIN LIFTER NAIK -> LANJUT MUNDUR");
        }

        // if (hardware_pitch > 2.0 && elapsed > 1.0)
        // {
        //   std_msgs::msg::Bool allow_msg;
        //   allow_msg.data = true;
        //   allow_lifter_up_pub->publish(allow_msg);

        //   targetX_world = currentX;
        //   targetY_world = currentY;

        //   proxy_was_seen = false;
        //   // proxy_stop_triggered = false;

        //   waiting_for_lifter_up_confirm = true;
        //   lifter_up_confirmed = false;

        //   st2_state = ST2_WAIT_LIFTER_UP_CONFIRM;

        //   // st2_state = ST2_STRAIGHT_CLIMB;
        //   RCLCPP_INFO(this->get_logger(), "PITCH > 2 -> IZIN LIFTER NAIK -> LANJUT MUNDUR");
        // }

        break;
      }

      case ST2_WAIT_LIFTER_UP_CONFIRM:
      {
        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        cmd.angular.z = 0.0;

        RCLCPP_INFO(this->get_logger(), "MENUNGGU KONFIRMASI LIFTER NAIK SELESAI...");

        if (lifter_up_confirmed)
        {
          lifter_up_confirmed = false;
          waiting_for_lifter_up_confirm = false;

          if (has_pending_step)
          {
            update_target_from_step(pending_row, pending_col, pending_height, pending_dir);
            has_wp = true;
            goal_active = true;
            wp_processing = false;
            wp_done_sent = false;
            climb_finished = false;
            goal_done = false;
            descend_published = false;
            waiting_lifter = false;
            proxy_was_seen = false;
            proxy_stop_triggered = false;
            has_pending_step = false;
            RCLCPP_INFO(this->get_logger(), "APPLY PENDING STEP → %s", pending_dir.c_str());
          }
          else
          {
            if (is_exit_step)
            {
              rotate_target_yaw = 0.0;
              st2_state = ST2_EXIT_ROTATE;
              is_exit_step = false;
              RCLCPP_INFO(this->get_logger(), "EXIT STEP → mutar hadap depan");
            }
            else
            {
              std_msgs::msg::Bool wp_msg;
              wp_msg.data = true;
              wp_reached2_pub->publish(wp_msg);
              RCLCPP_INFO(this->get_logger(), "WP REACHED → minta next_step");

              st2_state = ST2_IDLE;
            }
          }

          // targetX_world = currentX;
          // targetY_world = currentY;

          // st2_state = ST2_STRAIGHT_CLIMB;
          RCLCPP_INFO(this->get_logger(), "LIFTER KONFIRMASI NAIK -> LANJUT");
        }
        break;
      }

      case ST2_WAIT_LIFTER:
      {
        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        cmd.angular.z = 0.0;

        if (!wait_lifter_st2_timer_init)
        {
          wait_lifter_st2_start_time = this->now().seconds();
          wait_lifter_st2_timer_init = true;
          RCLCPP_WARN(this->get_logger(), "ST2_WAIT_LIFTER: timer dimulai (timeout 5s)");
        }
 
        double st2_wait_elapsed = this->now().seconds() - wait_lifter_st2_start_time;
        RCLCPP_INFO(this->get_logger(), "WAITING LIFTER IN STAGE 2 elapsed=%.1fs", st2_wait_elapsed);

        if (descend_lifter_up_received && need_reverse)
        {
          descend_lifter_up_received = false;
          proxy_stop_triggered = false;
          proxy_was_seen = false;
          st2_reverse_start_time = 0.0;
          pitch_stable_count = 0;
          wait_lifter_st2_timer_init = false;

          targetX_world = currentX;
          targetY_world = currentY;

          st2_state = ST2_REVERSE_AFTER_LIFTER;

          RCLCPP_INFO(this->get_logger(), "LIFTER UP DETECTED → lanjut maju mundur");
        }
        else
        {
          if (st2_wait_elapsed > 5.0)
          {
            wait_lifter_st2_timer_init = false;
            st2_state = ST2_AFTER_CLIMB;
            RCLCPP_WARN(this->get_logger(), "ST2_WAIT_LIFTER TIMEOUT (naik) → paksa ST2_AFTER_CLIMB");
          }
        }

        RCLCPP_INFO(this->get_logger(), "WAITING LIFTER IN STAGE 2");

        break;
      }

      case ST2_EXIT_ROTATE:
      {

        RCLCPP_INFO(this->get_logger(), "EXIT ROTATE | heading=%.2f target=%.2f yaw_error=%.2f",
                    heading, rotate_target_yaw, rotate_target_yaw - heading);

        double yaw_error = normalize_angle(rotate_target_yaw - heading);

        if (fabs(yaw_error) > 0.05)
        {
          // cmd.angular.z = (yaw_error > 0) ? 0.3 : -0.3;
          cmd.angular.z = std::copysign(0.3, yaw_error);
          exit_rotate_stable_count = 0;
        }
        else
        {
          cmd.angular.z = 0.0;
          cmd.linear.x = 0.0;

          exit_rotate_stable_count++;

          RCLCPP_INFO(this->get_logger(), "EXIT ROTATE stable count: %d/5", exit_rotate_stable_count);

          if(exit_rotate_stable_count >= 5)
          {
            exit_rotate_stable_count = 0;
            
            geometry_msgs::msg::Point exit_msg;
            exit_msg.x = currentX;
            exit_msg.y = currentY;
            stage2_exit_pub->publish(exit_msg);
  
            std_msgs::msg::Bool reached_msg;
            reached_msg.data = true;
            reached_pub->publish(reached_msg);
            RCLCPP_INFO(this->get_logger(), "EXIT ROTATE DONE → STAGE 3 pos=(%.2f, %.2f)", currentX, currentY);
  
            // current_state = MOVING_TO_TARGET;
            current_state = MOVING_TO_TARGET_STAGE3;
            st2_state = ST2_IDLE;
            is_stage2 = false;
          }
        }
        break;
      }

      case ST2_DONE:
      {
        if (!climb_finished)
        {
          break;
        }
        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        cmd.angular.z = -controlled_angle;

        has_wp = false;

        if (!wp_done_sent)
        {
          std_msgs::msg::Bool done_msg;
          done_msg.data = true;
          wp_done_pub->publish(done_msg);

          wp_done_sent = true;

          has_wp = false;
          goal_active = false;
          wp_processing = false;

          RCLCPP_INFO(this->get_logger(), "WP DONE PUBLISHED");
        }

        RCLCPP_INFO(this->get_logger(), "STAGE 2 DONE - ROBOT STOP");

        break;
      }

      case ST2_AFTER_CLIMB:
      {
        cmd.linear.x = 0.4;
        cmd.angular.z = -controlled_angle;

        RCLCPP_INFO(this->get_logger(), "ST2 AFTER CLIMB: jalan sampai limit");

        break;
      }

      case ST2_STOP_SENSOR:
      {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;

        if (!stop_initialize)
        {
          stop_time = this->now().seconds();
          stop_initialize = true;
        }

        if (this->now().seconds() - stop_time > 2.0)
        {
          start_x_after_climb = currentX;
          start_y_after_climb = currentY;

          st2_state = ST2_POST_CLIMB;
          stop_initialize = false;
        }

        RCLCPP_INFO(this->get_logger(), "ST2 STOP SENSOR");

        break;
      }

      // bool descend_published = false;
      case ST2_ROTATE:
      {

        yaw_error = normalize_angle(rotate_target_yaw - heading);

        RCLCPP_INFO(this->get_logger(), "ST2 ROTATE: heading=%.3f target=%.3f error=%.3f",
                    heading, rotate_target_yaw, yaw_error);

        if (fabs(yaw_error) < 0.03)
        {
          cmd.linear.x = 0.0;
          cmd.linear.y = 0.0;
          cmd.angular.z = 0.0;
          need_rotate = false;

          st2_state = ST2_STRAIGHT_CLIMB;

          RCLCPP_INFO(this->get_logger(), "ST2 ROTATE DONE → %s", need_reverse ? "mundur" : "maju");
        }
        else
        {
          cmd.linear.x = 0.0;
          cmd.linear.y = 0.0;
          cmd.angular.z = std::copysign(0.2, yaw_error);
        }

        break;
      }

      case ST2_POST_CLIMB:
      {
        double dx = currentX - start_x_after_climb;
        double dy = currentY - start_y_after_climb;
        double dist = sqrt(dx * dx + dy * dy);
        double angle_st2 = atan2(dy, dx);

        if (dist < 0.3)
        {
          // cmd.linear.x = 0.2;
          // cmd.linear.y = 0.0;

          if (dist < 0.01)
          {
            cmd.linear.x = 0.2;
            cmd.linear.y = 0.0;
          }
          else
          {
            // cmd.linear.x = 0.2 * cos(angle_st2);
            // cmd.linear.y = 0.2 * sin(angle_st2);

            cmd.linear.x = 0.2;
            cmd.linear.y = 0.0;
          }

          cmd.angular.z = -controlled_angle;

          // cmd.linear.x = 0.2 * cos(angle_st2);
          // cmd.linear.y = 0.2 * sin(angle_st2);
        }
        else
        {
          cmd.linear.x = 0.0;
          cmd.linear.y = 0.0;
          cmd.angular.z = 0.0;

          double raw_odom_x = currentX - odom_offset_x;
          double raw_odom_y = currentY - odom_offset_y;

          odom_offset_x = targetX_world - raw_odom_x;
          odom_offset_y = targetY_world - raw_odom_y;

          RCLCPP_INFO(
              this->get_logger(),
              "ODOM SYNC -> current dianggap di (%.2f, %.2f)",
              targetX_world,
              targetY_world);

          // climb_finished = true;
          std_msgs::msg::Bool wp_msg;
          wp_msg.data = true;
          wp_reached2_pub->publish(wp_msg);

          has_wp = false;
          goal_active = false;
          wp_processing = false;

          climb_finished = true;
          st2_state = ST2_IDLE;

          // st2_state = ST2_DONE;
        }

        RCLCPP_INFO(this->get_logger(), "ST2 POST CLIMB");

        break;
      }
      }

      break;
    }

    // =====================================================STAGE 3============================================================

    case MOVING_TO_TARGET_STAGE3:
    {

      bool is_climbing = fabs(hardware_pitch) > 1.5;

      RCLCPP_INFO(this->get_logger(), "STAGE3 MOVE: pos(%.2f,%.2f) target(%.2f,%.2f) dist=%.3f",
              currentX, currentY, targetX, targetY, distance);

      if(is_climbing || climbing_stage3)
      {
        climbing_stage3 = true;

        cmd.linear.x = 0.6;
        cmd.linear.y = 0.0;
        cmd.angular.z = -controlled_angle;

        RCLCPP_INFO(this->get_logger(), "STAGE3 CLIMBING pitch=%.2f", hardware_pitch);

        if(fabs(hardware_pitch) < 1.0)
        {
          climbing_stage3 = false;
          RCLCPP_INFO(this->get_logger(), "STAGE3 CLIMB DONE → balik odom");
        }
      }

      else if(distance > 0.03)
      {
        // cmd.linear.x = controlled_distance * std::cos(angle);
        cmd.linear.x = 0.3 * std::cos(angle);
        cmd.linear.y = controlled_distance * std::sin(angle);
        // cmd.linear.y = 0.3;
        cmd.angular.z = -controlled_angle;
      }
      else
      {
        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        cmd.angular.z = -controlled_angle;

        if(!target_reached_flag)
        {
          target_reached_flag = true;
          current_state = TARGET_REACHED;

          std_msgs::msg::Bool reached_msg;
          reached_msg.data = true;
          reached_pub->publish(reached_msg);

          RCLCPP_INFO(this->get_logger(), "STAGE3 WP REACHED → publish reached");

        }
      }
      break;

    }

    case TARGET_REACHED:
      cmd.linear.x = 0.0;
      cmd.linear.y = 0.0;
      cmd.angular.z = -controlled_angle;

      if (stage1_completed && !stage2_triggred)
      {

        stage2_triggred = true;
        // current_state = STAGE2_MOVE_STEPbSTEP;
        stage1_completed = false; // ✅ RESET FLAG
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
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gui_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr limit_stage2_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr after_climb_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reached_Astar_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr proxy_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr toStage2_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr wait_lifter_sub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr proxy_true_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reached_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr wp_reached2_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr climb_done_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr wp_done_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr descend_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr allow_lifter_up_pub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr buttonStage3_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr descend_lifter_up_sub;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr checking_input_sub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr path_list_sub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr next_step_sub;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr stage2_exit_pub;
  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr ir_code_sub;
  // di bagian private class Movement
  nav_msgs::msg::Odometry odom_robot_msg;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time hold_start_time;
};

// bool Movement::swing_reset_done = false;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Movement>());
  rclcpp::shutdown();
  return 0;
}