#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int8.hpp>
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

    lifter_grid_pub = this->create_publisher<std_msgs::msg::Int8>("lifter_grid_control", 10);

    limit_sub = this->create_subscription<std_msgs::msg::Bool>("limitSlideData", 10,
                                                               std::bind(&waypointPublish::limit_callback, this, std::placeholders::_1));

    // exit_pos_sub = this->create_subscription<geometry_msgs::msg::Point>("/stage2_exit_pos", 10,
    //                                                                     std::bind(&waypointPublish::exit_pos_callback, this, std::placeholders::_1));

    buttonStage2_sub = this->create_subscription<std_msgs::msg::Bool>("/button_stage2", 10,
                                                                      std::bind(&waypointPublish::buttonStage2_callback, this, std::placeholders::_1));

    entry_sub = this->create_subscription<std_msgs::msg::String>("/meihua/entry_point", 10,
                                                                    std::bind(&waypointPublish::entry_callback, this,std::placeholders::_1));

    robot_up_sub = this->create_subscription<std_msgs::msg::Bool>("/robot_up_wp0", 10,
                                                                  std::bind(&waypointPublish::robot_up_callback, this, std::placeholders::_1));

    start_sub = this->create_subscription<std_msgs::msg::Bool>("robot_start", 10,
                                                                std::bind(&waypointPublish::start_callback, this, std::placeholders::_1));

    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10,
                                                                  [this](const nav_msgs::msg::Odometry::SharedPtr msg)
                                                                  {
                                                                    current_robot_x = msg->pose.pose.position.x;
                                                                    current_robot_y = msg->pose.pose.position.y;
                                                                  });

    // toStage2_pub = this->create_publisher<std_msgs::msg::Bool>("toStage2", 10);

    waypoint = {

        // {0.2, 0.0}
        // {0.0, 1.0}

        {0.0, 1.1},
        {1.1, 1.1},
        {1.1, -1.15},
        {1.5, -1.15}};

        // {0.0, 1.1},
        // {-1.0, 1.1},
        // {1.1, -1.15},
        // {1.5, -1.15}
      // };

    current_waypoint_index = 0;
    waiting_ir = false;
    reached_latched = false;
    all_completed = false;

    RCLCPP_INFO(this->get_logger(), "Waypoint Publisher Ready, menunggu robot_start...");
    // send_waypoint(); // ← JANGAN langsung jalan di sini, harus nunggu tombol start (topic "robot_start")
  }

private:
  enum class WP_STATE
  {
    MOVING,
    WAIT_IR,
    WAIT_LIFTER,
    WAIT_STAGE2_LIFTER,
    COMPLETE
  };

  WP_STATE state = WP_STATE::MOVING;

  bool waiting_ir = false;
  bool reached_latched = false;
  bool all_completed = false;
  double exit_x = 0.0, exit_y = 0.0;
  double current_robot_x = 0.0, current_robot_y = 0.0;
  bool limit_triggered = false;
  bool exit_pos_received = false;
  bool stage2_done = false;
  bool exit_pos_pushed = false;
  bool stage1_done = false;
  bool lifter_wp0_triggered = false;
  bool retry_mode = false;
  bool start_triggered = false;

  size_t current_waypoint_index;

  static constexpr int IR_TIMEOUT_MS = 5000;

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

    if(current_waypoint_index == 0 && !retry_mode && !lifter_wp0_triggered)
    {
      lifter_wp0_triggered = true;

      std_msgs::msg::Bool lifter_msg;
      lifter_msg.data = true;
      lifter_pub->publish(lifter_msg);

      RCLCPP_INFO(this->get_logger(), "Robot mulai jalan ke WP0 -> lifter turun paralel");
    }

    RCLCPP_INFO(this->get_logger(), "Sending waypoint %ld (%.2f, %.2f)", current_waypoint_index, point.x, point.y);
  }

  // ===================== ADVANCE (ONLY ONE ENTRY POINT) =====================
  void advance_waypoint()
  {
    current_waypoint_index++;
    waiting_ir = false;
    state = WP_STATE::MOVING;

    send_waypoint();
  }

  void trigger_lifter_delayed(int delay_ms)
  {
    lifter_timer = this->create_wall_timer(
        std::chrono::milliseconds(delay_ms),
        [this]()
        {
          lifter_timer->cancel(); // one-shot, cancel setelah sekali jalan

          std_msgs::msg::Bool lifter_msg;
          lifter_msg.data = true;
          lifter_pub->publish(lifter_msg);

          RCLCPP_INFO(this->get_logger(), "robot naik (setelah delay)");
        });
  }

  // ===================== START CALLBACK (tombol start GUI) =====================
  void start_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data)
      return;
    if (start_triggered)
      return; // anti double trigger, jaga2 kalau topic dipublish berkali2

    start_triggered = true;

    RCLCPP_INFO(this->get_logger(), "ROBOT_START DITERIMA -> mulai kirim waypoint pertama");
    send_waypoint();
  }

  // ===================== REACHED CALLBACK =====================
  void reached_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (all_completed)
      return;
    if (!msg->data)
      return;
    if (reached_latched)
      return; // anti double trigger

    reached_latched = true;

    RCLCPP_INFO(this->get_logger(), "WAYPOINT REACHED: %ld", current_waypoint_index);
    if (retry_mode)
    {
      advance_waypoint();
      return;
    }

    // WP0 action
    if (current_waypoint_index == 0)
    {

      geometry_msgs::msg::Point stop_point;
      stop_point.x = current_robot_x;
      stop_point.y = current_robot_y;
      stop_point.z = 0.0;
      pose_pub->publish(stop_point);

      waypoint[1].second = current_robot_y;

      y_track_timer = this->create_wall_timer(std::chrono::milliseconds(100),[this]()
      {
        if(current_waypoint_index != 1){
          y_track_timer->cancel();
          return;
        }

        double dx = waypoint[1].first - current_robot_x;

        if(dx > 0.1)
        {
          waypoint[1].second = current_robot_y;

          geometry_msgs::msg::Point point;
          point.x = waypoint[1].first;
          point.y = waypoint[1].second;
          point.z = 0.0;
          pose_pub->publish(point);
        }
        else
        {
          y_track_timer->cancel();
        }
      });

      RCLCPP_INFO(this->get_logger(), "WP1 di-snap ke Y robot: %.3f", current_robot_y);

      // advance_waypoint();
      // return;

      // std_msgs::msg::Bool lifter_msg;
      // lifter_msg.data = true;
      // lifter_pub->publish(lifter_msg);

      // RCLCPP_INFO(this->get_logger(), "Trigger lifter turun");

      // RCLCPP_INFO(this->get_logger(), "WP0 reached → lanjut ke WP1, lifter turun dalam 1500ms...");

      // advance_waypoint();           // robot langsung jalan ke WP1

      state = WP_STATE::WAIT_LIFTER;

      // trigger_lifter_delayed(900); // lifter turun 1500ms kemudian (sambil robot jalan)
      return;
    }

    // WP1 → WAIT IR
    if (current_waypoint_index == 1 && !stage1_done)
    {
      state = WP_STATE::WAIT_IR;
      waiting_ir = true;

      RCLCPP_INFO(this->get_logger(), "WAITING IR...");

      ir_timeout_timer = this->create_wall_timer(                 // ← BARU
      std::chrono::milliseconds(IR_TIMEOUT_MS),
      [this]()
      {
        ir_timeout_timer->cancel();
        if (state != WP_STATE::WAIT_IR) return;
        RCLCPP_WARN(this->get_logger(), "IR TIMEOUT...");
        advance_waypoint();
      });
      return;
    }

    if (current_waypoint_index == 2)
    {
      geometry_msgs::msg::Point stop_point;
      stop_point.x = current_robot_x;
      stop_point.y = current_robot_y;
      stop_point.z = 0.0;
      pose_pub->publish(stop_point);

      waypoint[3].second = waypoint[2].second; 
      RCLCPP_INFO(this->get_logger(), "WP3 dikunci ke target ideal Y: %.3f", waypoint[3].second);


      // 2. Kirim perintah gerak posisi lifter langsung ke Teensy sesuai Grid GUI
      std_msgs::msg::Int8 grid_msg;
      grid_msg.data = selected_entry_col; // Mengirim angka 0, 1, atau 2
      lifter_grid_pub->publish(grid_msg);
      RCLCPP_INFO(this->get_logger(), "Mengirim Perintah Lifter Stage 2 ke Grid: %d", selected_entry_col);

      state = WP_STATE::WAIT_STAGE2_LIFTER;

      // 3. Berikan delay waktu aman (misal 3.5 detik) agar lifter selesai bergerak sebelum maju ke WP3
      stage2_lifter_timer = this->create_wall_timer(
          std::chrono::milliseconds(3500),
          [this]()
          {
            stage2_lifter_timer->cancel(); // One-shot timer
            RCLCPP_INFO(this->get_logger(), "Lifter Stage 2 Siap, Melanjutkan gerak ke Waypoint 3...");
            advance_waypoint(); // Robot mulai maju menembus gerbang
          });
      return;
    }

    // WP lainnya langsung lanjut
    advance_waypoint();
  }

  std::vector<std::pair<double, double>> retry_waypoints = {
      // {0.0, 0.0}, // WP0 — titik awal / sudut pertama
      {1.0, 0.0},  // WP1 — maju
      {1.0, -1.0}, // WP2 — belok kanan
      {0.0, -1.0}, // WP3 — balik
      {0.0, 0.0}   // WP4 — kembali ke titik awal (tutup segi empat)

      // {1.0, 0.0},
      // {1.0, 1.0},
      // {0.0, 1.0},
      // {0.0, 0.0}
  };

  void buttonStage2_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data)
      return;

    RCLCPP_INFO(this->get_logger(), "RETRY STAGE 2 DITERIMA → Reset & kirim ulang WP1");

    all_completed = false;
    reached_latched = false;
    waiting_ir = false;
    limit_triggered = false;
    stage1_done = false;
    stage2_done = false;
    exit_pos_pushed = false;
    exit_pos_received = false;

    retry_mode = true; // ← DITAMBAH
    state = WP_STATE::MOVING;
    waypoint = retry_waypoints;
    current_waypoint_index = 0;

    send_waypoint();
  }

  void limit_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (all_completed)
      return;
    if (!msg->data)
      return;
    if (limit_triggered)
      return;
    if (reached_latched)
      return;
    if (state != WP_STATE::MOVING)
      return;

    if (current_waypoint_index != 0)
      return;

    limit_triggered = true;
    reached_latched = true;

    geometry_msgs::msg::Point stop_point;
    stop_point.x = current_robot_x;
    stop_point.y = current_robot_y;
    stop_point.z = 0.0;
    pose_pub->publish(stop_point);

    RCLCPP_INFO(this->get_logger(), "LIMIT HIT WP0 → Anggap Reached + robot naik");

    waypoint[1].second = current_robot_y;

    y_track_timer = this->create_wall_timer(std::chrono::milliseconds(100), [this]()
                                            {
        if (current_waypoint_index != 1) {
            y_track_timer->cancel();
            return;
        }

        double dx = waypoint[1].first - current_robot_x;

        if (dx > 0.1)
        {
            waypoint[1].second = current_robot_y;

            geometry_msgs::msg::Point point;
            point.x = waypoint[1].first;
            point.y = waypoint[1].second;
            point.z = 0.0;
            pose_pub->publish(point);
        }
        else
        {
            y_track_timer->cancel();
        } });



    RCLCPP_INFO(this->get_logger(), "WP1 di-snap ke Y robot: %.3f", current_robot_y);

    std_msgs::msg::Bool lifter_msg;
    lifter_msg.data = true;
    lifter_pub->publish(lifter_msg);

    RCLCPP_INFO(this->get_logger(), "Trigger lifter turun, robot menunggu...");

    state = WP_STATE::WAIT_LIFTER;

    // advance_waypoint();

    // trigger_lifter_delayed(900);
  }

  void robot_up_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data)
      return;
    if (state != WP_STATE::WAIT_LIFTER)
      return;

    RCLCPP_INFO(this->get_logger(), "LIFTER SUDAH DI POSISI → lanjut jalan");
    advance_waypoint();
  }

  // void exit_pos_callback(const geometry_msgs::msg::Point::SharedPtr msg)
  // {
  //   exit_x = msg->x;
  //   exit_y = msg->y;

  //   exit_pos_received = true;
  //   stage2_done = true;
  //   all_completed = false;
  //   stage1_done = true;

  //   reached_latched = false;
  //   waiting_ir = false;
  //   limit_triggered = false;

  //   state = WP_STATE::MOVING;

  //   waypoint.push_back({exit_x, exit_y + -1.1});
  //   waypoint.push_back({exit_x + 2.0, exit_y + -1.0});

  //   current_waypoint_index = waypoint.size() - 2;

  //   // RCLCPP_INFO(this->get_logger(), "STAGE 3 WAYPOINTS: geser=(%.2f,%.2f) maju=(%.2f,%.2f)",
  //   // exit_x, exit_y - 1.0, exit_x + 1.0, exit_y - 1.0);

  //   RCLCPP_INFO(this->get_logger(),
  //               "STAGE 3 WAYPOINTS: geser=(%.2f,%.2f) maju=(%.2f,%.2f)",
  //               waypoint[waypoint.size() - 2].first,
  //               waypoint[waypoint.size() - 2].second,
  //               waypoint[waypoint.size() - 1].first,
  //               waypoint[waypoint.size() - 1].second);

  //   send_waypoint();
  //   // RCLCPP_INFO(this->get_logger(), "EXIT POS RECEIVED: (%.2f, %.2f)", exit_x, exit_y);
  // }

  void ir_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (all_completed)
      return;
    if (!msg->data)
      return;

    if (state != WP_STATE::WAIT_IR)
      return;

    if (ir_timeout_timer) ir_timeout_timer->cancel();   // ← BARU


    RCLCPP_INFO(this->get_logger(), "IR RECEIVED → CONTINUE");

    advance_waypoint();
  }

  void entry_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    if(current_waypoint_index > 3)
      return;

    RCLCPP_INFO(this->get_logger(), "Menerima target entry dari GUI: %s", msg->data.c_str());

    // int col = 1;
    if(msg->data.find("\"col\": 0") != std::string::npos)       selected_entry_col = 0;
    else if (msg->data.find("\"col\": 1") != std::string::npos) selected_entry_col = 1;
    else if (msg->data.find("\"col\": 2") != std::string::npos) selected_entry_col = 2;

    // Sesuaikan nilai meter (-0.95, -1.45, -1.95) dengan jarak asli di lapangan Anda!
    double target_y = -1.15;
    if (selected_entry_col == 0)       target_y = -0.5;
    else if (selected_entry_col == 1) target_y = -1.15;
    else if (selected_entry_col == 2) target_y = -1.5;

    waypoint[2].second = target_y;

    RCLCPP_INFO(this->get_logger(), "WP2 diupdate ke Y: %.2f", target_y);

  }

  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pose_pub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reached_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ir_sub;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr exit_pos_sub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lifter_pub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr limit_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::TimerBase::SharedPtr lifter_timer;
  rclcpp::TimerBase::SharedPtr y_track_timer;
  rclcpp::TimerBase::SharedPtr stage2_lifter_timer;
  rclcpp::TimerBase::SharedPtr ir_timeout_timer;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr buttonStage2_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr robot_up_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr entry_sub;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr lifter_grid_pub;


  int selected_entry_col = 1;

  std::vector<std::pair<double, double>> waypoint;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<waypointPublish>());
  rclcpp::shutdown();
  return 0;
}