#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "gui_kfs_msgs/msg/kfs_decision.hpp"
#include "mehua_pkg_msgs/msg/kfs_detection_array.hpp"
#include <std_msgs/msg/bool.hpp>
// #include "mehua_pkg_"
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>

// ================================visualisasi grid ======================================

// (0,0) top-left

//         kolom 0   kolom 1   kolom 2
// baris 0    0         1         2
// baris 1    3         4         5
// baris 2    6         7         8
// baris 3    9        10        11

// =====================================================================================================

struct Waypoint
{
  int x;
  int y;
  bool need_climb = false;
};

struct AstarNode
{
  int x, y, theta;
  double f, g, h;
  AstarNode *parent;

  AstarNode(int x_, int y_, int t_)
      : x(x_), y(y_), theta(t_), f(0), g(0), h(0), parent(nullptr) {}
};

struct NodeCmp
{
  bool operator()(AstarNode *a, AstarNode *b)
  {
    return a->f > b->f;
  }
};

class AStarPlanner : public rclcpp::Node
{
public:
  AStarPlanner() : Node("astar_planner")
  {

    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&AStarPlanner::odom_callback, this, std::placeholders::_1));

    decision_sub = this->create_subscription<gui_kfs_msgs::msg::KFSDecision>("/kfs_decision", 10,
                                                                             std::bind(&AStarPlanner::decision_callback, this, std::placeholders::_1));

    detection_yolo_sub = this->create_subscription<mehua_pkg_msgs::msg::KFSDetectionArray>("kfs_detections", 10,
                                                                                           std::bind(&AStarPlanner::kfs_callback, this, std::placeholders::_1));

    wp_reached2_sub = create_subscription<std_msgs::msg::Bool>("/planner/wp_reached", 10,
                                                               std::bind(&AStarPlanner::wp_reached_callback, this, std::placeholders::_1));

    climb_done_sub = create_subscription<std_msgs::msg::Bool>("/planner/climb_done", 10,
                                                              std::bind(&AStarPlanner::climbdone_callback, this, std::placeholders::_1));

    wp_done_sub = create_subscription<std_msgs::msg::Bool>("/wp_done", 10,
                                                           std::bind(&AStarPlanner::wp_done_callback, this, std::placeholders::_1));

    waypoint_pub = this->create_publisher<geometry_msgs::msg::Point>("/planner/waypoint", 10);

    reached_Astar_pub = this->create_publisher<std_msgs::msg::Bool>("/target_Astar_Done", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&AStarPlanner::publish_waypoint, this));

    grid_rows = 4;
    grid_cols = 3;

    grid_.resize(grid_rows, std::vector<int>(grid_cols, 0));

    height_map = {
        {0.4, 0.2, 0.4},
        {0.2, 0.4, 0.6},
        {0.4, 0.6, 0.4},
        {0.2, 0.4, 0.2}};
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::Subscription<gui_kfs_msgs::msg::KFSDecision>::SharedPtr decision_sub;
  rclcpp::Subscription<mehua_pkg_msgs::msg::KFSDetectionArray>::SharedPtr detection_yolo_sub;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr waypoint_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reached_Astar_pub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr wp_reached2_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr climb_done_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr wp_done_sub;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<std::vector<int>> grid_;
  std::vector<std::vector<double>> height_map;
  // std::queue<std::pair<int, int>> target_queue_;
  std::queue<int> target_queue_;

  int grid_rows, grid_cols;

  double robot_x, robot_y = 0;
  int robot_theta = 0;
  bool wp_published = false;

  bool goal_reached = false;
  bool wp_active = false;
  double robot_world_x;
  double robot_world_y;

  int robot_grid_x;
  int robot_grid_y;

  bool is_stage2 = false;

  const double MOVE_CONST = 1.0;
  const double ROTATE_CONST = 2.0;
  const double FORWARD_BONUS = -0.5;

  enum Mode
  {
    IDLE,
    CHASE
  };

  Mode mode = IDLE;

  std::vector<Waypoint> path_;
  size_t current_wp_ = 0;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // robot_x = (msg->pose.pose.position.x / 1.2) + 1.0;
    // robot_y = (msg->pose.pose.position.y / 1.2) + 0.0;

    robot_world_x = msg->pose.pose.position.x;
    robot_world_y = msg->pose.pose.position.y;

    robot_grid_x = std::round(robot_world_x / 1.2) + 1;
    robot_grid_y = std::round(robot_world_y / 1.2);

    double yaw = get_yaw(msg);
    robot_theta = yaw_to_dir(yaw);

    // RCLCPP_INFO(this->get_logger(), "robot grid pos: (%.1f, %.1f)", robot_x, robot_y);
  }

  double get_yaw(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    auto q = msg->pose.pose.orientation;
    double siny = 2 * (q.w * q.z + q.x * q.y);
    double cosy = 1 - 2 * (q.y * q.y + q.z * q.z);
    return atan2(siny, cosy);
  }

  double yaw_to_dir(double yaw)
  {
    // theta: 0=bawah, 1=kanan, 2=atas, 3=kiri
    if (yaw > -M_PI_4 && yaw <= M_PI_4)
      return 2; // robot menghadap depan = atas grid
    if (yaw > M_PI_4 && yaw <= 3 * M_PI_4)
      return 1; // kanan
    if (yaw < -M_PI_4 && yaw >= -3 * M_PI_4)
      return 3; // kiri
    return 0;   // bawah
  }

  // void decision_callback(const gui_kfs_msgs::msg::KFSDecision::SharedPtr msg)
  // {
  //   int target_idx = msg->chase_targets[0];

  //   int gx = target_idx % grid_cols;
  //   int gy = target_idx / grid_cols;

  //   generate_path(gx, gy);
  // }

  void decision_callback(const gui_kfs_msgs::msg::KFSDecision::SharedPtr msg)
  {
    // ===== CHASE =====
    if (msg->chase_targets.empty())
      return;

    is_stage2 = false;

    mode = CHASE;

    path_.clear();
    current_wp_ = 0;

    while (!target_queue_.empty())
      target_queue_.pop();

    // ===== RESET GRID =====
    for (auto &row : grid_)
      std::fill(row.begin(), row.end(), 0);

    // ===== AVOID =====
    for (auto idx : msg->avoid_targets)
    {
      int gx, gy;
      index_to_grid(idx, gx, gy);
      grid_[gy][gx] = 1;
    }

    for (auto target : msg->chase_targets)
    {
      target_queue_.push(target);
    }

    if (!target_queue_.empty())
    {
      int idx = target_queue_.front();
      target_queue_.pop();

      int gx, gy;
      index_to_grid(idx, gx, gy);

      RCLCPP_INFO(this->get_logger(), "START TARGET AWAL → (%d,%d)", gx, gy);

      generate_path(gx, gy);
    }

    // start_next_target();

    // int gx, gy;
    // index_to_grid(msg->chase_targets[0], gx, gy);

    // RCLCPP_INFO(this->get_logger(), "Decision masuk!");
    // RCLCPP_INFO(this->get_logger(), "CHASE RAW: %d → GRID (%d,%d)", msg->chase_targets[0], gx, gy);

    // generate_path(gx, gy);

    // // ===== CHASE =====
    // if (msg->chase_targets.empty())
    //   return;

    // is_stage2 = false;

    // mode = CHASE;

    // path_.clear();
    // current_wp_ = 0;

    // while (!target_queue_.empty())
    // target_queue_.pop();

    // // ===== RESET GRID =====
    // for (auto &row : grid_)
    //   std::fill(row.begin(), row.end(), 0);

    // // ===== AVOID =====
    // for (auto idx : msg->avoid_targets)
    // {
    //   int gx, gy;
    //   index_to_grid(idx, gx, gy);
    //   grid_[gy][gx] = 1;
    // }

    // int gx, gy;
    // index_to_grid(msg->chase_targets[0], gx, gy);

    // RCLCPP_INFO(this->get_logger(), "Decision masuk!");
    // RCLCPP_INFO(this->get_logger(), "CHASE RAW: %d → GRID (%d,%d)", msg->chase_targets[0], gx, gy);

    // generate_path(gx, gy);
  }

  void start_next_target()
  {

    if (!target_queue_.empty())
    {
      int idx = target_queue_.front();
      target_queue_.pop();

      int gx, gy;
      index_to_grid(idx, gx, gy);

      RCLCPP_INFO(this->get_logger(), "START TARGET → IDX %d → GRID (%d,%d)", idx, gx, gy);

      path_.clear();
      current_wp_ = 0;

      is_stage2 = false;

      generate_path(gx, gy);
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "SEMUA TARGET SELESAI");
      publish_goal_reached();
      mode = IDLE;
    }
  }

  void wp_done_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data)
      return;

    current_wp_++;

    RCLCPP_INFO(this->get_logger(), "WP DONE → next index %ld", current_wp_);

    wp_published = false;

    if (current_wp_ >= path_.size())
    {
      RCLCPP_INFO(this->get_logger(), "FINAL TARGET SELESAI");

      start_next_target();
      return;
    }
    publish_waypoint();
  }

  void wp_reached_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data)
      return;

    RCLCPP_INFO(this->get_logger(), "Robot sudah sampai waypoint");

    // wp_published = false;
  }

  void climbdone_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data)
      return;

    is_stage2 = true;

    if (!path_.empty() && current_wp_ < path_.size())
    {
      auto wp = path_[current_wp_];

      // robot_x = wp.x;
      // robot_y = wp.y;

      RCLCPP_INFO(this->get_logger(), "Climb done, robot at (%d,%d)", wp.x, wp.y);
    }
  }

  void publish_goal_reached()
  {
    auto msg = std_msgs::msg::Bool();
    msg.data = true;
    reached_Astar_pub->publish(msg);

    RCLCPP_INFO(this->get_logger(), "Goal reached signal sent → /target_Astar_Done");
  }

  // void wp_reached_callback(const std_msgs::msg::Bool::SharedPtr msg)
  // {
  //   if (!msg->data)
  //     return;

  //   if (current_wp_ < path_.size())
  //   {
  //     robot_x = path_[current_wp_].x;
  //     robot_y = path_[current_wp_].y;
  //     RCLCPP_INFO(this->get_logger(), "WP reached, robot now at (%.0f, %.0f)", robot_x, robot_y);
  //   }

  //   current_wp_++;
  //   publish_waypoint();
  // }

  // void climbdone_callback(const std_msgs::msg::Bool::SharedPtr msg)
  // {
  //   if (!msg->data)
  //     return;

  //   if (path_.empty())
  //   {
  //     robot_x = 1.0;
  //     robot_y = 0.0;
  //     RCLCPP_INFO(this->get_logger(), "Robot arrived at start grid (1,0)");
  //   }
  //   else
  //   {
  //     if (current_wp_ < path_.size())
  //     {
  //       robot_x = path_[current_wp_].x;
  //       robot_y = path_[current_wp_].y;

  //       RCLCPP_INFO(this->get_logger(), "Climb done, robot at (%.0f,%.0f)", robot_x, robot_y);

  //       current_wp_++;

  //     }
  //   }
  // }

  void kfs_callback(const mehua_pkg_msgs::msg::KFSDetectionArray::SharedPtr msg)
  {
    // update_kfs_r1_grid(msg);

    // if (path_constain_blocked_wp())
    // {
    //   RCLCPP_WARN(this->get_logger(), "Path blocked -> replan!");

    //   if (current_wp_ < path_.size())
    //   {
    //     auto wp = path_.back(); // target terakhir
    //     generate_path(wp.x, wp.y);
    //   }
    // }
  }

  void index_to_grid(int index, int &gx, int &gy)
  {
    gx = index % grid_cols;
    gy = index / grid_cols;

    RCLCPP_INFO(this->get_logger(), "IDX %d → GRID (%d,%d)", index, gx, gy);
  }

  //=======================================A*==============================================================

  std::vector<std::pair<int, int>> directions = {
      {0, 1},
      {1, 0},
      {0, -1},
      {-1, 0}};

  bool valid_move(AstarNode *current, int nx, int ny)
  {
    if (nx < 0 || nx >= grid_cols || ny < 0 || ny >= grid_rows)
      return false;

    if (grid_[ny][nx] != 0)
      return false;

    double h_now = height_map[current->y][current->x];
    double h_next = height_map[ny][nx];

    if (fabs(h_next - h_now) > 0.2)
      return false;

    if (h_next > h_now)
    {
      int required_theta = get_required_orientation(current->x, current->y, nx, ny);
      if (current->theta != required_theta)
        return false;
    }
    return true;
  }

  int get_required_orientation(int x, int y, int nx, int ny)
  {
    if (ny == y + 1)
      return 0;
    if (nx == x + 1)
      return 1;
    if (ny == y - 1)
      return 2;
    if (nx == x - 1)
      return 3;
    return 0;
  }

  double heuristic(int x1, int y1, int x2, int y2)
  {
    return abs(x1 - x2) + abs(y1 - y2);
  }

  void generate_path(int goal_x, int goal_y)
  {

    // int start_x = std::round(robot_x);
    // int start_y = std::round(robot_y);

    // int start_x = robot_grid_x;
    // int start_y = robot_grid_y;

    int start_x = std::clamp((int)std::round(robot_grid_x), 0, grid_cols - 1);

    int start_y = std::clamp((int)std::round(robot_grid_y), 0, grid_rows - 1);

    if (grid_[start_y][start_x] != 0)
    {
      RCLCPP_WARN(this->get_logger(), "Start blocked!");
      return;
    }
    if (grid_[goal_y][goal_x] != 0)
    {
      RCLCPP_WARN(this->get_logger(), "Goal blocked!");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "START obstacle? %d", grid_[start_y][start_x]);

    std::priority_queue<AstarNode *, std::vector<AstarNode *>, NodeCmp> open;
    std::vector<AstarNode *> all_nodes;

    AstarNode *start = new AstarNode(start_x, start_y, robot_theta);
    start->g = 0;
    start->h = heuristic(start_x, start_y, goal_x, goal_y);
    start->f = start->g + start->h;

    open.push(start);
    all_nodes.push_back(start);

    // std::vector<std::vector<bool>> closed(grid_rows, std::vector<bool>(grid_cols, false));

    std::vector<std::vector<std::vector<bool>>> closed(grid_rows, std::vector<std::vector<bool>>(grid_cols, std::vector<bool>(4, false)));

    AstarNode *goal_node = nullptr;

    while (!open.empty())
    {
      AstarNode *current = open.top();
      open.pop();

      if (closed[current->y][current->x][current->theta])
        continue;

      closed[current->y][current->x][current->theta] = true;

      // goal check
      if (current->x == goal_x && current->y == goal_y)
      {
        goal_node = current;
        break;
      }

      // arah robot saat ini
      int dx = directions[current->theta].first;
      int dy = directions[current->theta].second;

      bool facing_goal =
          (dx == 1 && goal_x > current->x) ||
          (dx == -1 && goal_x < current->x) ||
          (dy == 1 && goal_y > current->y) ||
          (dy == -1 && goal_y < current->y);

      // ===================== MAJU (HANYA JIKA ARAH BENAR) =====================
      if (facing_goal)
      {
        int nx = current->x + dx;
        int ny = current->y + dy;

        if (valid_move(current, nx, ny))
        {
          AstarNode *n = new AstarNode(nx, ny, current->theta);

          n->g = current->g + MOVE_CONST;
          n->h = heuristic(nx, ny, goal_x, goal_y);
          n->f = n->g + n->h;
          n->parent = current;

          open.push(n);
          all_nodes.push_back(n);
        }
      }

      // ===================== ROTATE (HANYA JIKA BELUM MENGHADAP) =====================
      else
      {
        for (int d : {1, -1})
        {
          int new_theta = (current->theta + d + 4) % 4;

          if (closed[current->y][current->x][new_theta])
            continue;

          AstarNode *n = new AstarNode(current->x, current->y, new_theta);

          n->g = current->g + ROTATE_CONST;
          n->h = heuristic(current->x, current->y, goal_x, goal_y);
          n->f = n->g + n->h;
          n->parent = current;

          open.push(n);
          all_nodes.push_back(n);
        }
      }
    }

    // =======BUILD PATH======

    path_.clear();

    while (goal_node)
    {
      if (path_.empty() ||
          path_.back().x != goal_node->x ||
          path_.back().y != goal_node->y)
      {
        // path_.push_back({goal_node->x, goal_node->y});

        bool need_climb = false;
        if (goal_node->parent)
        {
          double h_now = height_map[goal_node->parent->y][goal_node->parent->x];

          double h_next = height_map[goal_node->y][goal_node->x];

          if (h_next > h_now)
            need_climb = true;
        }

        path_.push_back({goal_node->x,
                         goal_node->y,
                         need_climb});
      }
      goal_node = goal_node->parent;

      RCLCPP_INFO(this->get_logger(), "Path size: %ld, mulai dari WP: %ld", path_.size(), current_wp_);

      wp_published = false;
    }

    std::reverse(path_.begin(), path_.end());

    if (is_stage2 && !path_.empty() && path_[0].x == start_x && path_[0].y == start_y)
    {
      RCLCPP_INFO(this->get_logger(), "Skip WP[0] (%d,%d) robot sudah di sini", start_x, start_y);
      current_wp_ = 1;
    }
    else
    {
      current_wp_ = 0;
    }

    for (auto *n : all_nodes)
      delete n;

    RCLCPP_INFO(this->get_logger(), "Path size: %ld, mulai dari WP: %ld", path_.size(), current_wp_);

    // std::reverse(path_.begin(), path_.end());
    // current_wp_ = 0;

    // for (auto *n : all_nodes)
    //   delete n;

    RCLCPP_INFO(this->get_logger(), "Path size: %ld", path_.size());
  }

  void publish_waypoint()
  {

    if (mode != CHASE)
      return;

    if (path_.empty() || current_wp_ >= path_.size())
      return;

    if (wp_published)
      return;

    auto wp = path_[current_wp_];

    geometry_msgs::msg::Point msg;
    msg.x = wp.x;
    msg.y = wp.y;
    msg.z = wp.need_climb ? 1.0 : 0.0;

    waypoint_pub->publish(msg);
    wp_published = true;

    RCLCPP_INFO(this->get_logger(), "Publish WP: (%.0f, %.0f) climb=%d", msg.x, msg.y, (int)wp.need_climb);

    // RCLCPP_INFO(this->get_logger(), "Publish WP: %.2f %.2f", msg.x, msg.y);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AStarPlanner>());
  rclcpp::shutdown();
  return 0;
}

// struct Waypoint
// {
//   int x;
//   int y;
// };

// struct AStarNode
// {
//   int x, y,theta;
//   double f, g, h;
//   AStarNode *parent;
//   AStarNode(int x_, int y_) : x(x_), y(y_), f(0), g(0), h(0), parent(nullptr) {}
// };

// struct NodeCmp
// {
//   bool operator()(AStarNode *a, AStarNode *b)
//   { return a->f > b->f; }
// };

// class AStarTargetNode : public rclcpp::Node
// {
// public:
//   AStarTargetNode() : Node("a_star_target_node")
//   {
//     // chase_sub = this->create_subscription<geometry_msgs::msg::Point>("/chase_from_backend", 10,
//     //                                                                  std::bind(&AStarTargetNode::chase_callback, this, std::placeholders::_1));

//     // avoid_sub = this->create_subscription<geometry_msgs::msg::Point>("/avoid_from_backend", 10,
//     //                                                                  std::bind(&AStarTargetNode::avoid_callback, this, std::placeholders::_1));

//     odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10,
//                                                                   std::bind(&AStarTargetNode::odom_callback, this, std::placeholders::_1));

//     waypoint_pub = this->create_publisher<geometry_msgs::msg::Point>("/planner/waypoint", 10);

//     decision_sub = this->create_subscription<gui_kfs_msgs::msg::KFSDecision>("/kfs_decision", 10,
//                                                                              std::bind(&AStarTargetNode::decision_callback, this, std::placeholders::_1));

//     detection_yolo_sub = this->create_subscription<mehua_pkg_msgs::msg::KFSDetectionArray>("/kfs_detections", 10,
//                                                                                            std::bind(&AStarTargetNode::kfs_callback, this, std::placeholders::_1));

//     timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&AStarTargetNode::publish_waypoint, this));

//     grid_rows = 4;
//     grid_cols = 3;
//     grid_.resize(grid_rows, std::vector<int>(grid_cols, 0));
//     kfs_r1_detected.resize(grid_rows, std::vector<bool>(grid_cols, false));
//     CELL_SIZE = 1.0;

//     robot_x = 0.0;
//     robot_y = 0.0;

//     // height_map = {
//     //   {0.4, 0.2, 0.4},
//     //   {0.2, 0.4, 0.6},
//     //   {0.4, 0.6, 0.4},
//     //   {0.2, 0.4, 0.2}
//     // };

//   }

//   private:
//   // rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr chase_sub;
//   // rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr avoid_sub;
//   rclcpp::Subscription<gui_kfs_msgs::msg::KFSDecision>::SharedPtr decision_sub;
//   rclcpp::Subscription<mehua_pkg_msgs::msg::KFSDetectionArray>::SharedPtr detection_yolo_sub;
//   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
//   rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr waypoint_pub;
//   rclcpp::TimerBase::SharedPtr timer_;

//   std::vector<std::vector<int>> grid_;
//   std::vector<std::vector<bool>> kfs_r1_detected;
//   std::vector<std::vector<double>>height_map;

//   int grid_rows, grid_cols;
//   double CELL_SIZE;

//   Waypoint current_target;
//   bool has_active_target = false;

//   double startX, startY;
//   double currentX, currentY;
//   double robot_x, robot_y;
//   bool start_received = false;

//   std::vector<Waypoint> path_;
//   size_t current_wp_ = 0;

//   std::queue<Waypoint> target_queue_;

//   void index_to_grid(int index, int &gx, int &gy)
//   {
//     // int cols = grid_cols;

//     gx = index % grid_cols;
//     gy = index / grid_cols;

//     gx = (grid_cols - 1) - gx;
//     // gy = (grid_rows - 1) - gy;

//     RCLCPP_INFO(this->get_logger(), "IDX %d → (%d,%d)", index, gx, gy);
//   }

//   void update_kfs_r1_grid(const mehua_pkg_msgs::msg::KFSDetectionArray::SharedPtr msg)
//   {
//     for (auto &row : kfs_r1_detected)
//       std::fill(row.begin(), row.end(), false);

//     for (const auto &det : msg->detections)
//     {
//       if (det.kfs_type != 1)
//         continue;

//       int gx, gy;
//       index_to_grid(det.id, gx, gy);

//       if (gx >= 0 && gx < grid_cols && gy >= 0 && gy < grid_rows)
//         kfs_r1_detected[gy][gx] = true;
//     }
//   }

//   bool path_constain_blocked_wp()
//   {
//     for (size_t i = current_wp_; i < path_.size(); i++)
//     {
//       auto &wp = path_[i];
//       if (kfs_r1_detected[wp.y][wp.x])
//         return true;
//     }

//     return false;
//   }

//   bool decision_received = false;
//   bool decision_processed = false;

//   void decision_callback(const gui_kfs_msgs::msg::KFSDecision::SharedPtr msg)
//   {
//     if (decision_processed)
//     {
//       RCLCPP_WARN(this->get_logger(), "Decision ignored (already processed)");
//       return;
//     }

//     decision_received = true;
//     decision_processed = true;

//     path_.clear();
//     current_wp_ = 0;
//     exit_sent = false;
//     has_active_target = false;

//     for (auto &row : grid_)
//       std::fill(row.begin(), row.end(), 0);

//     while(!target_queue_.empty())
//       target_queue_.pop();

//     for (auto idx : msg->avoid_targets)
//     {
//       int gx, gy;
//       index_to_grid(idx, gx, gy);
//       if (gx >= 0 && gx < grid_cols && gy >= 0 && gy < grid_rows)
//         grid_[gy][gx] = 1;
//     }

//     for (int idx : msg->chase_targets)
//     {
//       int gx, gy;
//       index_to_grid(idx, gx, gy);
//       // target_queue_.push({gx, gy});

//       if( gx >= 0 && gx < grid_cols && gy >= 0 && gy < grid_rows)
//       {
//         target_queue_.push({gx,gy});
//         RCLCPP_INFO(this->get_logger(), "push target (%d,%d)", gx, gy);
//       }
//     }

//     RCLCPP_INFO(this->get_logger(),"Queue size: %ld", target_queue_.size());

//     generate_path_to_target();
//   }

//   int choose_nearest_exit(int robot_x, int robot_y)
//   {
//     std::vector<int> exits = {9, 10, 11};

//     int best_idx = exits[0];
//     int best_dist = 999;

//     for (int idx : exits)
//     {
//       int gx, gy;
//       index_to_grid(idx, gx, gy);

//       int dist = std::abs(gx - robot_x) + std::abs(gy - robot_y);

//       if (dist < best_dist)
//       {
//         best_dist = dist;
//         best_idx = idx;
//       }
//     }

//     return best_idx;
//   }

//   void kfs_callback(const mehua_pkg_msgs::msg::KFSDetectionArray::SharedPtr msg)
//   {
//     update_kfs_r1_grid(msg);

//     if (path_constain_blocked_wp())
//       generate_path_to_target();
//   }

//   void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
//   {

//     currentX = msg->pose.pose.position.x;
//     currentY = msg->pose.pose.position.y;
//     robot_x = currentX;
//     robot_y = currentY;

//     if (!start_received)
//     {
//       startX = currentX;
//       startY = currentY;
//       start_received = true;
//     }
//   }

//   void generate_path_to_target()
//   {
//     if (!decision_received || !start_received)
//       return;

//     // RCLCPP_INFO(this->get_logger(), "Receive target decision from guiii.........");

//     // if (!start_received)
//     //   return;

//     if (target_queue_.empty())
//     {
//       RCLCPP_INFO(this->get_logger(), "No target Availablee");
//       return;
//     }

//     // Waypoint target = target_queue_.front();
//     // target_queue_.pop();

//     current_target = target_queue_.front();
//     has_active_target = true;

//     int start_x = std::clamp((int)std::round(robot_x / CELL_SIZE),0,grid_cols -1);
//     int start_y = std::clamp((int)std::round(robot_y / CELL_SIZE),0,grid_rows -1);

//     auto new_path = a_star_search(start_x, start_y, current_target.x, current_target.y);

//     if(!new_path.empty())
//     {
//       path_ = new_path;
//       current_wp_ = 0;
//       target_queue_.pop();
//     }
//     else
//     {
//       RCLCPP_WARN(this->get_logger(), "Path gagal, target tetap di queue");

//     }
//     // path_ = a_star_search(start_x, start_y, target.x, target.y);
//     // current_wp_ = 0;
//     RCLCPP_INFO(this->get_logger(), "Generated path to (%d,%d) with %lu waypoints", current_target.x, current_target.y, path_.size());

//     RCLCPP_INFO(this->get_logger(), "Generate path i panggil");
//     RCLCPP_INFO(this->get_logger(), "Start: %d %d", start_x, start_y);
//     RCLCPP_INFO(this->get_logger(), "Path size: %ld", path_.size());

//     // RCLCPP_INFO(this->get_logger(), "Target: %d %d", target_x, target_y);
//   }

//   std::vector<Waypoint> a_star_search(int start_x, int start_y, int goal_x, int goal_y)
//   {

//     std::priority_queue<AStarNode *, std::vector<AStarNode *>, NodeCmp> open_list;
//     std::vector<std::vector<bool>> closed(grid_rows, std::vector<bool>(grid_cols, false));

//     std::vector<Waypoint> result;
//     auto heuristic = [](int x1, int y1, int x2, int y2)
//     { return std::abs(x1 - x2) + std::abs(y1 - y2); };

//     // auto valid = [&](int x, int y)
//     // { return x >= 0 && x < grid_cols && y >= 0 && y < grid_rows && grid_[y][x] == 0; };

//     auto valid = [&](int x, int y)
//     {
//       if (x < 0 || x >= grid_cols || y < 0 || y >= grid_rows)
//         return false;

//       if (grid_[y][x] != 0)
//         return false;

//       if (kfs_r1_detected[y][x])
//         return false;

//       return true;
//     };

//     AStarNode *start = new AStarNode(start_x, start_y);
//     start->g = 0;
//     start->h = heuristic(start_x, start_y, goal_x, goal_y);
//     start->f = start->g + start->h;
//     open_list.push(start);
//     AStarNode *goal_node = nullptr;

//     while (!open_list.empty())
//     {
//       AStarNode *current = open_list.top();
//       open_list.pop();
//       if (closed[current->y][current->x])
//       {
//         delete current;
//         continue;
//       }
//       closed[current->y][current->x] = true;

//       if (current->x == goal_x && current->y == goal_y)
//       {
//         goal_node = current;
//         break;
//       }

//       std::vector<std::pair<int, int>> neighbors = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};
//       for (auto &n : neighbors)
//       {
//         int nx = current->x + n.first;
//         int ny = current->y + n.second;
//         if (valid(nx, ny) && !closed[ny][nx])
//         {
//           AStarNode *neighbors = new AStarNode(nx, ny);
//           neighbors->g = current->g + 1;
//           neighbors->h = heuristic(nx, ny, goal_x, goal_y);
//           neighbors->f = neighbors->g + neighbors->h;
//           neighbors->parent = current;
//           open_list.push(neighbors);
//         }
//       }
//     }

//     if (goal_node)
//     {
//       AStarNode *n = goal_node;
//       while (n)
//       {
//         result.push_back({n->x, n->y});
//         n = n->parent;
//       }
//       std::reverse(result.begin(), result.end());
//     }
//     return result;
//   }

//   bool exit_sent = false;

//   void publish_waypoint()
//   {

//     if(path_.empty())
//       return;

//     if (current_wp_ >= path_.size())
//     {
//       // generate_path_to_target();
//       // return;

//       if(has_active_target)
//       {
//         target_queue_.pop();
//         has_active_target = false;
//       }

//       if(target_queue_.empty())
//       {
//         generate_path_to_target();
//         return;
//       }

//       if (!exit_sent)
//       {
//         int robot_grid_x = std::round(robot_x / CELL_SIZE);
//         int robot_grid_y = std::round(robot_y / CELL_SIZE);

//         int exit_idx = choose_nearest_exit(robot_grid_x, robot_grid_y);

//         int gx, gy;
//         index_to_grid(exit_idx, gx, gy);

//         target_queue_.push({gx, gy});

//         exit_sent = true;

//         RCLCPP_INFO(this->get_logger(), "semua target selesai , menuju exit");

//         generate_path_to_target();
//         return;
//       }

//       RCLCPP_INFO(this->get_logger(),"robot keluar dari mehua forest");
//     }
//     Waypoint wp = path_[current_wp_];
//     geometry_msgs::msg::Point msg;
//     msg.x = wp.x * CELL_SIZE;
//     msg.y = wp.y * CELL_SIZE;
//     msg.z = 0.0;
//     waypoint_pub->publish(msg);

//     current_wp_++;
//   }
// };

// int main(int argc, char **argv)
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<AStarTargetNode>());
//   rclcpp::shutdown();
//   return 0;
// }
