#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "gui_kfs_msgs/msg/kfs_decision.hpp"
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>

struct Waypoint
{
  int x;
  int y;
};

struct AStarNode
{
  int x, y;
  double f, g, h;
  AStarNode *parent;
  AStarNode(int x_, int y_) : x(x_), y(y_), f(0), g(0), h(0), parent(nullptr) {}
};

struct NodeCmp
{
  bool operator()(AStarNode *a, AStarNode *b) { return a->f > b->f; }
};

class AStarTargetNode : public rclcpp::Node
{
public:
  AStarTargetNode() : Node("a_star_target_node")
  {
    // chase_sub = this->create_subscription<geometry_msgs::msg::Point>("/chase_from_backend", 10,
    //                                                                  std::bind(&AStarTargetNode::chase_callback, this, std::placeholders::_1));

    // avoid_sub = this->create_subscription<geometry_msgs::msg::Point>("/avoid_from_backend", 10,
    //                                                                  std::bind(&AStarTargetNode::avoid_callback, this, std::placeholders::_1));

    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10,
                                                                  std::bind(&AStarTargetNode::odom_callback, this, std::placeholders::_1));

    waypoint_pub = this->create_publisher<geometry_msgs::msg::Point>("/planner/waypoint", 10);

    decision_sub = this->create_subscription<gui_kfs_msgs::msg::KFSDecision>("/kfs_decision", 10,
                                                                            std::bind(&AStarTargetNode::decision_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&AStarTargetNode::publish_waypoint, this));

    grid_rows = 4;
    grid_cols = 3;
    grid_.resize(grid_rows, std::vector<int>(grid_cols, 0));
    CELL_SIZE = 1.0;

    robot_x = 0.0;
    robot_y = 0.0;
  }

private:
  // rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr chase_sub;
  // rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr avoid_sub;
  rclcpp::Subscription<gui_kfs_msgs::msg::KFSDecision>::SharedPtr decision_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr waypoint_pub;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<std::vector<int>> grid_;
  int grid_rows, grid_cols;
  double CELL_SIZE;

  double startX, startY;
  double currentX, currentY;
  double robot_x, robot_y;
  bool start_received = false;

  std::vector<Waypoint> path_;
  size_t current_wp_ = 0;

  std::queue<Waypoint> target_queue_;
    
  void index_to_grid(int index, int &gx, int &gy)
  {
    gx = index % grid_cols;
    gy = index / grid_cols;
  }

  void decision_callback(const gui_kfs_msgs::msg::KFSDecision::SharedPtr msg)
  {
    for (auto &row : grid_)
      std::fill(row.begin(), row.end(), 0);

    for (auto idx : msg->avoid_targets)
    {
      int gx,gy;
      index_to_grid(idx,gx,gy);
      if(gx >= 0 && gx < grid_cols && gy >= 0 && gy < grid_rows)
        grid_[gy][gx] = 1;
    }

    while(!target_queue_.empty())
      target_queue_.pop();

    for(int idx : msg->chase_targets)
    {
      int gx,gy;
      index_to_grid(idx,gx,gy);
      target_queue_.push({gx, gy});

    }

    generate_path_to_target();
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {

    currentX = msg->pose.pose.position.x;
    currentY = msg->pose.pose.position.y;
    robot_x = currentX;
    robot_y = currentY;

    if (!start_received)
    {
      startX = currentX;
      startY = currentY;
      start_received = true;
    }
  }


  void generate_path_to_target()
  {
    if (target_queue_.empty())
      return;

    if (!start_received)
      return;

    Waypoint target = target_queue_.front();
    target_queue_.pop();

    int start_x = std::round(robot_x / CELL_SIZE);
    int start_y = std::round(robot_y / CELL_SIZE);

    path_ = a_star_search(start_x, start_y, target.x, target.y);
    current_wp_ = 0;
    RCLCPP_INFO(this->get_logger(), "Generated path to (%d,%d) with %lu waypoints", target.x, target.y, path_.size());
  }

  std::vector<Waypoint> a_star_search(int start_x, int start_y, int goal_x, int goal_y)
  {

    std::priority_queue<AStarNode *, std::vector<AStarNode *>, NodeCmp> open_list;
    std::vector<std::vector<bool>> closed(grid_rows, std::vector<bool>(grid_cols, false));

    std::vector<Waypoint> result;
    auto heuristic = [](int x1, int y1, int x2, int y2)
    { return std::abs(x1 - x2) + std::abs(y1 - y2); };
    auto valid = [&](int x, int y)
    { return x >= 0 && x < grid_cols && y >= 0 && y < grid_rows && grid_[y][x] == 0; };

    AStarNode *start = new AStarNode(start_x, start_y);
    start->g = 0;
    start->h = heuristic(start_x, start_y, goal_x, goal_y);
    start->f = start->g + start->h;
    open_list.push(start);
    AStarNode *goal_node = nullptr;

    while (!open_list.empty())
    {
      AStarNode *current = open_list.top();
      open_list.pop();
      if (closed[current->y][current->x])
      {
        delete current;
        continue;
      }
      closed[current->y][current->x] = true;

      if (current->x == goal_x && current->y == goal_y)
      {
        goal_node = current;
        break;
      }

      std::vector<std::pair<int, int>> neighbors = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};
      for (auto &n : neighbors)
      {
        int nx = current->x + n.first;
        int ny = current->y + n.second;
        if (valid(nx, ny) && !closed[ny][nx])
        {
          AStarNode *neighbors = new AStarNode(nx, ny);
          neighbors->g = current->g + 1;
          neighbors->h = heuristic(nx, ny, goal_x, goal_y);
          neighbors->f = neighbors->g + neighbors->h;
          neighbors->parent = current;
          open_list.push(neighbors);
        }
      }
    }

    if (goal_node)
    {
      AStarNode *n = goal_node;
      while (n)
      {
        result.push_back({n->x, n->y});
        n = n->parent;
      }
      std::reverse(result.begin(), result.end());
    }
    return result;
  }

  void publish_waypoint()
  {
    if (current_wp_ >= path_.size())
    {
      generate_path_to_target();
      return;
    }

    Waypoint wp = path_[current_wp_];
    geometry_msgs::msg::Point msg;
    msg.x = wp.x * CELL_SIZE;
    msg.y = wp.y * CELL_SIZE;
    msg.z = 0.0;
    waypoint_pub->publish(msg);

    current_wp_++;
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AStarTargetNode>());
  rclcpp::shutdown();
  return 0;
}
