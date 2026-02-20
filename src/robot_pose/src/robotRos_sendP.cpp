#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cmath>


class waypointPublish : public rclcpp::Node
{
public:
  waypointPublish() : Node("send_pose")
  {

    pose_pub = this->create_publisher<geometry_msgs::msg::Point>("/pose", 10);

    reached_sub = this->create_subscription<std_msgs::msg::Bool>("/target_reached", 10,
      std::bind(&waypointPublish::reached_callback, this, std::placeholders::_1));

    // timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&waypointPublish::control, this));

    waypoint= {
      // {0.3 , 1.0},
      // {1.0 , 1.0}
      // {2.0 , -1.6}
      {1.0 , 0.0},
      {1.0 , 1.0}
    };

    current_waypoint_index = 0;
    waypoint_sent = false;
    all_complated = false;

    RCLCPP_INFO(this->get_logger(), "waypoint publisher start");
    send_waypoint();

  }

private:
/*
void callback(){
if(count== 0 && target available){
  tampung_odom = odom
  count =1
}
   if (fabs(yaw_error) > 0.001)
        {
            control_distance = 0.0; // stay in place while turning
            control_rotate = base_rotate.control_base_rotation(errorYaw, deltaT);
            std::cout << "turn" << std::endl;
        }
        else
        {
            control_distance = base_distance.control_base_distance(distance, deltaT);
        }
  error = target + tampung odom

}
*/
  void reached_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if(msg->data && !all_complated){
      RCLCPP_INFO(this->get_logger(),"waypoint reached");

      current_waypoint_index++;

      if(current_waypoint_index < waypoint.size()){
        send_waypoint();
      }else{
        all_complated = true;
        RCLCPP_INFO(this->get_logger(), "ALL WAYPOINTS COMPLETE");
      }
    }
  }


  void send_waypoint(){
    if(current_waypoint_index < waypoint.size()){
      geometry_msgs::msg::Point point;
      point.x = waypoint[current_waypoint_index].first;
      point.y = waypoint[current_waypoint_index].second;
      point.z = 0.0;

      pose_pub->publish(point);
      waypoint_sent = true;
    }
  }

  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pose_pub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reached_sub;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<std::pair<double, double>>waypoint;
  size_t current_waypoint_index;
  bool waypoint_sent;
  bool all_complated;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<waypointPublish>());
  rclcpp::shutdown();
  return 0;
}