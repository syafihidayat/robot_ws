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

    reached_sub = this->create_subscription<std_msgs::msg::Bool>(
      "/target_reached", 10,
      std::bind(&waypointPublish::reached_callback, this, std::placeholders::_1));

    ir_sub = this->create_subscription<std_msgs::msg::Bool>(
      "/infraReceive", 10,
      std::bind(&waypointPublish::ir_callback, this, std::placeholders::_1));

    lifter_pub = this->create_publisher<std_msgs::msg::Bool>("lifter_control", 10);

    toStage2_pub = this->create_publisher<std_msgs::msg::Bool>("toStage2", 10);

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
  bool stage2_sent = false;

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
    if (current_waypoint_index == 1)
    {
      state = WP_STATE::WAIT_IR;
      waiting_ir = true;

      RCLCPP_INFO(this->get_logger(), "WAITING IR...");
      return; 
    }

    // if(current_waypoint_index == 2 && !stage2_sent)
    // {

    //   stage2_sent = true;

    //   std_msgs::msg::Bool stage2_msg;
    //   stage2_msg.data = true;
    //   toStage2_pub->publish(stage2_msg);

    //   RCLCPP_INFO(this->get_logger(), "ENTRY STAGE2 - CHANGE SPEED");

    // }

    // WP lainnya langsung lanjut
    advance_waypoint();
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
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lifter_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr toStage2_pub;

  std::vector<std::pair<double, double>> waypoint;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<waypointPublish>());
  rclcpp::shutdown();
  return 0;
}


// class waypointPublish : public rclcpp::Node
// {
// public:
//   waypointPublish() : Node("send_pose")
//   {

//     pose_pub = this->create_publisher<geometry_msgs::msg::Point>("/pose", 10);

//     reached_sub = this->create_subscription<std_msgs::msg::Bool>("/target_reached", 10,
//       std::bind(&waypointPublish::reached_callback, this, std::placeholders::_1));

//     lifter_pub = this->create_publisher<std_msgs::msg::Bool>("lifter_control", 10);

//     ir_sub = this->create_subscription<std_msgs::msg::Bool>("/infraReceive", 10,
//     std::bind(&waypointPublish::ir_callback, this,std::placeholders::_1));

//     toStage2_pub = this->create_publisher<std_msgs::msg::Bool>("/toStage2", 10);

//     // timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&waypointPublish::control, this));

//     waypoint= {

//       {0.2 , 1.0},
//       {1.1 , 1.0},
//       {2.0 , -0.4}
//     };

//     current_waypoint_index = 0;
//     waypoint_sent = false;
//     all_complated = false;

//     RCLCPP_INFO(this->get_logger(), "waypoint publisher start");
//     send_waypoint();

//   }

//   private:

//   bool ir_received = false;
//   bool last_reached = false;
//   bool prev_reaached = false;
//   // bool waiting_ack = false;
//   bool waypoint_sent;
//   bool all_complated;



//   void reached_callback(const std_msgs::msg::Bool::SharedPtr msg)
//   {
//     if(msg->data && !prev_reaached && !all_complated)
//     // if(msg->data && !all_complated && !last_reached)
//     {
//       RCLCPP_INFO(this->get_logger(),"waypoint reached");

//       if(current_waypoint_index == 0)
//       {
//         std_msgs::msg::Bool lifter_msg;
//         lifter_msg.data = true;
//         lifter_pub->publish(lifter_msg);

//         RCLCPP_INFO(this->get_logger(), "Trigger lifter turun");
//       }

//       if(current_waypoint_index == 1)
//       {
//         // ir_received = false;
//         RCLCPP_INFO(this->get_logger(), "Waiting IR before continue...");
//         return;
//       }

//       // if(current_waypoint_index == 2)
//       // {
//       //   std_msgs::msg::Bool toStage2_msg;
//       //   toStage2_msg.data = true;
//       //   toStage2_pub->publish(toStage2_msg);

//       //   RCLCPP_INFO(this->get_logger(), "To stage 2 change speed");

//       // }

//       current_waypoint_index++;
//       // ir_received = false;

//       if(current_waypoint_index < waypoint.size()){
//         send_waypoint();
//       }else{
//         all_complated = true;
//         RCLCPP_INFO(this->get_logger(), "ALL WAYPOINTS COMPLETE");
//       }

//     }

//     prev_reaached = msg->data;
//   }


//   void ir_callback(const std_msgs::msg::Bool::SharedPtr msg)
//   {
//     if(msg->data && current_waypoint_index == 1 && !ir_received)
//     {
//       ir_received = true;
//       // last_reached = false;
//       RCLCPP_INFO(this->get_logger(), "IR received");

//       // current_waypoint_index++;

//       // if(current_waypoint_index < waypoint.size())
//       // {
//       //   send_waypoint();
//       // }else{
//       //   all_complated = true;
//       //   RCLCPP_INFO(this->get_logger(), "ALL_COMPLATED");
//       // }

//     }
//   }


//   void send_waypoint(){
//     if(current_waypoint_index < waypoint.size()){

//       geometry_msgs::msg::Point point;
//       point.x = waypoint[current_waypoint_index].first;
//       point.y = waypoint[current_waypoint_index].second;
//       point.z = 0.0;

//       pose_pub->publish(point);
//       waypoint_sent = true;
//     }
//   }

//   rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pose_pub;
//   rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reached_sub;
//   rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lifter_pub;
//   rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr toStage2_pub;
//   rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ir_sub;
//   rclcpp::TimerBase::SharedPtr timer_;
//   std::vector<std::pair<double, double>>waypoint;
//   size_t current_waypoint_index;
// };

// int main(int argc, char **argv)
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<waypointPublish>());
//   rclcpp::shutdown();
//   return 0;
// }



// #include <rclcpp/rclcpp.hpp>
// #include <std_msgs/msg/bool.hpp>
// #include <geometry_msgs/msg/point.hpp>
// #include <cmath>

// class waypointPublish : public rclcpp::Node
// {
// public:
//   waypointPublish() : Node("send_pose")
//   {

//     pose_pub = this->create_publisher<geometry_msgs::msg::Point>("/pose", 10);

//     reached_sub = this->create_subscription<std_msgs::msg::Bool>("/target_reached", 10,
//     std::bind(&waypointPublish::reached_callback, this, std::placeholders::_1));

//     limit_sub = this->create_subscription<std_msgs::msg::Bool>("/limitdata", 10,
//     std::bind(&waypointPublish::limit_callback, this, std::placeholders::_1));

//     timer_ = this->create_wall_timer(std::chrono::milliseconds(100),std::bind(&waypointPublish::check_progress, this));

//     waypoint = {
  
//         // {0.2, 0.9},
//         // {1.0, 0.9}

//         // {0.0 , 1.0}
//         {0.5, 0.9},
//         {0.9, 0.9}

//     };

//     current_waypoint_index = 0;
//     waypoint_sent = false;
//     all_complated = false;
    
//     RCLCPP_INFO(this->get_logger(), "waypoint publisher start");
//     send_waypoint();
//   }
  
//   private:
  
//   bool last_limit = false;
//   bool target_reached_flag = false;
//   bool limit_triggered = false;

//   void reached_callback(const std_msgs::msg::Bool::SharedPtr msg)
//   {

//     if(msg->data)
//     {
//       target_reached_flag = true;
//     }
//     // if ((msg->data  || limit_triggered )&& !all_complated)
//     // {
//     //   RCLCPP_INFO(this->get_logger(), "waypoint reached(odom / limit)");

//     //   current_waypoint_index++;

//     //   limit_triggered = true;

//     //   if (current_waypoint_index < waypoint.size())
//     //   {
//     //     send_waypoint();
//     //   }
//     //   else
//     //   {
//     //     all_complated = true;
//     //     RCLCPP_INFO(this->get_logger(), "ALL WAYPOINTS COMPLETE");
//     //   }
//     // }
//   }

  
//   void limit_callback(const std_msgs::msg::Bool::SharedPtr msg)
//   {
//     if(msg->data != last_limit)
//     {
//       RCLCPP_INFO(this->get_logger(),msg->data ? "LIMIT TOUCHED" : "LIMIT CLEARED");
      
//       if(msg->data)
//       {
//         limit_triggered = true;
//       }
//     }
//     last_limit = msg->data;
//   }

//   void check_progress()
//   {
//     if(all_complated)
//       return;

//     if(target_reached_flag || limit_triggered)
//     {

//       RCLCPP_INFO(this->get_logger(), "waypoint reached(odom / limit)");

//       current_waypoint_index++;
      
//       limit_triggered = true;
//       target_reached_flag = false;
      
//       if (current_waypoint_index < waypoint.size())
//       {
//         send_waypoint();
//       }
//       else
//       {
//         all_complated = true;
//         RCLCPP_INFO(this->get_logger(), "ALL WAYPOINTS COMPLETE");
//       }
//     }
//   }

//   void send_waypoint()
//   {
//     if (current_waypoint_index < waypoint.size())
//     {
//       geometry_msgs::msg::Point point;
//       point.x = waypoint[current_waypoint_index].first;
//       point.y = waypoint[current_waypoint_index].second;
//       point.z = 0.0;

//       pose_pub->publish(point);
//       waypoint_sent = true;
//     }
//   }
  
//   rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pose_pub;
//   rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reached_sub;
//   rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr limit_sub;
//   rclcpp::TimerBase::SharedPtr timer_;
//   std::vector<std::pair<double, double>> waypoint;
//   size_t current_waypoint_index;
//   bool waypoint_sent;
//   bool all_complated;
// };

// int main(int argc, char **argv)
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<waypointPublish>());
//   rclcpp::shutdown();
//   return 0;
// }