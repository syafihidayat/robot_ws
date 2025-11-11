#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cmath>

class Mehua : public rclcpp::Node
{
public:
    Mehua() : Node("MF_Move")
    {
        next_step_sub = this->create_subscription<std_msgs::msg::Bool>("/next_step", 10,
                                                                       std::bind(&Mehua::next_step_callback, this, std::placeholders::_1));

        pose2_pub = this->create_publisher<geometry_msgs::msg::Point>("/pose_steps", 10);

        // timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&Movement::base_move, this));

        position_steps = {
            {0.0, 0.8}
            // {1.0, 0.5}
            // {0.0, 0.5},
            // {0.0, 0.0}
        };

        current_step = 0;
        steps_sent = false;
        all_complated = false;

        RCLCPP_INFO(this->get_logger(), "waiting stage 1 finish");

        publish_current_step();
    }

private:

    void next_step_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data && !all_complated)
        {
            RCLCPP_INFO(this->get_logger(), "step reached");

            current_step++;

            if (current_step < position_steps.size())
            {
                publish_current_step();
            }
            else
            {
                all_complated = true;
                RCLCPP_INFO(this->get_logger(), "ALL STEPS COMPALTE");
            }
        }
    }

    void publish_current_step()
    {
        if (current_step < position_steps.size())
        {
            geometry_msgs::msg::Point step;
            step.x = position_steps[current_step].first;
            step.y = position_steps[current_step].second;
            step.z = 0.0;

            pose2_pub->publish(step);
            steps_sent = true;
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pose2_pub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr next_step_sub;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::pair<double, double>>position_steps;
    size_t current_step;
    bool steps_sent;
    bool all_complated;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Mehua>());
    rclcpp::shutdown();
    return 0;
}