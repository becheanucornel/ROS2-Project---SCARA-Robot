#include <chrono>
#include <fstream>
#include <memory>
#include <vector>
#include <string>
#include <iostream>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nlohmann/json.hpp"

using json = nlohmann::json;
using namespace std::chrono_literals;

struct Waypoint {
    // Initialising variables
    geometry_msgs::msg::PoseStamped pose;
    double gripper_val;
};

class MotionPublisher : public rclcpp::Node
{
public:
    MotionPublisher() : Node("motion_publisher_node")
    {
        // Defining node parameters
        this->declare_parameter("waypoint_file", "");

        // Node publisher
        arm_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/scara_commands", 10);

        load_positions_from_file();

        timer_ = this->create_wall_timer(
            3000ms, 
            std::bind(&MotionPublisher::timer_callback, this));
    }

private:
    std::vector<Waypoint> mission_queue_;
    size_t current_step_ = 0;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr arm_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    void load_positions_from_file()
    {
        std::string file_path = this->get_parameter("waypoint_file").as_string();

        if (file_path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No 'waypoint_file' parameter provided!");
            return;
        }

        std::ifstream f(file_path);
        if (!f.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open file!");
            return;
        }

        try {
            json data = json::parse(f);

            for (const auto& item : data) {
                Waypoint wp;
                wp.pose.header.frame_id = "odom";
                wp.pose.pose.position.x = item.value("x", 0.9);
                wp.pose.pose.position.y = item.value("y", 0.0);
                wp.pose.pose.position.z = item.value("z", 0.2);
                
                double yaw = item.value("yaw", 0.0);
                wp.pose.pose.orientation.z = sin(yaw / 2.0);
                wp.pose.pose.orientation.w = cos(yaw / 2.0);

                wp.gripper_val = item.value("gripper", 0.0);

                mission_queue_.push_back(wp);
            }
            RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints.", mission_queue_.size());

        } catch (const json::parse_error& e) {
            RCLCPP_ERROR(this->get_logger(), "JSON Parse Error: %s", e.what());
        }
    }

    void timer_callback()
    {
        if (mission_queue_.empty()) return;

        if (current_step_ >= mission_queue_.size()) {
            RCLCPP_INFO(this->get_logger(), "Mission Complete. Restarting...");
            current_step_ = 0;
        }

        Waypoint wp = mission_queue_[current_step_];
        wp.pose.header.stamp = this->get_clock()->now();
        wp.pose.pose.orientation.x = wp.gripper_val;

        arm_publisher_->publish(wp.pose);
        
        std::string grip_state = (wp.gripper_val > 0.5) ? "CLOSED" : "OPEN";
        RCLCPP_INFO(this->get_logger(), "Step [%zu]: Moving to (%.2f, %.2f) | Gripper: %s", 
            current_step_ + 1, 
            wp.pose.pose.position.x, wp.pose.pose.position.y,
            grip_state.c_str());

        current_step_++;
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotionPublisher>());
    rclcpp::shutdown();
    return 0;
}