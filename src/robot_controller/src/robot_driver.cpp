#include <chrono>
#include <cmath>
#include <memory>
#include <algorithm>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

// Robot Dimensions
const double L1 = 0.5;
const double L2 = 0.4;

// Speed Settings
const double MAX_SPEED_RAD = 1.0;
const double MAX_SPEED_LIN = 0.5;
const double MAX_SPEED_GRIPPER = 0.05; // 5cm/s finger speed
const double TIMER_PERIOD = 0.05; 

class RobotDriver : public rclcpp::Node
{
public:
    RobotDriver() : Node("robot_driver_node")
    {
        // Subscribe to commands (Pose + Gripper info inside)
        command_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/scara_commands", 10, std::bind(&RobotDriver::command_callback, this, std::placeholders::_1));

        joint_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(TIMER_PERIOD * 1000)),
            std::bind(&RobotDriver::timer_callback, this));

        // Initialize Joints (0-3: Arm, 4: Gripper)
        current_joints_ = {0.0, 0.0, 0.0, 0.0, 0.0};
        target_joints_  = {0.0, 0.0, 0.0, 0.0, 0.0};

        RCLCPP_INFO(this->get_logger(), "Robot Driver Ready. Controlling Arm + Gripper.");
    }

private:
    struct JointConfig { double shoulder, elbow, quill, theta, gripper; };
    struct CartesianPose { double x, y, z, yaw, gripper_cmd; };

    JointConfig current_joints_;
    JointConfig target_joints_;  

    void command_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        CartesianPose target_pose;
        target_pose.x = msg->pose.position.x;
        target_pose.y = msg->pose.position.y;
        target_pose.z = msg->pose.position.z;

        // --- EXTRACT GRIPPER COMMAND ---
        // Read the value hidden in orientation.x (0.0 or 1.0)
        target_pose.gripper_cmd = msg->pose.orientation.x; 

        // Extract Yaw
        tf2::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
        double r, p, yaw;
        tf2::Matrix3x3(q).getRPY(r, p, yaw);
        target_pose.yaw = yaw;

        JointConfig solution;
        if (solve_ik(target_pose, solution)) {
            target_joints_ = solution; 
            RCLCPP_INFO(this->get_logger(), "New Target. Gripper Cmd: %.1f", target_pose.gripper_cmd);
        } else {
            RCLCPP_WARN(this->get_logger(), "Target Unreachable!");
        }
    }

    void timer_callback()
    {
        // 1. Interpolate Arm Joints
        move_joint(current_joints_.shoulder, target_joints_.shoulder, MAX_SPEED_RAD);
        move_joint(current_joints_.elbow,    target_joints_.elbow,    MAX_SPEED_RAD);
        move_joint(current_joints_.quill,    target_joints_.quill,    MAX_SPEED_LIN);
        move_joint(current_joints_.theta,    target_joints_.theta,    MAX_SPEED_RAD);
        
        // 2. Interpolate Gripper (Smoothly open/close)
        // Convert 0.0/1.0 command to physical meters (0.03m max travel)
        double gripper_target_m = target_joints_.gripper * 0.03; 
        move_joint(current_joints_.gripper, gripper_target_m, MAX_SPEED_GRIPPER);

        // 3. Publish Everything
        CartesianPose visual_pose = solve_fk(current_joints_);
        publish_state(current_joints_, visual_pose);
    }

    void move_joint(double &current, double target, double speed_limit)
    {
        double step = speed_limit * TIMER_PERIOD;
        double diff = target - current;

        if (std::abs(diff) <= step) {
            current = target;
        } else {
            current += (diff > 0 ? step : -step); 
        }
    }

    bool solve_ik(const CartesianPose& target, JointConfig& joints) {
        joints.quill = target.z;
        joints.gripper = target.gripper_cmd; // Pass command directly to joint config

        double r_sq = target.x * target.x + target.y * target.y;
        double C2 = (r_sq - L1*L1 - L2*L2) / (2 * L1 * L2);
        if (C2 < -1.0 || C2 > 1.0) return false;
        double S2 = std::sqrt(1 - C2*C2);
        joints.elbow = std::atan2(S2, C2);
        double k1 = L1 + L2 * C2;
        double k2 = L2 * S2;
        joints.shoulder = std::atan2(target.y, target.x) - std::atan2(k2, k1);
        joints.theta = target.yaw - joints.shoulder - joints.elbow;
        return true;
    }

    CartesianPose solve_fk(const JointConfig& joints) {
        CartesianPose pose;
        pose.x = L1 * cos(joints.shoulder) + L2 * cos(joints.shoulder + joints.elbow);
        pose.y = L1 * sin(joints.shoulder) + L2 * sin(joints.shoulder + joints.elbow);
        pose.z = joints.quill;
        pose.yaw = joints.shoulder + joints.elbow + joints.theta;
        return pose;
    }

    void publish_state(const JointConfig& joints, const CartesianPose& pose) {
        sensor_msgs::msg::JointState js;
        js.header.stamp = this->get_clock()->now();
        
        // PUBLISH ALL JOINTS (Arm + 2 Fingers)
        js.name = {"shoulder_joint", "elbow_joint", "quill_joint", "theta_joint", "left_finger_joint", "right_finger_joint"};
        
        // Use the interpolated gripper value for both fingers
        js.position = {joints.shoulder, joints.elbow, joints.quill, joints.theta, joints.gripper, joints.gripper};
        
        joint_publisher_->publish(js);

        // Publish Transform
        geometry_msgs::msg::TransformStamped ts;
        ts.header.stamp = this->get_clock()->now();
        ts.header.frame_id = "odom";
        ts.child_frame_id = "axis";
        ts.transform.translation.x = pose.x;
        ts.transform.translation.y = pose.y;
        ts.transform.translation.z = pose.z;
        tf2::Quaternion q;
        q.setRPY(0, 0, pose.yaw);
        ts.transform.rotation.x = q.x();
        ts.transform.rotation.y = q.y();
        ts.transform.rotation.z = q.z();
        ts.transform.rotation.w = q.w();
        broadcaster_->sendTransform(ts);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr command_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publisher_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotDriver>());
    rclcpp::shutdown();
    return 0;
}