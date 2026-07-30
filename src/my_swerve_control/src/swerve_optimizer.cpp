#include <cmath>
#include <vector>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class SwerveTestNode : public rclcpp::Node {
public:
    SwerveTestNode() : Node("swerve_test_node") {
        joint_names_ = {
            "front_left_steer_joint", "mid_left_steer_joint", "back_left_steer_joint",
            "front_right_steer_joint", "mid_right_steer_joint", "back_right_steer_joint"
        };
        
        wheel_coords_x = {0.3, 0.0, -0.3, 0.3, 0.0, -0.3};
        wheel_coords_y = {0.2, 0.2, 0.2, -0.2, -0.2, -0.2};
        wheel_radius_ = 0.05;

        // Both publishers now use Float64MultiArray!
        steer_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/swerve_steer_controller/commands", 10);
        drive_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/swerve_drive_controller/commands", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), std::bind(&SwerveTestNode::kinematics_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "Swerve Node: Publishing RAW Positions and Velocities...");
    }

private:
    void kinematics_loop() {
        float Vx = 0.2, Vy = 0.2, omega = 0.0; 
        float px = 0.0, py = 0.0;

        auto drive_msg = std_msgs::msg::Float64MultiArray();
        auto steer_msg = std_msgs::msg::Float64MultiArray();

        for (size_t i = 0; i < joint_names_.size(); i++) {
            float dx = wheel_coords_x[i] - px;
            float dy = wheel_coords_y[i] - py;

            float vx = Vx - omega * dy;
            float vy = Vy + omega * dx;

            double target_angle = atan2(vy, vx);
            double linear_velocity = sqrt(vx * vx + vy * vy);
            double angular_motor_speed = linear_velocity / wheel_radius_;

            steer_msg.data.push_back(target_angle);
            drive_msg.data.push_back(angular_motor_speed);
        }

        steer_pub_->publish(steer_msg);
        drive_pub_->publish(drive_msg);
    }

    std::vector<std::string> joint_names_;
    std::vector<double> wheel_coords_x;
    std::vector<double> wheel_coords_y;
    double wheel_radius_;
    
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr steer_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr drive_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SwerveTestNode>());
    rclcpp::shutdown();
    return 0;
}