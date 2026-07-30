#define _USE_MATH_DEFINES
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

// M_PI is POSIX, not standard C++. Fine under glibc, absent on MinGW/MSVC.
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

// Wrap an angle to (-pi, pi].
static double wrap_pi(double a)
{
    while (a > M_PI)   a -= 2.0 * M_PI;
    while (a <= -M_PI) a += 2.0 * M_PI;
    return a;
}

class SwerveControllerNode : public rclcpp::Node {
public:
    SwerveControllerNode() : Node("swerve_controller")
    {
        joint_names_ = {
            "front_left_steer_joint", "mid_left_steer_joint", "back_left_steer_joint",
            "front_right_steer_joint", "mid_right_steer_joint", "back_right_steer_joint"
        };

        wheel_coords_x_ = {0.3, 0.0, -0.3, 0.3, 0.0, -0.3};
        wheel_coords_y_ = {0.2, 0.2, 0.2, -0.2, -0.2, -0.2};

        wheel_radius_    = this->declare_parameter("wheel_radius", 0.05);
        pivot_x_         = this->declare_parameter("pivot_x", 0.0);
        pivot_y_         = this->declare_parameter("pivot_y", 0.0);
        max_motor_speed_ = this->declare_parameter("max_motor_speed", 20.0);  // rad/s
        cmd_timeout_     = this->declare_parameter("cmd_timeout", 0.5);       // s

        last_angle_.assign(joint_names_.size(), 0.0);

        steer_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/swerve_steer_controller/commands", 10);
        drive_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/swerve_drive_controller/commands", 10);

        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                cmd_ = *msg;
                last_cmd_time_ = this->now();
            });

        last_cmd_time_ = this->now();

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&SwerveControllerNode::kinematics_loop, this));

        RCLCPP_INFO(this->get_logger(),
                    "Swerve controller up: 6 modules, listening on /cmd_vel");
    }

private:
    // Choose the shorter way to reach a heading. If a module would have to swing
    // more than 90 degrees, point it the opposite way and drive the wheel
    // backwards instead -- the ground motion is identical and no module ever
    // rotates more than a quarter turn.
    void optimize_module(size_t i, double target_angle, double target_velocity,
                         double & out_angle, double & out_velocity) const
    {
        const double delta = wrap_pi(target_angle - last_angle_[i]);

        if (std::abs(delta) > M_PI_2) {
            out_angle    = wrap_pi(target_angle + M_PI);
            out_velocity = -target_velocity;
        } else {
            out_angle    = target_angle;
            out_velocity = target_velocity;
        }
    }

    // atan2(0, 0) is undefined and snaps a stopped module to zero heading. Hold
    // the last commanded angle instead, so the modules stay put when the robot
    // stops or when a module sits on the centre of rotation.
    void apply_singularity_hold(size_t i, double & angle, double & velocity) const
    {
        if (std::abs(velocity) <= kSpeedEpsilon) {
            angle    = last_angle_[i];
            velocity = 0.0;
        }
    }

    // If any module is asked for more than the motor can deliver, scale every
    // module down by the same factor. Clipping one module instead would change
    // the direction the robot actually travels.
    void normalise_speeds(std::vector<double> & speeds) const
    {
        double max_speed = 0.0;
        for (double s : speeds) max_speed = std::max(max_speed, std::abs(s));

        if (max_speed > max_motor_speed_) {
            const double scale = max_motor_speed_ / max_speed;
            for (double & s : speeds) s *= scale;
        }
    }

    void kinematics_loop()
    {
        // Fail safe to a stop if commands stop arriving.
        if ((this->now() - last_cmd_time_).seconds() > cmd_timeout_) {
            cmd_ = geometry_msgs::msg::Twist();
        }

        const double Vx    = cmd_.linear.x;
        const double Vy    = cmd_.linear.y;
        const double omega = cmd_.angular.z;

        const size_t n = joint_names_.size();
        std::vector<double> angles(n), speeds(n);

        for (size_t i = 0; i < n; i++) {
            const double dx = wheel_coords_x_[i] - pivot_x_;
            const double dy = wheel_coords_y_[i] - pivot_y_;

            // Velocity of this module in the body frame.
            const double vx = Vx - omega * dy;
            const double vy = Vy + omega * dx;

            const double raw_angle    = std::atan2(vy, vx);
            const double raw_velocity = std::hypot(vx, vy) / wheel_radius_;

            double angle, velocity;
            optimize_module(i, raw_angle, raw_velocity, angle, velocity);
            apply_singularity_hold(i, angle, velocity);

            angles[i] = angle;
            speeds[i] = velocity;
        }

        normalise_speeds(speeds);

        for (size_t i = 0; i < n; i++) last_angle_[i] = angles[i];

        auto steer_msg = std_msgs::msg::Float64MultiArray();
        auto drive_msg = std_msgs::msg::Float64MultiArray();
        steer_msg.data = angles;
        drive_msg.data = speeds;

        steer_pub_->publish(steer_msg);
        drive_pub_->publish(drive_msg);
    }

    static constexpr double kSpeedEpsilon = 1e-3;

    std::vector<std::string> joint_names_;
    std::vector<double> wheel_coords_x_;
    std::vector<double> wheel_coords_y_;
    std::vector<double> last_angle_;

    double wheel_radius_;
    double pivot_x_;
    double pivot_y_;
    double max_motor_speed_;
    double cmd_timeout_;

    geometry_msgs::msg::Twist cmd_;
    rclcpp::Time last_cmd_time_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr steer_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr drive_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SwerveControllerNode>());
    rclcpp::shutdown();
    return 0;
}
