#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <mocap4r2_msgs/msg/rigid_bodies.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <algorithm>
#include <chrono>

class MocapController : public rclcpp::Node
{
public:
  MocapController() : Node("mocap_controller"),
    last_dist_error_(0.0), last_yaw_error_(0.0), last_forward_error_(0.0),
    last_forward_error_rate_(0.0), last_yaw_error_rate_(0.0), last_time_(this->now())
  {
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    mocap_sub_ = create_subscription<mocap4r2_msgs::msg::RigidBodies>(
      "/rigid_bodies", 10,
      std::bind(&MocapController::mocap_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Controller started.");
  }

private:

  static double smallestDeltaAngle(const double& x, const double& y)
  {
    return atan2(sin(x - y), cos(x - y));
  }

  void mocap_callback(const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg)
  {
    if (msg->rigidbodies.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000, "No rigid bodies in mocap message");
      return;
    }

    // pick first rigid body (safer than hard-coded index)
    auto rb = msg->rigidbodies.front();

    double x = rb.pose.position.x;
    double y = rb.pose.position.y;
    double yaw = tf2::getYaw(rb.pose.orientation);

    // target position
    double goal_x = 0.0;
    double goal_y = 0.0;

    // PD control parameters (safer defaults)
    const double K_p_lin = 0.8;
    const double K_d_lin = 0.1;
    const double K_p_ang = 0.3;
    const double K_d_ang = 0.2;

    // safety / behavior parameters
    const double tolerance = 0.05; // 5 cm
    const double yaw_thresh = 0.6; // rad; if yaw error larger than this, don't drive forward
    const double max_lin = 0.7; // m/s
    const double max_ang = 1.5; // rad/s
    const double deriv_alpha = 0.6; // derivative filter smoothing (0..1)

    // compute position error (world frame)
    double dx = goal_x - x;
    double dy = goal_y - y;

    double dist = std::sqrt(dx*dx + dy*dy);

    double target_yaw = 0.0;
    double yaw_error = 0.0;

    if (dist > 1e-6) {
      target_yaw = atan2(dy, dx);
      yaw_error = smallestDeltaAngle(target_yaw, yaw);
    }

    // compute time delta
    auto current_time = this->now();
    double dt = (current_time - last_time_).seconds();
    if (dt < 1e-6) dt = 1e-6; // avoid division by zero

    // transform world error into robot body-forward signed error
    double forward_error = std::cos(yaw) * dx + std::sin(yaw) * dy; // signed distance along robot x-axis

    // derivatives (raw)
    double forward_error_rate_raw = (forward_error - last_forward_error_) / dt;
    double yaw_error_rate_raw = (yaw_error - last_yaw_error_) / dt;

    // low-pass filter derivatives to reduce noise
    double forward_error_rate = deriv_alpha * last_forward_error_rate_ + (1.0 - deriv_alpha) * forward_error_rate_raw;
    double yaw_error_rate = deriv_alpha * last_yaw_error_rate_ + (1.0 - deriv_alpha) * yaw_error_rate_raw;

    // store current values for next iteration
    last_forward_error_ = forward_error;
    last_forward_error_rate_ = forward_error_rate;
    last_yaw_error_ = yaw_error;
    last_yaw_error_rate_ = yaw_error_rate;
    last_time_ = current_time;

    geometry_msgs::msg::Twist cmd;

    // PD on forward signed error (positive means drive forward)
    double v_cmd = K_p_lin * forward_error + K_d_lin * forward_error_rate;

    // if facing away from goal, don't try to move forward
    if (std::abs(yaw_error) > yaw_thresh) v_cmd = 0.0;

    // PD for angular velocity
    double omega_cmd = K_p_ang * yaw_error + K_d_ang * yaw_error_rate;

    // deadband when close to goal
    if (dist < tolerance) {
      v_cmd = 0.0;
      omega_cmd = 0.0;
    }

    // clamp outputs to safe limits
    v_cmd = std::clamp(v_cmd, -max_lin, max_lin);
    omega_cmd = std::clamp(omega_cmd, -max_ang, max_ang);

    cmd.linear.x = v_cmd;
    cmd.angular.z = omega_cmd;
    //cmd.linear.x = 0.0;
    //cmd.angular.z = 0.0;

    RCLCPP_INFO(get_logger(),
      "dist: %.3f, forward_err: %.3f, v_cmd: %.3f, omega_cmd: %.3f",
      dist, forward_error, v_cmd, omega_cmd);

    cmd_pub_->publish(cmd);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<mocap4r2_msgs::msg::RigidBodies>::SharedPtr mocap_sub_;
  
  // PD controller state
  double last_dist_error_;
  double last_yaw_error_;
  double last_forward_error_;
  double last_forward_error_rate_;
  double last_yaw_error_rate_;
  rclcpp::Time last_time_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MocapController>());
  rclcpp::shutdown();
  return 0;
}