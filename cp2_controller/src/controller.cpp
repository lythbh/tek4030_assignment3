#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <mocap4r2_msgs/msg/rigid_bodies.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <chrono>

class MocapController : public rclcpp::Node
{
public:
  MocapController() : Node("mocap_controller"),
    last_dist_error_(0.0), last_yaw_error_(0.0),
    first_run_(true) // <--- FIX: Flag to handle startup
  {
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    mocap_sub_ = create_subscription<mocap4r2_msgs::msg::RigidBodies>(
      "/rigid_bodies", 10,
      std::bind(&MocapController::mocap_callback, this, std::placeholders::_1));

    last_time_ = this->now();
    RCLCPP_INFO(get_logger(), "Controller started.");
  }

private:

  static double smallestDeltaAngle(const double& x, const double& y)
  {
    return atan2(sin(x - y), cos(x - y));
  }

  void mocap_callback(const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg)
  {
    // --- SAFETY CHECK 1: Empty Message ---
    if (msg->rigidbodies.empty()) return;

    // --- SAFETY CHECK 2: Index Out of Bounds ---
    // You were accessing [4]. Ensure it exists or default to [0]
    // If you specifically need ID 4, you must verify size > 4.
    size_t target_idx = 4;
    if (msg->rigidbodies.size() <= target_idx) {
        // Fallback or return to avoid crash
        // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Rigid body index %zu not found", target_idx);
        return; 
    }
    auto rb = msg->rigidbodies[target_idx];

    // --- SAFETY CHECK 3: NaN Handling (Fixes the "NaN after moving" issue) ---
    if (std::isnan(rb.pose.position.x) || std::isnan(rb.pose.position.y)) {
        // Tracking lost. STOP the robot to be safe.
        stop_robot();
        return; 
    }

    double x = rb.pose.position.x;
    double y = rb.pose.position.y;
    double yaw = tf2::getYaw(rb.pose.orientation);

    double goal_x = 0.0;
    double goal_y = 0.0;

    // PD parameters
    double K_p_lin = 0.5;
    double K_d_lin = 0.1;
    double K_p_ang = 1.0;
    double K_d_ang = 0.2;
    
    double tolerance = 0.05; 

    double dx = goal_x - x;
    double dy = goal_y - y;
    double dist = std::sqrt(dx*dx + dy*dy);

    double target_yaw = 0.0;
    double yaw_error = 0.0;

    if(dist > 0.01) // Small deadband to prevent atan2 instability
    {
      // <--- FIX: Removed 'double' to fix variable shadowing
      target_yaw = atan2(dy, dx);
      yaw_error = smallestDeltaAngle(target_yaw, yaw);
    }

    // Time delta
    auto current_time = this->now();
    double dt = (current_time - last_time_).seconds();
    if (dt < 1e-6) dt = 1e-6; 

    // <--- FIX: Handle first run to avoid derivative spike (Jumping)
    if (first_run_) {
        last_dist_error_ = dist;
        last_yaw_error_ = yaw_error;
        last_time_ = current_time;
        first_run_ = false;
        return; 
    }

    double dist_error_rate = (dist - last_dist_error_) / dt;
    double yaw_error_rate = (yaw_error - last_yaw_error_) / dt;

    last_dist_error_ = dist;
    last_yaw_error_ = yaw_error;
    last_time_ = current_time;

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = K_p_lin * dist + K_d_lin * dist_error_rate;
    cmd.angular.z = K_p_ang * yaw_error + K_d_ang * yaw_error_rate;

    // --- MISSION OPTION: Speed Clamping ---
    // Prevents the robot from flying off if errors are large
    double max_speed = 0.25; 
    if (cmd.linear.x > max_speed) cmd.linear.x = max_speed;
    if (cmd.linear.x < -max_speed) cmd.linear.x = -max_speed;

    // --- MISSION OPTION: Deadband (Stop at goal) ---
    if (dist < tolerance) {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
    }

    cmd_pub_->publish(cmd);
  }

  void stop_robot() {
      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = 0.0; 
      cmd.angular.z = 0.0;
      cmd_pub_->publish(cmd);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<mocap4r2_msgs::msg::RigidBodies>::SharedPtr mocap_sub_;
  
  double last_dist_error_;
  double last_yaw_error_;
  rclcpp::Time last_time_;
  bool first_run_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MocapController>());
  rclcpp::shutdown();
  return 0;
}
