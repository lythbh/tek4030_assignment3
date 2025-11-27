#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <mocap4r2_msgs/msg/rigid_bodies.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <chrono>

// Define states for our robot behavior
enum class RobotState {
  APPROACH,   // Driving towards (0,0)
  ALIGN_X,    // At (0,0), turning to face X axis
  FINISHED    // Task complete
};

class MocapController : public rclcpp::Node
{
public:
  MocapController() : Node("mocap_controller"),
    last_dist_error_(0.0), last_yaw_error_(0.0),
    last_time_(this->now()),
    current_state_(RobotState::APPROACH) // Start in Approach mode
  {
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    mocap_sub_ = create_subscription<mocap4r2_msgs::msg::RigidBodies>(
      "/rigid_bodies", 10,
      std::bind(&MocapController::mocap_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Controller started. State: APPROACH");
  }

private:

  static double smallestDeltaAngle(const double& x, const double& y)
  {
    return atan2(sin(x - y), cos(x - y));
  }

  void mocap_callback(const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg)
  {
    if (msg->rigidbodies.empty()) return;
    if (msg->rigidbodies.size() <= 4) return;

    auto rb = msg->rigidbodies[4];

    // Tracking loss filter
    if (std::abs(rb.pose.position.x) < 1e-6 && std::abs(rb.pose.position.y) < 1e-6) return;

    double x = rb.pose.position.x;
    double y = rb.pose.position.y;
    double yaw = tf2::getYaw(rb.pose.orientation);

    // Goal is (0,0)
    double goal_x = 0.0;
    double goal_y = 0.0;
    
    // Calculate distance to goal
    double dx = goal_x - x;
    double dy = goal_y - y;
    double dist = std::sqrt(dx*dx + dy*dy);
    
    // Time calculation
    auto current_time = this->now();
    double dt = (current_time - last_time_).seconds();
    if (dt < 1e-6) dt = 1e-6;

    // --- STATE MACHINE LOGIC ---
    double tolerance = 0.05; // 5cm tolerance
    double align_tolerance = 0.05; // ~3 degrees tolerance

    if (current_state_ == RobotState::APPROACH) {
        if (dist < tolerance) {
            current_state_ = RobotState::ALIGN_X;
            RCLCPP_INFO(get_logger(), "Target Reached. Switching to ALIGN_X.");
        }
    } 
    else if (current_state_ == RobotState::ALIGN_X) {
        // If we drift away too far, should we switch back? 
        // For now, let's just stay in ALIGN mode to prevent oscillation.
        
        // Optional: Check if alignment is done
        // if (std::abs(yaw) < align_tolerance) current_state_ = RobotState::FINISHED;
    }

    // --- CONTROL CALCULATION BASED ON STATE ---

    double target_yaw = 0.0;
    double cmd_linear_x = 0.0;
    
    // PID Gains
    double K_p_lin = 1.2;
    double K_d_lin = 0.5;
    double K_p_ang = 1.5;
    double K_d_ang = 0.1; 

    double yaw_error = 0.0;

    if (current_state_ == RobotState::APPROACH) {
        // 1. Point towards the goal
        target_yaw = atan2(dy, dx);
        
        // Calculate Errors
        yaw_error = smallestDeltaAngle(target_yaw, yaw);
        double dist_error_rate = (dist - last_dist_error_) / dt;

        // Linear Motion Logic (with orientation gate)
        double raw_linear = K_p_lin * dist + K_d_lin * dist_error_rate;
        double max_angle_threshold = 1.0;

        if (std::abs(yaw_error) > max_angle_threshold) {
            cmd_linear_x = 0.0;
        } else {
            double scale = (max_angle_threshold - std::abs(yaw_error)) / max_angle_threshold;
            cmd_linear_x = raw_linear * scale;
        }

    } 
    else if (current_state_ == RobotState::ALIGN_X) {
        // 1. Point towards World X (Yaw = 0.0)
        target_yaw = 0.0; 
        
        // 2. Linear velocity is ZERO. We rotate in place.
        cmd_linear_x = 0.0;
        
        // Calculate yaw error relative to X-axis
        yaw_error = smallestDeltaAngle(target_yaw, yaw);
    }
    else if (current_state_ == RobotState::FINISHED) {
        // Do nothing
        yaw_error = 0.0;
        cmd_linear_x = 0.0;
    }

    // --- COMMON PID FOR ROTATION ---
    
    // Derivative for yaw (Fixed wrap-around issue)
    double delta_yaw_error = smallestDeltaAngle(yaw_error, last_yaw_error_);
    double yaw_error_rate = delta_yaw_error / dt;

    double cmd_angular_z = K_p_ang * yaw_error + K_d_ang * yaw_error_rate;

    // Update History
    last_dist_error_ = dist;
    last_yaw_error_ = yaw_error;
    last_time_ = current_time;

    // --- SAFETY CLAMPS ---
    const double MAX_LIN_VEL = 0.5;
    const double MAX_ANG_VEL = 1.0;

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = std::clamp(cmd_linear_x, -MAX_LIN_VEL, MAX_LIN_VEL);
    cmd.angular.z = std::clamp(cmd_angular_z, -MAX_ANG_VEL, MAX_ANG_VEL);

    // Logging state helps debugging
    RCLCPP_INFO(get_logger(),
      "State: %d | Dist: %.3f | YawErr: %.3f | CmdLin: %.3f",
      (int)current_state_, dist, yaw_error, cmd.linear.x);

    cmd_pub_->publish(cmd);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<mocap4r2_msgs::msg::RigidBodies>::SharedPtr mocap_sub_;
  
  double last_dist_error_;
  double last_yaw_error_;
  rclcpp::Time last_time_;
  
  RobotState current_state_; // Variable to hold the current state
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MocapController>());
  rclcpp::shutdown();
  return 0;
}
