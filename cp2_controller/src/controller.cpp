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
    last_time_(this->now())
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
    if (msg->rigidbodies.empty())
      return;

    // Safety check: ensure index 4 exists
    if (msg->rigidbodies.size() <= 4) 
      return;

    auto rb = msg->rigidbodies[4];

    // FIX: Filter out tracking loss.
    // If Mocap loses markers, it returns exactly (0,0,0).
    // It is physically impossible to jump from 0.4m to 0.0m in 0.1s.
    if (std::abs(rb.pose.position.x) < 1e-6 && std::abs(rb.pose.position.y) < 1e-6) {
        // Ignore this frame and keep moving based on previous valid data
        // or let the external watchdog handle safety.
        return;
    }

    double x = rb.pose.position.x;
    double y = rb.pose.position.y;
    double yaw = tf2::getYaw(rb.pose.orientation);

    // target position
    double goal_x = 0.0;
    double goal_y = 0.0;

    // PD control parameters
    double K_p_lin = 1.2;
    double K_d_lin = 0.5;
    double K_p_ang = 0.8;
    double K_d_ang = 0.1;
    
    // tolerance (offset) in meters
    double tolerance = 0.05;

    // compute position error
    double dx = goal_x - x;
    double dy = goal_y - y;

    double dist = std::sqrt(dx*dx + dy*dy);
    
    // compute time delta
    auto current_time = this->now();
    double dt = (current_time - last_time_).seconds();
    if (dt < 1e-6) dt = 1e-6;

    double target_yaw = 0.0;
    double yaw_error = 0.0;
    double yaw_error_rate = 0.0;
    double dist_error_rate = 0.0;

    geometry_msgs::msg::Twist cmd;

    // Only calculate errors if outside tolerance
    if(dist > tolerance)
    {
      target_yaw = atan2(dy, dx);
      yaw_error = smallestDeltaAngle(target_yaw, yaw);
      
      // 2. FIX: Handle Angle Wrap in Derivative
      // If error jumps from -3.1 to +3.1, the delta is small, not huge.
      double delta_yaw_error = smallestDeltaAngle(yaw_error, last_yaw_error_);
      yaw_error_rate = delta_yaw_error / dt;
      
      dist_error_rate = (dist - last_dist_error_) / dt;
    }
    else {
      // Inside tolerance: Stop everything. 
      while (target_yaw < 0.1 && target_yaw > -0.1){
        cmd.angular.z = 0.1;
        cmd.linear.x = 0.0;
      }
    }

    // store current errors for next iteration
    // If we are in tolerance, we essentially reset the history to 0
    last_dist_error_ = dist;
    last_yaw_error_ = yaw_error;
    last_time_ = current_time;
    
    // FIX: Soft Start Orientation Gate.
    // Instead of a hard cut-off at 1.0 rad (which caused the start-stop oscillation),
    // we scale the linear velocity based on how well aligned the robot is.
    // Error > 1.0 rad: 0% Speed (Turn only)
    // Error = 0.0 rad: 100% Speed
    double max_angle_threshold = 1.0;

    double raw_linear = K_p_lin * dist + K_d_lin * dist_error_rate;

    if (std::abs(yaw_error) > max_angle_threshold) {
        cmd.linear.x = 0.0;
    } else {
        // Linear scaling
        double scale = (max_angle_threshold - std::abs(yaw_error)) / max_angle_threshold;
        cmd.linear.x = raw_linear * scale;
    }
    
    // Clamp output to prevent motor saturation (optional but recommended)
    if(cmd.linear.x > 0.5) cmd.linear.x = 0.5; 

    cmd.angular.z = K_p_ang * yaw_error + K_d_ang * yaw_error_rate;

    // --- SAFETY CLAMP START ---
    
    // Define maximum limits (adjust these to your robot's specs)
    const double MAX_LIN_VEL = 0.5; // meters/sec
    const double MAX_ANG_VEL = 1.0; // radians/sec (~57 degrees/sec)

    // Clamp Linear Velocity
    if (cmd.linear.x > MAX_LIN_VEL) cmd.linear.x = MAX_LIN_VEL;
    if (cmd.linear.x < -MAX_LIN_VEL) cmd.linear.x = -MAX_LIN_VEL;

    // Clamp Angular Velocity
    if (cmd.angular.z > MAX_ANG_VEL) cmd.angular.z = MAX_ANG_VEL;
    if (cmd.angular.z < -MAX_ANG_VEL) cmd.angular.z = -MAX_ANG_VEL;

    // --- SAFETY CLAMP END ---

    RCLCPP_INFO(get_logger(),
      "dist: %.3f, cmd_lin: %.3f, yaw_err: %.3f",
      dist, cmd.linear.x, yaw_error);

    cmd_pub_->publish(cmd);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<mocap4r2_msgs::msg::RigidBodies>::SharedPtr mocap_sub_;
  
  // PD controller state
  double last_dist_error_;
  double last_yaw_error_;
  rclcpp::Time last_time_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MocapController>());
  rclcpp::shutdown();
  return 0;
}
