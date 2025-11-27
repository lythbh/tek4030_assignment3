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

    auto rb = msg->rigidbodies[4];

    double x = rb.pose.position.x;
    double y = rb.pose.position.y;
    double yaw = tf2::getYaw(rb.pose.orientation);

    // target position
    double goal_x = 0.0;
    double goal_y = 0.0;

    // PD control parameters
    double K_p_lin = 0.5;
    double K_d_lin = 0.1;
    double K_p_ang = 1.0;
    double K_d_ang = 0.2;
    
    // tolerance (offset) in meters - robot stops within this distance from goal
    double tolerance = 0.05; // 5 centimeters

    // compute position error
    double dx = goal_x - x;
    double dy = goal_y - y;

    double dist = std::sqrt(dx*dx + dy*dy);

    double target_yaw = 0.0;
    double yaw_error = 0.0;

    if(dist > 1e-6)
    {
      double target_yaw = atan2(dy, dx);
      double yaw_error = smallestDeltaAngle(target_yaw, yaw);
    }

    // compute time delta
    auto current_time = this->now();
    double dt = (current_time - last_time_).seconds();
    if (dt < 1e-6) dt = 1e-6; // avoid division by zero

    // compute derivatives
    double dist_error_rate = (dist - last_dist_error_) / dt;
    double yaw_error_rate = (yaw_error - last_yaw_error_) / dt;

    // store current errors for next iteration
    last_dist_error_ = dist;
    last_yaw_error_ = yaw_error;
    last_time_ = current_time;

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = K_p_lin * dist + K_d_lin * dist_error_rate;
    cmd.angular.z = K_p_ang * yaw_error + K_d_ang * yaw_error_rate;
    //cmd.linear.x = 0.0;
    //cmd.angular.z = 0.0;

    RCLCPP_INFO(get_logger(),
      "dist: %.3f, dist_error_rate: %.3f",
    dist, dist_error_rate);

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
