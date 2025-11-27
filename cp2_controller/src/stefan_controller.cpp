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
      last_time_(this->now()),
      first_run_(true) // --- FIX: Added flag for derivative kick
    {
        cmd_pub_ = create_publisher<geometry_msgs::msg/Twist>("/cmd_vel", 10);

        mocap_sub_ = create_subscription<mocap4r2_msgs::msg/RigidBodies>(
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
        if (std::abs(rb.pose.position.x) < 1e-6 && std::abs(rb.pose.position.y) < 1e-6) {
            return;
        }

        double x = rb.pose.position.x;
        double y = rb.pose.position.y;
        double yaw = tf2::getYaw(rb.pose.orientation);

        // --- FIX: Add NAN check for safety ---
        if (std::isnan(x) || std::isnan(y) || std::isnan(yaw))
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Received NAN from mocap. Halting robot.");
            geometry_msgs::msg::Twist cmd; // Publish zero
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            cmd_pub_->publish(cmd);
            return;
        }

        // target position
        double goal_x = 0.0;
        double goal_y = 0.0;
        // --- FIX: Define the final heading goal from the assignment ---
        const double final_heading_goal = 0.0; // 0 degrees

        // PD control parameters
        double K_p_lin = 1.2;
        double K_d_lin = 0.5;
        double K_p_ang = 1.5;
        double K_d_ang = 0.0; // You can try tuning this, e.g., 0.1
        
        // tolerance (offset) in meters
        double tolerance = 0.05;
        // --- FIX: Add a heading tolerance for the final stop ---
        const double heading_tolerance = 0.05; // ~3 degrees

        // compute position error
        double dx = goal_x - x;
        double dy = goal_y - y;

        double dist = std::sqrt(dx*dx + dy*dy);
        
        // compute time delta
        auto current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        if (dt < 1e-6) dt = 1e-6;

        // --- FIX: Prevent derivative kick on first run ---
        if (first_run_)
        {
            dt = 1e-6; // Effectively disables D-term for this loop
            first_run_ = false;
        }

        double target_yaw = 0.0;
        double yaw_error = 0.0;
        double yaw_error_rate = 0.0;
        double dist_error_rate = 0.0;

        geometry_msgs::msg::Twist cmd;

        // --- FIX: This is the 2-STAGE CONTROLLER LOGIC ---
        if(dist > tolerance)
        {
            // --- STAGE 1: GO-TO-GOAL (Far from origin) ---
            
            // 1. Target is the origin
            target_yaw = atan2(dy, dx);
            yaw_error = smallestDeltaAngle(target_yaw, yaw);
            
            // 2. Calculate derivatives
            double delta_yaw_error = smallestDeltaAngle(yaw_error, last_yaw_error_);
            yaw_error_rate = delta_yaw_error / dt;
            dist_error_rate = (dist - last_dist_error_) / dt;

            // 3. Calculate Linear Velocity (using your soft-start)
            double raw_linear = K_p_lin * dist + K_d_lin * dist_error_rate;
            double max_angle_threshold = 1.0;

            if (std::abs(yaw_error) > max_angle_threshold) {
                cmd.linear.x = 0.0; // Turn in place
            } else {
                // Linear scaling
                double scale = (max_angle_threshold - std::abs(yaw_error)) / max_angle_threshold;
                cmd.linear.x = raw_linear * scale;
            }
            
            // 4. Calculate Angular Velocity
            cmd.angular.z = K_p_ang * yaw_error + K_d_ang * yaw_error_rate;
        }
        else
        {
            // --- STAGE 2: POSTURE REGULATION (At origin) ---
            
            // 1. Target is the final heading (0 degrees)
            target_yaw = final_heading_goal;
            yaw_error = smallestDeltaAngle(target_yaw, yaw);

            // 2. Calculate derivatives
            double delta_yaw_error = smallestDeltaAngle(yaw_error, last_yaw_error_);
            yaw_error_rate = delta_yaw_error / dt;
            // Distance error is 0, so derivative tries to brake
            dist_error_rate = (0.0 - last_dist_error_) / dt; 

            // 3. STOP Linear Velocity
            cmd.linear.x = 0.0; 

            // 4. Calculate Angular Velocity
            cmd.angular.z = K_p_ang * yaw_error + K_d_ang * yaw_error_rate;

            // 5. Stop turning when aligned
            if (std::abs(yaw_error) < heading_tolerance) {
                cmd.angular.z = 0.0;
            }
        }
        // --- END OF 2-STAGE LOGIC ---


        // store current errors for next iteration
        // We store 'dist' in stage 1, and 0.0 in stage 2
        last_dist_error_ = (dist > tolerance) ? dist : 0.0;
        last_yaw_error_ = yaw_error;
        last_time_ = current_time;
        
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
            "dist: %.3f, cmd_lin: %.3f, yaw_err: %.3f, cmd_ang: %.3f",
            dist, cmd.linear.x, yaw_error, cmd.angular.z);

        cmd_pub_->publish(cmd);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<mocap4r2_msgs::msg/RigidBodies>::SharedPtr mocap_sub_;
    
    // PD controller state
    double last_dist_error_;
    double last_yaw_error_;
    rclcpp::Time last_time_;
    bool first_run_; // --- FIX: Added flag
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MocapController>());
    rclcpp::shutdown();
    return 0;
}
