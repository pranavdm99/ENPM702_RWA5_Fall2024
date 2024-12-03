#include "waypoint_reacher.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

void WaypointReacher::move(double lin_x,
                           double lin_y,
                           double lin_z,
                           double ang_x,
                           double ang_y,
                           double ang_z)
{
    // Create a Twist message
    geometry_msgs::msg::Twist twist{};

    // Copy the linear and angular velocities
    twist.linear.x = lin_x;
    twist.linear.y = lin_y;
    twist.linear.z = lin_z;
    twist.angular.x = ang_x;
    twist.angular.y = ang_y;
    twist.angular.z = ang_z;

    // Publish the message to the cmd_vel topic
    cmd_vel_pub_->publish(twist);
}

void WaypointReacher::stop()
{
    // Pass zeros to the move function to bring the Robot to a halt
    move(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}

double WaypointReacher::get_yaw_from_quaternion(
    const geometry_msgs::msg::Quaternion &curr_orientation)
{
    // Get the Quaternion from the current orientation
    tf2::Quaternion quaternion{curr_orientation.x, curr_orientation.y,
                               curr_orientation.z, curr_orientation.w};

    // Convert quaternion to roll, pitch, and yaw
    double roll{};
    double pitch{};
    double yaw{};
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

    return yaw;
}

void WaypointReacher::waypoint_callback(
    const bot_waypoint_msgs::msg::BotWaypoint::SharedPtr msg)
{
    // Copy the waypoint info, which contains the pose and the tolerance
    current_waypoint_ = *msg;

    // Set the waypoint_received_ flag to enter the control loop
    // Reset the goal_position_reached_ and goal_orientation_reached_ and
    // published_next_waypoint_ flags to enter the correct sections of the loop
    waypoint_received_ = true;
    goal_position_reached_ = false;
    goal_orientation_reached_ = false;
    published_next_waypoint_ = false;

    RCLCPP_INFO(this->get_logger(),
                "Received new waypoint: x=%.2f, y=%.2f, theta=%.2f, tolerance=%.2f",
                current_waypoint_.waypoint.x, current_waypoint_.waypoint.y,
                current_waypoint_.waypoint.theta, (current_waypoint_.tolerance / 10.0));
}

void WaypointReacher::odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Update the current pose
    current_pose_ = msg->pose.pose;
}

void WaypointReacher::control_loop()
{
    // Check if a waypoint is available
    if (!waypoint_received_)
        return;

    // As long as a waypoint is available, keep publishing the Current Pose in this control loop
    RCLCPP_INFO(this->get_logger(), "Current 2D Pose: x = %.2f, y = %.2f, theta = %.2f",
                current_pose_.position.x, current_pose_.position.y,
                get_yaw_from_quaternion(current_pose_.orientation));

    // Now check if the goal defined by the waypoint has been reached by the Robot
    if (!goal_position_reached_)
    {
        // The Robot has not reached the goal, compute the error in distance to
        // cover in X and Y directions to reach the goal
        double dx{current_waypoint_.waypoint.x - current_pose_.position.x};
        double dy{current_waypoint_.waypoint.y - current_pose_.position.y};

        // Compute the remaining (Euclidean) distance to cover to reach the goal
        double distance{std::sqrt(dx * dx + dy * dy)};

        // Convert tolerance to distance in meters
        double tolerance_distance = current_waypoint_.tolerance / 10.0;

        // Move towards the waypoint if the distance to cover is larger than the
        // tolerance
        if (distance > tolerance_distance)
        {
            // From the current orientation of the Robot, compute the yaw
            double curr_yaw = get_yaw_from_quaternion(current_pose_.orientation);

            // Compute the remaining angle to reach the goal
            double dtheta = std::atan2(dy, dx);

            // Compute the correction in the angle
            double angle_correction = dtheta - curr_yaw;

            // Normalize angle_correction to [-pi, pi]
            angle_correction = atan2(sin(angle_correction), cos(angle_correction));

            // Proportional control constants
            double k_linear{1.0};
            double k_angular{0.8};

            // Limit velocities and publish the command
            double lin_x_vel{std::min(k_linear * distance, 0.22)};
            double ang_z_vel{std::min(std::max(k_angular * angle_correction, -0.5), 0.5)};
            move(lin_x_vel, 0.0, 0.0, 0.0, 0.0, ang_z_vel);
        }
        else
        {
            // Linear error is within tolerance, so stop moving momentarily.
            // After this we orient the Turtlebot so that the target orientation
            // is also reached (within a tolerance). Set the position reached flag
            stop();
            goal_position_reached_ = true;
        }
    }
    else if (!goal_orientation_reached_)
    {
        // This means the Robot has reached the coordinates of the goal
        // (but not necessarily the angle)

        // Read the current yaw angle of the Robot
        double curr_yaw{get_yaw_from_quaternion(current_pose_.orientation)};

        // Check the final angle to rotate
        double theta_goal{current_waypoint_.waypoint.theta};

        // Compute the correction in the angle
        double angle_correction{theta_goal - curr_yaw};

        // Normalize angle_correction to [-pi, pi]
        angle_correction = atan2(sin(angle_correction), cos(angle_correction));

        if (std::abs(angle_correction) > 0.05)
        {
            // Rotate in place
            double k_angular{1.0};
            double ang_z_vel{std::min(std::max(k_angular * angle_correction, -1.0), 1.0)};
            move(0.0, 0.0, 0.0, 0.0, 0.0, ang_z_vel);
        }
        else
        {
            // Angular error is also within tolerance, so stop rotation as well
            // and set the orientation reached flag
            stop();
            goal_orientation_reached_ = true;
        }
    }
    else if (!published_next_waypoint_)
    {
        // At this point, there is no more movement required, just request for
        // the next waypoint
        RCLCPP_INFO(this->get_logger(),
                    "Reached Waypoint Target at x=%.2f, y=%.2f, theta=%.2f, tolerance=%.2f",
                    current_waypoint_.waypoint.x, current_waypoint_.waypoint.y,
                    current_waypoint_.waypoint.theta, (current_waypoint_.tolerance / 10.0));

        // Reset the received flag
        waypoint_received_ = false;

        // Publish true to next_waypoint topic
        std_msgs::msg::Bool next_wp_msg{};
        next_wp_msg.data = true;
        next_waypoint_pub_->publish(next_wp_msg);

        // Set the published flag
        published_next_waypoint_ = true;
        RCLCPP_INFO(this->get_logger(), "Waiting for next waypoint...");
    }
}
