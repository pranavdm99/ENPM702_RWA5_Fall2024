/**
 * @file waypoint_reacher.hpp
 * @author Pranav ANV (anvpran@umd.edu)
 * @author Pranav Deshakulkarni Manjunath (pdeshaku@umd.edu)
 * @author Lakshmi Pravallika Adibhatla (ladibhat@umd.edu)
 * @brief This file implements the WaypointReacher class corresponding to the waypoint_reacher node to guide the Robot to reach its targets
 * @version 0.1
 * @date 2024-12-02
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include "bot_waypoint_msgs/msg/bot_waypoint.hpp"
#include "std_msgs/msg/bool.hpp"

#include <cmath>

using namespace std::chrono_literals;

class WaypointReacher : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new WaypointReacher node instance
     *
     */
    WaypointReacher()
        // Initialize attributes
        : Node{"waypoint_reacher"}, waypoint_received_{false}, goal_position_reached_{false}, goal_orientation_reached_{false}, published_next_waypoint_{false}
    {
        // Create the subscriber to receive the waypoints messages of type bot_waypoint_msgs::msg::BotWaypoint from the topic "bot_waypoint" from the publisher
        waypoint_sub_ = this->create_subscription<bot_waypoint_msgs::msg::BotWaypoint>("bot_waypoint", 10, std::bind(&WaypointReacher::waypoint_callback, this, std::placeholders::_1));

        // Create the subscriber to the Odom topic to receive the current position and orientation of the Robot
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&WaypointReacher::odom_callback, this, std::placeholders::_1));

        // Create the publisher for the "cmd_vel" topic to send the velocity commands to turtlebot to reach the required target waypoints
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // Create the publisher for the "next_waypoint" command to request for the next waypoint in the sequence (if any)
        next_waypoint_pub_ = this->create_publisher<std_msgs::msg::Bool>("next_waypoint", 10);

        // Create a wall timer for the control loop
        timer_ = this->create_wall_timer(500ms, std::bind(&WaypointReacher::control_loop, this));
    }

private:
    /**
     * @brief Callback function for the bot_waypoint topic, receives and stores the target point to reach
     *
     * @param msg Contains the goal coordinates, orientation and tolerance values
     */
    void waypoint_callback(const bot_waypoint_msgs::msg::BotWaypoint::SharedPtr msg);

    /**
     * @brief Callback function for the odom topic, receives and stores the latest position and orientation information
     *
     * @param msg Contains the X,Y,Z position and Quaternion orientation of the Robot relative to the home position (0,0,0s)
     */
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    /**
     * @brief Command to move to the Turtlebot (publishes to the "cmd_vel" topic)
     *
     * @param lin_x Linear X Velocity
     * @param lin_y Linear Y Velocity
     * @param lin_z Linear Z Velocity
     * @param ang_x Angular X (Roll) Velocity
     * @param ang_y Angular Y (Pitch) Velocity
     * @param ang_z Angular Z (Yaw) Velocity
     */
    void move(double lin_x, double lin_y, double lin_z, double ang_x, double ang_y, double ang_z);

    /**
     * @brief Command to stop the Turtlebot (publishes 0s to the "cmd_vel" topic)
     *
     */
    void stop();

    /**
     * @brief Timer callback, checks if a waypoint is available and sends the velocity commands to reach the waypoint
     *
     */
    void control_loop();

    /**
     * @brief Converts quaternion orientation into a yaw angle.
     *
     * @param curr_orientation Const reference to the current orientation obtained from odom
     * @return double Yaw angle (About Z)
     */
    double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion &curr_orientation);

    /**
     * @brief Subscriber to the bot_waypoint topic
     *
     */
    rclcpp::Subscription<bot_waypoint_msgs::msg::BotWaypoint>::SharedPtr waypoint_sub_;

    /**
     * @brief Subscriber to the odom topic
     *
     */
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    /**
     * @brief Publisher to the cmd_vel topic
     *
     */
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    /**
     * @brief Publisher to the next_waypoint topic
     *
     */
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr next_waypoint_pub_;

    /**
     * @brief Timer instance
     *
     */
    rclcpp::TimerBase::SharedPtr timer_;

    /**
     * @brief Current waypoint as received from the WaypointPublisher
     *
     */
    bot_waypoint_msgs::msg::BotWaypoint current_waypoint_;

    /**
     * @brief Current pose as received from Odom
     *
     */
    geometry_msgs::msg::Pose current_pose_;

    /**
     * @brief This flag is set when new waypoint is received and reset when the robot reaches this waypoint
     *
     */
    bool waypoint_received_;

    /**
     * @brief This flag is set when the position specified by the waypoint is reached within the tolerance specified by the waypoint
     *
     */
    bool goal_position_reached_;

    /**
     * @brief This flag is set when the position specified by the waypoint is reached within a fixed tolerance (0.05 rad)
     *
     */
    bool goal_orientation_reached_;

    /**
     * @brief This flag is set when the Reacher requests for a new waypoint from the Publisher
     *
     */
    bool published_next_waypoint_;
};
