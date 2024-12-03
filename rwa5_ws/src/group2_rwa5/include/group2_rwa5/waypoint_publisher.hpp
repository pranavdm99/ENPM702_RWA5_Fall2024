/**
 * @file waypoint_publisher.hpp
 * @author Pranav ANV (anvpran@umd.edu)
 * @author Pranav Deshakulkarni Manjunath (pdeshaku@umd.edu)
 * @author Lakshmi Pravallika Adibhatla (ladibhat@umd.edu)
 * @brief This file implements the WaypointPublisher class corresponding to the waypoint_publisher node to manage the waypoints along its motion
 * @version 0.1
 * @date 2024-12-02
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <rclcpp/rclcpp.hpp>
#include "bot_waypoint_msgs/msg/bot_waypoint.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "std_msgs/msg/bool.hpp"

/**
 * @brief Using this namespace so that the "ms" suffix can be used to enter timer duration values with tick periods in "ms"
 *
 */
using namespace std::chrono_literals;

/**
 * @brief This class corresponds to the "waypoint_publisher" node. It manages and publishes a sequence of waypoints for the reacher node
 *
 */
class WaypointPublisher : public rclcpp::Node
{
public:
    /**
     * @brief Initializes a new WaypointPublisher node instance. It copies the waypoints, and set the request to pending so that a new request is published
     *
     * @param wp_list This is a const reference to the waypoints which need to be published
     */
    WaypointPublisher(const std::vector<bot_waypoint_msgs::msg::BotWaypoint> &wp_list)
        : Node("waypoint_publisher"), waypoints_{wp_list}, index_{0}, new_request_pending_{true}
    {
        // Create the publisher to publish to the topic "bot_waypoint" messages of type bot_waypoint_msgs::msg::BotWaypoint
        waypoint_pub_ = this->create_publisher<bot_waypoint_msgs::msg::BotWaypoint>("bot_waypoint", 10);

        // Create a subscriber to receive the signal from the reacher node to send the point in the sequence in the topic "next_waypoint"
        next_waypoint_sub_ = this->create_subscription<std_msgs::msg::Bool>("next_waypoint", 10, std::bind(&WaypointPublisher::next_waypoint_callback, this, std::placeholders::_1));

        // Create a timer to handle the control action of this node, which is to check if there is a request for the next waypoint
        timer_ = this->create_wall_timer(500ms, std::bind(&WaypointPublisher::timer_callback, this));
    }

private:
    /**
     * @brief Timer callback to handle the control loop for this node
     *
     */
    void timer_callback();

    /**
     * @brief Callback function to handle the request for the next waypoint in the sequence
     *
     * @param msg Only contains a boolean value
     */
    void next_waypoint_callback(const std_msgs::msg::Bool::SharedPtr msg);

    /**
     * @brief Create a publisher to publish the waypoint with coordinates to the reacher, once upon creation, and again whenever there is a request from the reacher node
     *
     */
    rclcpp::Publisher<bot_waypoint_msgs::msg::BotWaypoint>::SharedPtr waypoint_pub_;

    /**
     * @brief Create a subscriber to the next waypoint request from the reacher node, when the goal is reached by the Robot
     *
     */
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr next_waypoint_sub_;

    /**
     * @brief Create a wall timer to handle the control of this node, and check for waypoint requests asynchronously
     *
     */
    rclcpp::TimerBase::SharedPtr timer_;

    /**
     * @brief The container of waypoints, initialized from the constructor of this class, which expects a const reference to all the waypoint objects in the form of a vector
     *
     */
    std::vector<bot_waypoint_msgs::msg::BotWaypoint> waypoints_;

    /**
     * @brief Position within the container at which the next request is to be sent
     *
     */
    size_t index_;

    /**
     * @brief Boolean value which is set when a request is pending and reset after it has been handled
     *
     */
    bool new_request_pending_;
};
