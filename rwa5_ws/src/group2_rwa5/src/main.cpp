
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <memory>
#include "waypoint_publisher.hpp"
#include "waypoint_reacher.hpp"
#include "bot_waypoint_msgs/msg/bot_waypoint.hpp"

int main(int argc, char **argv)
{
    // Initilaize RCLPP
    rclcpp::init(argc, argv);

    // Create the 3 waypoint objects: Hard coding into main
    bot_waypoint_msgs::msg::BotWaypoint wp1;
    wp1.waypoint.set__x(4.0);
    wp1.waypoint.set__y(4.0);
    wp1.waypoint.set__theta(1.57);
    wp1.set__tolerance(wp1.SMALL);

    bot_waypoint_msgs::msg::BotWaypoint wp2;
    wp2.waypoint.set__x(4.0);
    wp2.waypoint.set__y(-4.0);
    wp2.waypoint.set__theta(3.14);
    wp2.set__tolerance(wp2.MEDIUM);

    bot_waypoint_msgs::msg::BotWaypoint wp3;
    wp3.waypoint.set__x(-4.0);
    wp3.waypoint.set__y(4.0);
    wp3.waypoint.set__theta(-3.14);
    wp3.set__tolerance(wp3.LARGE);

    // Push the waypoints to a vector
    std::vector<bot_waypoint_msgs::msg::BotWaypoint> waypoints{};
    waypoints.push_back(wp1);
    waypoints.push_back(wp2);
    waypoints.push_back(wp3);

    // Create the nodes: Creating the reacher node first so that the subscription to bot_waypoint would be ready before it is published (for safely ensuring no message loss)
    auto reacher_node = std::make_shared<WaypointReacher>();
    auto publisher_node = std::make_shared<WaypointPublisher>(waypoints);

    // Create a single threaded executor to run both nodes
    rclcpp::executors::SingleThreadedExecutor executor{};

    // Add both nodes to the executor
    executor.add_node(publisher_node);
    executor.add_node(reacher_node);

    // Spin the executor to process the callbacks
    executor.spin();

    // Shutdown ROS2
    rclcpp::shutdown();
    return 0;
}
