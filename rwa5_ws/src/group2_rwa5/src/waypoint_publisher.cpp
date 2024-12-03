#include <waypoint_publisher.hpp>

void WaypointPublisher::timer_callback()
{
    // Check if there are any pending requests
    if (!new_request_pending_)
        return;

    // Given that there is a pending request, check if the index has exceeded the size
    if (index_ < waypoints_.size())
    {
        // One or more waypoints are still available, publish the waypoint at the current index and reset the pending flag
        new_request_pending_ = false;
        waypoint_pub_->publish(waypoints_[index_]);
        RCLCPP_INFO(this->get_logger(), "Published waypoint %zu", index_);
        index_++;
    }
    else if (index_ >= waypoints_.size())
    {
        // No more waypoints available, shutdown the node
        RCLCPP_INFO(this->get_logger(), "All waypoints published. Shutting down.");
        rclcpp::shutdown();
    }
}

void WaypointPublisher::next_waypoint_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    // Received the request for the next waypoint, check validity
    if (!msg->data)
        return;

    // Request is valid, set the pending flag
    new_request_pending_ = true;
    RCLCPP_INFO(this->get_logger(), "Received signal to publish next waypoint.");
}
