#pragma once
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>

class DroneController : public rclcpp::Node
{
public:
    DroneController();
    void takeoff();
    void land();
    void arm_disarm(const float& cmd);

    double get_current_lat();
    double get_current_lon();

private:
    void odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
    void vehicle_global_position_callback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg);

    // Publishers
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr m_command_publisher;

    // Subscribers
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr m_odometry_subscriber;
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr m_vehicle_global_position_subscriber;

    // Received data
    double m_current_x = 0, m_current_y = 0, m_current_z = 0;
    double m_current_lat = 0, m_current_lon = 0;
};