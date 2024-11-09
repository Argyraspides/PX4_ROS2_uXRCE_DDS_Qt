#include "drone_controller.hpp"
#include <iostream>

DroneController::DroneController() : Node("drone_controller")
{
  // Set up QoS profile for PX4
  rclcpp::QoS qos(1);
  qos.keep_last(1);
  qos.best_effort();
  qos.durability_volatile();
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

  // Use QoS settings for both publisher and subscriber
  m_command_publisher = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", qos);

  auto odometry_callback_bind = std::bind(&DroneController::odometry_callback, this, std::placeholders::_1);

  m_odometry_subscriber = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
      "/fmu/out/vehicle_odometry",
      qos,
      odometry_callback_bind);

  auto vehicle_global_position_callback_bind = std::bind(&DroneController::vehicle_global_position_callback, this, std::placeholders::_1);

  m_vehicle_global_position_subscriber = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
      "/fmu/out/vehicle_global_position",
      qos,
      vehicle_global_position_callback_bind);
}

void DroneController::takeoff()
{

  // First make sure we're in takeoff mode
  auto mode_msg = px4_msgs::msg::VehicleCommand();
  auto now = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now());
  mode_msg.timestamp = now.time_since_epoch().count();
  mode_msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
  mode_msg.param1 = 1.0f; // Custom mode
  mode_msg.param2 = 4.0f; // Custom sub-mode
  mode_msg.target_system = 1;
  mode_msg.target_component = 1;
  mode_msg.source_system = 1;
  mode_msg.source_component = 1;
  mode_msg.from_external = true;

  m_command_publisher->publish(mode_msg);

  // Small delay to ensure mode switch is processed
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Then send takeoff command
  auto takeoff_msg = px4_msgs::msg::VehicleCommand();
  now = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now());
  takeoff_msg.timestamp = now.time_since_epoch().count();
  takeoff_msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF;
  takeoff_msg.param1 = -1.0f; // Pitch angle (negative angle for ascending)
  takeoff_msg.param2 = 0.0f;  // Empty
  takeoff_msg.param3 = 0.0f;  // Empty
  takeoff_msg.param4 = NAN;   // Yaw angle (NAN = use current heading)
  takeoff_msg.param5 = NAN;   // Latitude (NAN = use current position)
  takeoff_msg.param6 = NAN;   // Longitude (NAN = use current position)
  takeoff_msg.param7 = 2.5f;  // Altitude
  takeoff_msg.target_system = 1;
  takeoff_msg.target_component = 1;
  takeoff_msg.source_system = 1;
  takeoff_msg.source_component = 1;
  takeoff_msg.from_external = true;

  m_command_publisher->publish(takeoff_msg);
}

void DroneController::land()
{
  auto msg = px4_msgs::msg::VehicleCommand();
  msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND;
  m_command_publisher->publish(msg);
}

// Arm/disarm drone. 1.0f means arm, 0.0f means disarm
void DroneController::arm_disarm(const float &cmd)
{
  auto msg = px4_msgs::msg::VehicleCommand();

  // Get timestamp
  auto now = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now());
  msg.timestamp = now.time_since_epoch().count();

  // Command should be VEHICLE_CMD_COMPONENT_ARM_DISARM
  msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;

  msg.param1 = cmd;

  msg.target_system = 1;
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;
  msg.from_external = true;

  m_command_publisher->publish(msg);
}

double DroneController::get_current_lat()
{
  return m_current_lat;
}

double DroneController::get_current_lon()
{
  return m_current_lon;
}

void DroneController::odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
  m_current_x = msg->position[0];
  m_current_y = msg->position[1];
  m_current_z = msg->position[2];

  // std::cout << "XYZ: (" << m_current_x << "," << m_current_y << "," << m_current_z << ")" << std::endl;
}

void DroneController::vehicle_global_position_callback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg)
{

  m_current_lat = msg->lat;
  m_current_lon = msg->lon;
}
