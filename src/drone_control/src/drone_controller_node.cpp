#include <QApplication>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include "main_window.hpp"
#include "drone_controller.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);

    auto drone_controller = std::make_shared<DroneController>();
    MainWindow window(drone_controller);
    window.show();

    // Create a timer for ROS2 spinning
    QTimer ros_timer;
    ros_timer.connect(&ros_timer, &QTimer::timeout, [drone_controller]() {
        rclcpp::spin_some(drone_controller);
    });
    ros_timer.start(10);  // Spin every 10ms

    int result = app.exec();
    rclcpp::shutdown();
    return result;
}