#include <rclcpp/rclcpp.hpp>
#include <power_monitor/power_monitor.h>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PowerMonitor>());
    rclcpp::shutdown();
    return 0;
}