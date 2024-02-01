#ifndef POWER_MONITOR_VOLTAGE_DRIVER_
#define POWER_MONITOR_VOLTAGE_DRIVER_

#include <chrono>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <mvp_msgs/msg/power.hpp>

#include <power_monitor/MCP3424.h>
#include <power_monitor/default.h>

using namespace std::chrono_literals;

class PowerMonitor : public rclcpp::Node
{
public:
    PowerMonitor();

    void CallbackTimer();

private:
    void LoadParam();

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<mvp_msgs::msg::Power>::SharedPtr publisher_;

    //! ROS parameters

    std::string frame_id_;

    //! parameters

    std::shared_ptr<MCP3424> voltage_;

    std::shared_ptr<MCP3424> current_;

    MCP3424Config voltage_param_;

    MCP3424Config current_param_;

    int rate_;

    double voltage_ratio_;

    double current_ratio_;

    double current_offset_;

    double current_scale_;    

};

#endif // POWER_MONITOR_VOLTAGE_DRIVER_