#ifndef POWER_MONITOR_VOLTAGE_DRIVER_
#define POWER_MONITOR_VOLTAGE_DRIVER_

#include <memory>
#include <string>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

#include <power_monitor/MCP3424.h>
#include <power_monitor/default.h>

#include <chrono>

class VoltageDriver
{
public:
    VoltageDriver(const ros::NodeHandle &nh, 
                  const ros::NodeHandle &nh_private);

    ~VoltageDriver() {}

    void Refresh();

    int GetRate();

    void CallbackTimer(const ros::TimerEvent& event);
    
private:

    void LoadParam();

    ros::NodeHandle nh_;

    ros::NodeHandle nh_private_;

    ros::Publisher power_pub_;
 
    ros::Timer timer_;

    std::string frame_id_;

    std::shared_ptr<MCP3424> voltage_;

    // std::shared_ptr<MCP3424> current_;

    MCP3424Config voltage_param_;

    MCP3424Config current_param_;

    int rate_;

    double voltage_ratio_;

    double current_ratio_;

    double current_offset_;

    double current_scale_;
};

#endif // POWER_MONITOR_VOLTAGE_DRIVER_