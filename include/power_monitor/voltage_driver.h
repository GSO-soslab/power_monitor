#ifndef POWER_MONITOR_VOLTAGE_DRIVER_
#define POWER_MONITOR_VOLTAGE_DRIVER_

#include <memory>
#include <string>
#include <ros/ros.h>
#include <mvp_msgs/Power.h>
#include <power_monitor/MCP3424.h>
#include <power_monitor/default.h>

class VoltageDriver
{
public:
    VoltageDriver(const ros::NodeHandle &nh, 
                  const ros::NodeHandle &nh_private);

    ~VoltageDriver() {}

    void Refresh();

    int GetRate();

private:

    void LoadParam();

    ros::NodeHandle nh_;

    ros::NodeHandle nh_private_;

    ros::Publisher voltage_pub_;
 
    std::shared_ptr<MCP3424> adc_;

    MCP3424Config adc_param_;

    int rate_;

    int multiplier_;

    std::string voltage_topic_;

    std::string frame_id_;
};

#endif // POWER_MONITOR_VOLTAGE_DRIVER_