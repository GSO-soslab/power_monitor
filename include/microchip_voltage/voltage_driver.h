#ifndef MICROCHIP_VOLTAGE_VOLTAGE_DRIVER_
#define MICROCHIP_VOLTAGE_VOLTAGE_DRIVER_

#include <memory>
#include <ros/ros.h>
#include <mvp_msgs/Power.h>
#include <microchip_voltage/MCP3424.h>

class VoltageDriver
{
public:
    VoltageDriver(const ros::NodeHandle &nh, 
                  const ros::NodeHandle &nh_private);

    ~VoltageDriver() {}

    void Refresh();

private:

    void LoadParam();

    ros::NodeHandle nh_;

    ros::NodeHandle nh_private_;

    ros::Publisher voltage_pub_;
 
    std::shared_ptr<MCP3424> adc_;
};

#endif // MICROCHIP_VOLTAGE_VOLTAGE_DRIVER_