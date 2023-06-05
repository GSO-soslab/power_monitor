#include <microchip_voltage/voltage_driver.h>

VoltageDriver::VoltageDriver(const ros::NodeHandle &nh, 
                             const ros::NodeHandle &nh_private) :
    nh_(nh), nh_private_(nh_private)
{

}

VoltageDriver::LoadParam() {
  
}
