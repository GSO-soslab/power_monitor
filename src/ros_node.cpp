#include <power_monitor/MCP3424.h>
#include <power_monitor/power_driver.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "Power_Monitor_Node"); 
  
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  VoltageDriver node(nh, nh_private);

  ros::Rate loop_rate(node.GetRate());

  while(ros::ok()) {
    node.Refresh();

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}