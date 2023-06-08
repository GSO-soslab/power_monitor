#include <power_monitor/voltage_driver.h>

VoltageDriver::VoltageDriver(const ros::NodeHandle &nh, 
                             const ros::NodeHandle &nh_private) :
    nh_(nh), nh_private_(nh_private)
{
    // load
    LoadParam();

    // setup sensor
    adc_ = std::make_shared<MCP3424>(&adc_param_);

    // setup ros
    voltage_pub_ = nh_.advertise<mvp_msgs::Power>("power_monitor/voltage",10);
}

void VoltageDriver::LoadParam() {
    // sensor configuration
    std::string address;
    int gain, bitrate, conv_mode, channel;
    nh_private_.param<std::string> ("sensor/address", address, DEFAULT_ADC_ADDRESS);
    nh_private_.param<int> ("sensor/gain", gain, DEFAULT_ADC_GAIN);
    nh_private_.param<int> ("sensor/bitrate", bitrate, DEFAULT_ADC_BITRATE);
    nh_private_.param<int> ("sensor/mode", conv_mode, DEFAULT_ADC_MODE);
    nh_private_.param<int> ("sensor/channel", channel, DEFAULT_ADC_CHANNEL);

    adc_param_.address = std::stoul(address, nullptr, 16);
    adc_param_.gain = (uint8_t)gain;
    adc_param_.bitrate = (uint8_t)bitrate;
    adc_param_.conv_mode = (uint8_t)conv_mode;
    adc_param_.channel = (uint8_t)channel;

    // system configuration
    nh_private_.param<int> ("system/rate", rate_, DEFAULT_ADC_RATE);
    nh_private_.param<int> ("system/multiplier", multiplier_, DEFAULT_ADC_RATE);

    // ros configuration
    nh_private_.param<std::string> ("frame_id", frame_id_, DEFAULT_ROS_FRAME_ID);
}

int VoltageDriver::GetRate() {
    return rate_;
}

void VoltageDriver::Refresh() {
    auto data = adc_->readVoltage() * multiplier_;

    // printf("voltage: %f\n", data);

    mvp_msgs::Power msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id_;
    msg.voltage = data;

    voltage_pub_.publish(msg);
}