#include <power_monitor/power_driver.h>

VoltageDriver::VoltageDriver(const ros::NodeHandle &nh, 
                             const ros::NodeHandle &nh_private) :
    nh_(nh), nh_private_(nh_private)
{
    // load
    LoadParam();

    // setup sensor
    voltage_ = std::make_shared<MCP3424>(&voltage_param_);
    current_ = std::make_shared<MCP3424>(&current_param_);

    // setup ros
    power_pub_ = nh_.advertise<mvp_msgs::Power>("power_monitor/power",10);
}

void VoltageDriver::LoadParam() {
    // sensor configuration for voltage
    std::string voltage_address;
    int voltage_gain, voltage_bitrate, voltage_conv_mode, voltage_channel;
    nh_private_.param<std::string> ("voltage/address", voltage_address, DEFAULT_ADC_ADDRESS);
    nh_private_.param<int> ("voltage/gain", voltage_gain, DEFAULT_ADC_GAIN);
    nh_private_.param<int> ("voltage/bitrate", voltage_bitrate, DEFAULT_ADC_BITRATE);
    nh_private_.param<int> ("voltage/mode", voltage_conv_mode, DEFAULT_ADC_MODE);
    nh_private_.param<int> ("voltage/channel", voltage_channel, DEFAULT_ADC_CHANNEL);

    voltage_param_.address = std::stoul(voltage_address, nullptr, 16);
    voltage_param_.gain = (uint8_t)voltage_gain;
    voltage_param_.bitrate = (uint8_t)voltage_bitrate;
    voltage_param_.conv_mode = (uint8_t)voltage_conv_mode;
    voltage_param_.channel = (uint8_t)voltage_channel;

    // sensor configuration for current
    std::string current_address;
    int current_gain, current_bitrate, current_conv_mode, current_channel;
    nh_private_.param<std::string> ("current/address", current_address, DEFAULT_ADC_ADDRESS);
    nh_private_.param<int> ("current/gain", current_gain, DEFAULT_ADC_GAIN);
    nh_private_.param<int> ("current/bitrate", current_bitrate, DEFAULT_ADC_BITRATE);
    nh_private_.param<int> ("current/mode", current_conv_mode, DEFAULT_ADC_MODE);
    nh_private_.param<int> ("current/channel", current_channel, DEFAULT_ADC_CHANNEL);

    current_param_.address = std::stoul(current_address, nullptr, 16);
    current_param_.gain = (uint8_t)current_gain;
    current_param_.bitrate = (uint8_t)current_bitrate;
    current_param_.conv_mode = (uint8_t)current_conv_mode;
    current_param_.channel = (uint8_t)current_channel;

    // system configuration
    nh_private_.param<int> ("system/rate", rate_, DEFAULT_ADC_RATE);
    nh_private_.param<int> ("system/voltage_multiplier", voltage_multiplier_, DEFAULT_ADC_MULTIPLIER);
    nh_private_.param<double> ("system/current_offset", current_offset_, DEFAULT_ADC_OFFSET);
    nh_private_.param<double> ("system/current_scale", current_scale_, DEFAULT_ADC_SCALE);

    // ros configuration
    nh_private_.param<std::string> ("frame_id", frame_id_, DEFAULT_ROS_FRAME_ID);
}

int VoltageDriver::GetRate() {
    return rate_;
}

void VoltageDriver::Refresh() {
    auto voltage_data = voltage_->readVoltage() * voltage_multiplier_;
    auto current_data = ( current_->readVoltage() * voltage_multiplier_ - current_offset_ ) / current_scale_;

    // printf("voltage: %f\n", voltage_data);
    // printf("current: %f\n", current_data);

    mvp_msgs::Power msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id_;
    msg.voltage = voltage_data;
    msg.current = current_data;

    power_pub_.publish(msg);
}