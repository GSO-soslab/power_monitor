#include <power_monitor/power_driver.h>

VoltageDriver::VoltageDriver(const ros::NodeHandle &nh, 
                             const ros::NodeHandle &nh_private) :
    nh_(nh), nh_private_(nh_private)
{
    // load
    LoadParam();

    // setup sensor
    voltage_ = std::make_shared<MCP3424>(&voltage_param_);
    // current_ = std::make_shared<MCP3424>(&current_param_);

    // setup ros
    power_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("power_monitor/power",10);
    timer_ = nh_.createTimer(ros::Duration(1.0/rate_), &VoltageDriver::CallbackTimer, this);
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
    nh_private_.param<double> ("system/voltage_ratio", voltage_ratio_, DEFAULT_ADC_RATIO);
    nh_private_.param<double> ("system/current_ratio", current_ratio_, DEFAULT_ADC_RATIO);
    nh_private_.param<double> ("system/current_offset", current_offset_, DEFAULT_ADC_OFFSET);
    nh_private_.param<double> ("system/current_scale", current_scale_, DEFAULT_ADC_SCALE);

    // ros configuration
    nh_private_.param<std::string> ("frame_id", frame_id_, DEFAULT_ROS_FRAME_ID);
}

int VoltageDriver::GetRate() {
    return rate_;
}

void VoltageDriver::CallbackTimer(const ros::TimerEvent& event) {
    auto curr_time = ros::Time::now().toSec();

    // auto t1 = std::chrono::high_resolution_clock::now(); 
    Refresh();
    // auto t2 = std::chrono::high_resolution_clock::now(); 
    // printf("[Refresh Time Cost]: %.6fs\n", 
    //     std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() * 1e-6);    
}

void VoltageDriver::Refresh() {
    // auto t1 = std::chrono::high_resolution_clock::now(); 
    auto voltage_data = voltage_->readVoltage() / voltage_ratio_;
    // auto t2 = std::chrono::high_resolution_clock::now(); 
    auto current_data = 0.0;
    // auto current_data = ( current_->readVoltage() / current_ratio_ - current_offset_ ) / current_scale_;
    // auto t3 = std::chrono::high_resolution_clock::now(); 
    // printf("[Inside Time Cost]: voltage=%.6fs, current=%.6fs\n", 
    //     std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() * 1e-6,
    //     std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count() * 1e-6);

    // printf("voltage: %f\n", voltage_data);
    // printf("current: %f\n", current_data);

    std_msgs::Float32MultiArray msg;

    // Set the dimensions
    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());

    // Example: 1D array (1x2 matrix)
    msg.layout.dim[0].label = "voltage_current";
    msg.layout.dim[0].size = 2;
    msg.layout.dim[0].stride = 2;  

    msg.layout.data_offset = 0;  // Offset from the start

    // Fill the array with data (1x2 matrix)
    msg.data.clear();
    msg.data.push_back(voltage_data);
    msg.data.push_back(current_data);

    // Publish the message
    power_pub_.publish(msg);
}