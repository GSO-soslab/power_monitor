#include <power_monitor/power_monitor.h>

PowerMonitor::PowerMonitor() : Node("power_monitor_node") 
{
    // setup the parameters
    LoadParam();

    // setup sensor
    voltage_ = std::make_shared<MCP3424>(&voltage_param_);
    current_ = std::make_shared<MCP3424>(&current_param_);

    // setup the ros
    timer_ = this->create_wall_timer( 
        std::chrono::milliseconds(1000/rate_), 
        std::bind(&PowerMonitor::CallbackTimer, this));

    // publisher_ = this->create_publisher<mvp_msgs::msg::Power>("~/power_monitor", 10);  
    publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("~/power_monitor", 10);    
}

void PowerMonitor::CallbackTimer() 
{
    RCLCPP_INFO(this->get_logger(), "Hello %d!", rate_);

    auto voltage_data = voltage_->readVoltage() / voltage_ratio_;
    auto current_data = ( current_->readVoltage() / current_ratio_ - current_offset_ ) / current_scale_;
    //! DEBUG:
    RCLCPP_INFO(this->get_logger(), "voltage: %f!", voltage_data);
    RCLCPP_INFO(this->get_logger(), "current: %f!", current_data);


    // publish
    // auto message = mvp_msgs::msg::Power();
    auto message = std_msgs::msg::Float64MultiArray();
    // message.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    // message.header.frame_id = frame_id_;
    message.data[0] = voltage_data;
    message.data[1] = current_data;
    // message.voltage = voltage_data;
    // message.current = current_data;

    publisher_->publish(message);
}

void PowerMonitor::LoadParam() 
{
    // =================== params for voltage =================== //
    this->declare_parameter("voltage.address", DEFAULT_ADC_ADDRESS);
    this->declare_parameter("voltage.gain",    DEFAULT_ADC_GAIN);
    this->declare_parameter("voltage.bitrate", DEFAULT_ADC_BITRATE);
    this->declare_parameter("voltage.mode",    DEFAULT_ADC_MODE);
    this->declare_parameter("voltage.channel", DEFAULT_ADC_CHANNEL);

    std::string voltage_address;
    int voltage_gain, voltage_bitrate, voltage_conv_mode, voltage_channel;

    this->get_parameter("voltage.address", voltage_address);
    this->get_parameter("voltage.gain",    voltage_gain);
    this->get_parameter("voltage.bitrate", voltage_bitrate);
    this->get_parameter("voltage.mode",    voltage_conv_mode);
    this->get_parameter("voltage.channel", voltage_channel);

    voltage_param_.address = std::stoul(voltage_address, nullptr, 16);
    voltage_param_.gain = (uint8_t)voltage_gain;
    voltage_param_.bitrate = (uint8_t)voltage_bitrate;
    voltage_param_.conv_mode = (uint8_t)voltage_conv_mode;
    voltage_param_.channel = (uint8_t)voltage_channel;
    //! DEUBG:
    // printf("voltage: address=%d, gain=%d, bit=%d, mode=%d, chan=%d\n",
    //         voltage_param_.address, voltage_param_.gain,
    //         voltage_param_.bitrate, voltage_param_.conv_mode,
    //         voltage_param_.channel);

    // =================== params for current =================== //
    this->declare_parameter("current.address", DEFAULT_ADC_ADDRESS);
    this->declare_parameter("current.gain",    DEFAULT_ADC_GAIN);
    this->declare_parameter("current.bitrate", DEFAULT_ADC_BITRATE);
    this->declare_parameter("current.mode",    DEFAULT_ADC_MODE);
    this->declare_parameter("current.channel", DEFAULT_ADC_CHANNEL);

    std::string current_address;
    int current_gain, current_bitrate, current_conv_mode, current_channel;

    this->get_parameter("current.address", current_address);
    this->get_parameter("current.gain",    current_gain);
    this->get_parameter("current.bitrate", current_bitrate);
    this->get_parameter("current.mode",    current_conv_mode);
    this->get_parameter("current.channel", current_channel);

    current_param_.address = std::stoul(current_address, nullptr, 16);
    current_param_.gain = (uint8_t)current_gain;
    current_param_.bitrate = (uint8_t)current_bitrate;
    current_param_.conv_mode = (uint8_t)current_conv_mode;
    current_param_.channel = (uint8_t)current_channel;
    //! DEUBG:
    // printf("current: address=%d, gain=%d, bit=%d, mode=%d, chan=%d\n",
    //         current_param_.address, current_param_.gain,
    //         current_param_.bitrate, current_param_.conv_mode,
    //         current_param_.channel);

    // =================== params for system =================== //
    this->declare_parameter("system.rate",           DEFAULT_ADC_RATE);
    this->declare_parameter("system.voltage_ratio",  DEFAULT_ADC_RATIO);
    this->declare_parameter("system.current_ratio",  DEFAULT_ADC_RATIO);
    this->declare_parameter("system.current_offset", DEFAULT_ADC_OFFSET);
    this->declare_parameter("system.current_scale",  DEFAULT_ADC_SCALE);

    this->get_parameter("system.rate",           rate_);
    this->get_parameter("system.voltage_ratio",  voltage_ratio_);
    this->get_parameter("system.current_ratio",  current_ratio_);
    this->get_parameter("system.current_offset", current_offset_);
    this->get_parameter("system.current_scale",  current_scale_); 

    // =================== params for ROS =================== //
    // this->declare_parameter("ros.frame_id", DEFAULT_ROS_FRAME_ID);

    // this->get_parameter("ros.frame_id", frame_id_);
}

