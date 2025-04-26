#include "sensor_node.hpp"
#include <chrono>
#include <rclcpp/rate.hpp>
#include <thread>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>

sensor::SensorNode::SensorNode(const rclcpp::NodeOptions& options): Node("sensor", options) {
    init_cam_device();
    load_cam_info();
    init_cam_param();
    init_node();
    start_camera();
}

void sensor::SensorNode::init_cam_device() {
    RCLCPP_INFO(this->get_logger(), "Starting initiating hik camera!");

    //枚举相机设备并打印数量
    MV_CC_DEVICE_INFO_LIST device_list;
    nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    RCLCPP_INFO(this->get_logger(), "Found camera count = %d", device_list.nDeviceNum);

    //找不到相机的话，每隔1s试着重新获取一次
    while (device_list.nDeviceNum == 0 && rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "No camera found!");
        RCLCPP_INFO(this->get_logger(), "Enum state: [%x]", nRet);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    }

    //使用设备中的第一个相机创建相机句柄
    MV_CC_CreateHandle(&camera_handle_, device_list.pDeviceInfo[0]);
    MV_CC_OpenDevice(camera_handle_); //打开相机句柄
    // MV_CC_GetImageInfo(camera_handle_, &img_info_); //获取相机信息
}


void sensor::SensorNode::init_node() {
    // Init convert param
    convert_param_.nWidth = img_info_.nWidthValue;
    convert_param_.nHeight = img_info_.nHeightValue;
    convert_param_.enDstPixelType = PixelType_Gvsp_BGR8_Packed;

    sensor_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/camera/" + camera_name_,
        rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());

    int target_fps = 0;
    this->declare_parameter("fps", target_fps);
    this->get_parameter("fps", target_fps);
    rate_controller_ = std::make_shared<rclcpp::Rate>(target_fps);
    RCLCPP_WARN(this->get_logger(), "fps: %d", target_fps);   
}

void sensor::SensorNode::init_cam_param() {
    rcl_interfaces::msg::ParameterDescriptor param_desc;
    MVCC_FLOATVALUE f_value;
    param_desc.integer_range.resize(1);
    param_desc.integer_range[0].step = 1;
    // Set exposure time from param file
    param_desc.description = "Exposure time in microseconds";
    MV_CC_GetFloatValue(camera_handle_, "ExposureTime", &f_value);
    param_desc.integer_range[0].from_value = f_value.fMin;
    param_desc.integer_range[0].to_value = f_value.fMax;
    int exposure_time;
    this->declare_parameter("exposure_time", 5000);
    this->get_parameter("exposure_time", exposure_time);
    MV_CC_SetFloatValue(camera_handle_, "ExposureTime", exposure_time);
    RCLCPP_INFO(this->get_logger(), "Exposure time: %d", exposure_time);

    // Set gain from param file
    param_desc.description = "Gain";
    MV_CC_GetFloatValue(camera_handle_, "Gain", &f_value);
    param_desc.integer_range[0].from_value = f_value.fMin;
    param_desc.integer_range[0].to_value = f_value.fMax;
    double gain;
    this->declare_parameter("gain", 32.0);
    this->get_parameter("gain", gain);
    MV_CC_SetFloatValue(camera_handle_, "Gain", gain);
    RCLCPP_INFO(this->get_logger(), "Gain: %f", gain);

    //Set camera feature from mfs file
    nRet = MV_CC_FeatureLoad(camera_handle_, camera_cfg_name_.c_str());
    if (MV_OK != nRet) {
        RCLCPP_WARN(this->get_logger(),
                    "fail to load camera configuration file: %s, nRet: [%x]",
                    camera_cfg_name_.c_str(),
                    nRet);
    }
    MV_CC_GetImageInfo(camera_handle_, &img_info_);
    // use bilinearity bayer interpolation
    nRet = MV_CC_SetBayerCvtQuality(camera_handle_, 1);
    if (MV_OK != nRet) {
        RCLCPP_WARN(this->get_logger(),
                    "fail to enable bilinearity bayer interpolation, nRet: [%x]",
                    nRet);
    }
}

rcl_interfaces::msg::SetParametersResult sensor::SensorNode::parameters_callback(
    const std::vector<rclcpp::Parameter>& parameters) //参数回调函数，有参数修改时会被触发
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto& param: parameters) {
        if (param.get_name() == "exposure_time") { //modify exposure time
            int status = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", param.as_int());
            RCLCPP_INFO(this->get_logger(), "Exposure time set to: %ld", param.as_int());
            if (MV_OK != status) {
                result.successful = false;
                result.reason = "Failed to set exposure time, status = " + std::to_string(status);
            }
        } else if (param.get_name() == "gain") { //modify gain
            int status = MV_CC_SetFloatValue(camera_handle_, "Gain", param.as_double());
            RCLCPP_INFO(this->get_logger(), "Exposure time set to: %f", param.as_double());
            if (MV_OK != status) {
                result.successful = false;
                result.reason = "Failed to set gain, status = " + std::to_string(status);
            }
        } else {
            result.successful = false;
            result.reason = "Unknown parameter: " + param.get_name();
        }
    }
    return result;
}

void sensor::SensorNode::load_cam_info() {
    // Load camera name
    this->declare_parameter("camera_name", "");
    this->get_parameter("camera_name", camera_name_);
    RCLCPP_INFO(this->get_logger(), "Camera Name: %s", camera_name_.c_str());

    //load camera config file location
    this->declare_parameter("camera_config_url", "");
    this->get_parameter("camera_config_url", camera_cfg_name_);
}

void sensor::SensorNode::capture_thread_loop() {
    MV_FRAME_OUT out_frame;
    RCLCPP_INFO(this->get_logger(), "image thread started!");
    auto last_t = std::chrono::system_clock::now();
    int fps = 0;

    while (rclcpp::ok()) {
        auto img_msg = std::make_unique<sensor_msgs::msg::Image>();
        img_msg->header.stamp = this->now();
        img_msg->header.frame_id = camera_name_;
        
        nRet = MV_CC_GetImageBuffer(camera_handle_, &out_frame, 1000);
        // RCLCPP_INFO(this->get_logger(), "Get buffer status: [%x]", nRet);
        
        if (MV_OK == nRet) {
            img_msg->height = img_info_.nHeightValue;
            img_msg->width = img_info_.nWidthValue;
            img_msg->encoding = "bgr8";
            img_msg->step = img_info_.nWidthValue * 3;
            img_msg->data.resize(img_msg->height * img_msg->step);
            
            convert_param_.pDstBuffer = img_msg->data.data();
            convert_param_.nDstBufferSize = img_msg->data.size();
            convert_param_.pSrcData = out_frame.pBufAddr;
            convert_param_.nSrcDataLen = out_frame.stFrameInfo.nFrameLen;
            convert_param_.enSrcPixelType = out_frame.stFrameInfo.enPixelType;

            MV_CC_ConvertPixelType(camera_handle_, &convert_param_);
            
            sensor_pub_->publish(std::move(img_msg));

            // RCLCPP_WARN(this->get_logger(), "publish image, current fps = %d", fps);

            MV_CC_FreeImageBuffer(camera_handle_, &out_frame);
            camera_fail_count_ = 0;
            // RCLCPP_WARN(this->get_logger(), "BEFORE SLEEP");
            rate_controller_->sleep();
            // RCLCPP_WARN(this->get_logger(), "AFTER SLEEP");
            
            auto t = std::chrono::system_clock::now();
            ++fps;
            if (t - last_t >= std::chrono::seconds(1)) {
                RCLCPP_INFO(this->get_logger(),
                            "publish image, current fps = %d",
                            fps);
                fps = 0;
                last_t = t;
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Get buffer failed! nRet: [%x]", nRet);
            MV_CC_StopGrabbing(camera_handle_);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            MV_CC_StartGrabbing(camera_handle_);
            RCLCPP_WARN(this->get_logger(), "Cam restarted! trying to get frame again...");
            camera_fail_count_++;
        }

        if (camera_fail_count_ > 5) {
            RCLCPP_FATAL(this->get_logger(), "Camera failed!");
            rclcpp::shutdown();
        }
    }
}


void sensor::SensorNode::close_camera() {
    if (capture_thread_.joinable()) {
        capture_thread_.join();
    }
    if (camera_handle_ != nullptr) {
        MV_CC_StopGrabbing(camera_handle_);
        MV_CC_CloseDevice(camera_handle_);
        MV_CC_DestroyHandle(&camera_handle_);
    }
    RCLCPP_INFO(this->get_logger(), "camera successfully closed!");
}


void sensor::SensorNode::start_camera() {
    //相机开始抓取
    MV_CC_StartGrabbing(camera_handle_);
    capture_thread_ = std::thread(&SensorNode::capture_thread_loop, this);
}


sensor::SensorNode::~SensorNode() {
    // if (this->recorder_init_thread.joinable()) {
    //     this->recorder_init_thread.join();
    // }
    // this->recorder_inited = false;
    close_camera();
}

//register the node to workspace
RCLCPP_COMPONENTS_REGISTER_NODE(sensor::SensorNode)