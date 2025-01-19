#ifndef SENSOR_NODE_H
#define SENSOR_NODE_H

#include "../hikSDK/include/MvCameraControl.h"
// ROS
#include "std_msgs/msg/string.hpp"
#include <memory>
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <stack>

// namespace hik_camera
namespace sensor {
class SensorNode: public rclcpp::Node {
public:
    explicit SensorNode(const rclcpp::NodeOptions& options);
    ~SensorNode() override;

private:
    //初始化
    void init_cam_device();
    // void init_imu_device();
    void init_node();
    void init_cam_param();
    void load_cam_info();
    //线程循环
    void capture_thread_loop();

    //启动线程
    void start_camera();

    //退出清理
    void close_camera();

    //参数回调函数，有参数修改时会被触发
    rcl_interfaces::msg::SetParametersResult
    parameters_callback(const std::vector<rclcpp::Parameter>& parameters);
    //参数回调句柄
    OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;

    //相机相关变量
    int camera_fail_count_ = 0; //相机错误计数
    int nRet = MV_OK; //camera status code from hiSDk
    void* camera_handle_; // ptr to camera
    MV_IMAGE_BASIC_INFO img_info_; //info of image
    MV_CC_PIXEL_CONVERT_PARAM convert_param_; //camera conversion param
    std::string camera_name_;
    std::string camera_cfg_name_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr sensor_pub_;


    //该节点的线程
    std::thread capture_thread_; //抓取线程
    std::shared_ptr<rclcpp::Rate> rate_controller_;
    int sync_period_ms = 10;
    std::atomic<bool> recorder_inited = false;
};
} // namespace sensor

#endif // SENSOR_NODE_H