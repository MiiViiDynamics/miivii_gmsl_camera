#ifndef MIIVII_GMSL_CAMERA__MIIVII_GMSL_CAMERA_HPP_
#define MIIVII_GMSL_CAMERA__MIIVII_GMSL_CAMERA_HPP_

#include "miivii_gmsl_camera/miivii_gmsl_camera.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>

#include <string.h>
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include "MvGmslCamera.h"
#include <chrono>
// #include <opencv2/opencv.hpp>
#include <cstdlib>

#include <miivii_gmsl_camera/msg/miivii_time.hpp>

using namespace std::chrono;
using std::string;

#define DEFAULT_VIDEO "/dev/video0"

#define DEFAULT_CAMERA_FORMAT "UYVY"
#define DEFAULR_CAMERA_RESOLUTION "1280x720"

#define DEFAULT_OUTPUT_FORMAT "BGRA32"
#define DEFAULR_OUTPUT_RESOLUTION "640x360"

namespace miivii_gmsl
{

    class MiiviiGmslCamera : public rclcpp::Node
    {
    public:
        explicit MiiviiGmslCamera(rclcpp::NodeOptions const &options);
        virtual ~MiiviiGmslCamera();
        void timer_callback();

    private:
        camera_context_t ctx[8] = {};
        sync_out_a_cfg_client_t stCameraCfgSend;
        std::string g_camera_dev = "NONE";

        double publish_rate;
        int camera_num = 8;
        int active_camera_num;
        int sync_camera_num;
        int async_camera_num;
        int sync_camera_freq;
        int async_camera_freq;
        uint sync_camera_trigger;
        uint async_camera_trigger;
        std::vector<int64_t> async_angle;
        miivii::MvGmslCamera *mvcam;

        std::vector<sensor_msgs::msg::CameraInfo> camera_info_msg_;

        rclcpp::Time last_time;

        void DeclareParameters();
        void GetParameters();
        sensor_msgs::msg::CameraInfo createCameraInfoFromYAML(const std::string &filepath);
        void CreateCtx();
        void CreateMvGmslCamera();
        void StartTimer();
        void SpiltResolution(std::string camera_resolution, uint *width, uint *height);

        bool fileExists(const std::string &filename);

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_[8]; // 图像消息发布者指针
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher_[8];
        rclcpp::TimerBase::SharedPtr image_publisher_timer_; // 定时器指针，用于定期发布图像消息

        rclcpp::Publisher<miivii_gmsl_camera::msg::MiiviiTime>::SharedPtr time_publisher_;
    };
}

#endif
