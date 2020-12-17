#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>
#include <signal.h>
#include <std_srvs/Empty.h>
#include <sys/stat.h>
#include <unistd.h>
#include <sstream>
#include "MvGmslCamera.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#define TEST 0
#define MAX_CAMERA_NUM 4

using namespace std;
namespace miivii {

class miivii_gmsl_ros_node
{
public:
    // private ROS node handle
    ros::NodeHandle node_;

    // MiiVii sdk handler
    MvGmslCamera *m_gmsl_camera;

    // manager to handle all calibration files
    camera_info_manager::CameraInfoManager *info_manager_;

    // camera global parameters
    int camWidth, camHeight, fps, camCount;
    int enable_sync_;
    std::string domain_name, dev_name;

    // camera local parameters
    std::string camera_name_[MAX_CAMERA_NUM];
    std::string camera_topic_list[MAX_CAMERA_NUM];
    std::string camera_info_topic_list[MAX_CAMERA_NUM];
    std::string camera_cali_file_list[MAX_CAMERA_NUM];

    image_transport::CameraPublisher gmsl_image_pub_[MAX_CAMERA_NUM];
    sensor_msgs::CameraInfo gmsl_camera_info[MAX_CAMERA_NUM];
    std::string gmsl_camera_frame_id[MAX_CAMERA_NUM];

    uint8_t *gmsl_outbuf[MAX_CAMERA_NUM];

    miivii_gmsl_ros_node() : node_("~") {
    ROS_INFO("starting to run gmsl camera construct function");

    info_manager_ = new camera_info_manager::CameraInfoManager(node_);

    // grab the parameters
    node_.param("video_device", dev_name, std::string("/dev/video0"));
    node_.param("image_height", camHeight, 720);
    node_.param("image_width", camWidth, 1280);

    node_.param("camCount", camCount, 1);
    if (camCount > MAX_CAMERA_NUM || camCount < 0) {
        camCount = 1;
    }

    uint camCount_, camWidth_, camHeight_, fps_;
    camCount_ = camCount;
    camWidth_ = camWidth;
    camHeight_ = camHeight;

    node_.param("fps", fps, 25);
    fps_ = fps;

    node_.param("domain", domain_name, std::string("miivii_gmsl_camera"));

    for (int i = 0; i < camCount; i++)
    {
        // setup camera image topic
        node_.param(
            std::string("camera") + std::to_string(i + 1) + std::string("_topic"),
            camera_topic_list[i], std::string("camera") + std::to_string(i + 1));

        // setup camera name
        node_.param(
            std::string("camera") + std::to_string(i + 1) + std::string("_name"),
            camera_name_[i], std::string("gmsl_camera") + std::to_string(i + 1));

        // setup camera info topic
        node_.param(
            std::string("camera") + std::to_string(i + 1) +
                std::string("_info_topic"),
            camera_info_topic_list[i],
            std::string("camera") + std::to_string(i + 1) + std::string("_info"));

        // setup camera calibration file
        node_.param(std::string("camera") + std::to_string(i + 1) +
                        std::string("_cali_file"),
                    camera_cali_file_list[i],
                    std::string("camera") + std::to_string(i + 1));
        // setup camera calibration file
        node_.param(std::string("camera") + std::to_string(i + 1) +
                        std::string("_frame_id"),
                    gmsl_camera_frame_id[i],
                    std::string("camera") + std::to_string(i + 1));
    }

    node_.param("enable_sync", enable_sync_, 1);

    ROS_INFO("dev_name = %s   camWidth = %d   camHeight = %d    fps = %d",
            dev_name.c_str(), camWidth, camHeight, fps);

    m_gmsl_camera = new MvGmslCamera(dev_name, camCount_,
                                        camWidth_,
                                        camHeight_,
                                        fps_);

    // advertise the main image topic
    image_transport::ImageTransport it(node_);

    for (int i = 0; i < camCount; i++)
    {
        gmsl_image_pub_[i] =  it.advertiseCamera((domain_name + camera_topic_list[i]).c_str(), 1);

        ROS_INFO("camera_cali_file_list %d %s", i, camera_cali_file_list[i].c_str());
        info_manager_->resolveURL(camera_cali_file_list[i], camera_name_[i]);

        if (info_manager_->validateURL(camera_cali_file_list[i]))
        {
            info_manager_->setCameraName(camera_name_[i]);
            info_manager_->loadCameraInfo( camera_cali_file_list[i]);
            gmsl_camera_info[i] = info_manager_->getCameraInfo();
        }
        else
        {
            ROS_WARN(
                "fail to load calibration file for camera %s, fallback to empty "
                "calibration",
                camera_topic_list[i].c_str());
        }
    }

    for (int i = 0; i < MAX_CAMERA_NUM; i++)
    {
        gmsl_outbuf[i] = NULL;
    }
}

virtual ~miivii_gmsl_ros_node()
{
    if (m_gmsl_camera != NULL)
    {
        delete m_gmsl_camera;
        m_gmsl_camera = NULL;
    }
}

bool grab_and_send_image()
{
    uint64_t timestap;
    if (m_gmsl_camera->GetImagePtr(gmsl_outbuf, timestap))
    {
        ros::Time Ts;
        if (enable_sync_ == 1)
        {
            Ts.fromNSec(timestap);
        }
        else
        {
            Ts = ros::Time::now();
        }

        for (int i = 0; i < camCount; i++)
        {
            // pub camera image
            cv_bridge::CvImage out_msg;
            out_msg.header.frame_id = gmsl_camera_frame_id[i];
            out_msg.header.stamp = Ts;
            out_msg.encoding = sensor_msgs::image_encodings::RGBA8;
            out_msg.image = cv::Mat(camHeight, camWidth, CV_8UC4, gmsl_outbuf[i]);
            // pub camera info
            gmsl_camera_info[i].header.stamp = Ts;
            gmsl_camera_info[i].header.frame_id = gmsl_camera_frame_id[i];
            sensor_msgs::CameraInfoPtr ci(
                new sensor_msgs::CameraInfo(gmsl_camera_info[i]));
            gmsl_image_pub_[i].publish(out_msg.toImageMsg(), ci);
        }
        return true;
    }
    return false;
}

bool spin()
{
    ros::Rate loop_rate(fps);
    while (node_.ok())
    {
        if (!grab_and_send_image())
            ROS_WARN("gmsl camera did not respond in time.");

        ros::spinOnce();
        loop_rate.sleep();
    }
    return true;
}
};

}  // namespace miivii

int main(int argc, char **argv)
{
    ros::init(argc, argv, "miivii_gmsl_ros");

    miivii::miivii_gmsl_ros_node node;

    node.spin();
    return EXIT_SUCCESS;
}
