#include <rclcpp/rclcpp.hpp>
#include "miivii_gmsl_camera/miivii_gmsl_camera.hpp"
#include <csignal>  

std::shared_ptr<miivii_gmsl::MiiviiGmslCamera> node = nullptr;
bool quit = false;
bool request = false;


void signal_handler(int signal) {
  RCLCPP_INFO(node->get_logger(), "Received SIGINT, shutting down...");
  request = true;
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv); // 初始化ROS节点和参数服务器
  node = std::make_shared<miivii_gmsl::MiiviiGmslCamera>(rclcpp::NodeOptions{});

  signal(SIGINT, signal_handler);

  while(!quit){
    if(request){
      RCLCPP_ERROR(node->get_logger(), "Exiting!!");
      quit = true;
      break;
    }
    node->timer_callback();
  }

  // rclcpp::spin(node); // 运行节点并保持活动状态，直到被关闭或异常发生
  rclcpp::shutdown(); // 关闭ROS节点和清理资源
  node = nullptr;
  return 0;
}
