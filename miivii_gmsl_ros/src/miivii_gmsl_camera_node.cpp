#include "miivii_gmsl_camera/miivii_gmsl_camera.hpp"
#include <rclcpp/rclcpp.hpp>
#include <csignal>
#include <thread>
#include <atomic>

std::shared_ptr<miivii_gmsl::MiiviiGmslCamera> node = nullptr;
std::atomic<bool> quit{false};
std::atomic<bool> request{false};

void signal_handler(int signal) {
    RCLCPP_INFO(rclcpp::get_logger("main"), "Received %d, shutting down...", signal);
    request = true;
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    
    // 创建节点
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "--log-level", "info"});
    node = std::make_shared<miivii_gmsl::MiiviiGmslCamera>(options);
    
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    
    uint64_t loop_count = 0;
    
    // 主循环
    while (!quit && rclcpp::ok()) {
        if (request) {
            RCLCPP_INFO(node->get_logger(), "Shutdown requested");
            quit = true;
            break;
        }
        try {
            // 使用try-catch避免异常导致程序退出
            node->timer_callback();
            loop_count++;
            // 每100次循环打印一次状态
            if (loop_count % 100 == 0) {
                RCLCPP_DEBUG(node->get_logger(), "Main loop running, count: %lu", loop_count);
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node->get_logger(), "Exception in timer_callback: %s", e.what());
        }
        // 处理ROS消息（非阻塞）
        executor.spin_some();
    }
    
    RCLCPP_INFO(node->get_logger(), "Main loop exited after %lu iterations", loop_count);
    rclcpp::shutdown();
    node = nullptr;
    return 0;
}
