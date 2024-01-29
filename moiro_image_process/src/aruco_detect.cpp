#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <memory>
#include <string>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>

using namespace std::chrono_literals;

class ArUcoMarkerDetector : public rclcpp::Node
{
    
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArUcoMarkerDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
