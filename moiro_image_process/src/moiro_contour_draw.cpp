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

class MoiroContourDraw : public rclcpp::Node
{
public:
    MoiroContourDraw() : Node("moiro_contour_draw")
    {
        // Publisher
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("moiro_contour_draw", 10);
        // Subscriber
        subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", 10, std::bind(&MoiroContourDraw::draw_contour, this, std::placeholders::_1));
        // Timer
        timer_ = this->create_wall_timer(500ms, std::bind(&MoiroContourDraw::timer_callback, this));
        // Node Start Message
        RCLCPP_INFO(this->get_logger(), "MoiroContourDraw has been started.");
    }

private:
    void draw_contour(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat img = cv_ptr->image;
        cv::Mat gray_img;
        cv::cvtColor(img, gray_img, cv::COLOR_BGR2GRAY);
        cv::Mat bin_img;
        cv::threshold(gray_img, bin_img, 127, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(bin_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
        cv::drawContours(img, contours, -1, cv::Scalar(0, 0, 255), 2);
        img_ = img;
    }

    void timer_callback()
    {
        if (img_.empty())
        {
            return;
        }
        cv_bridge::CvImage cv_img;
        cv_img.image = img_;
        cv_img.encoding = "bgr8";
        cv_img.header.stamp = this->now();
        publisher_->publish(*cv_img.toImageMsg());
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::Mat img_;
};

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MoiroContourDraw>());
    rclcpp::shutdown();
    return 0;
}