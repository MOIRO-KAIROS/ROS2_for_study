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

class MoiroObjTracking : public rclcpp::Node
{
public:
    MoiroObjTracking() : Node("moiro_obj_tracking")
    {
        // Publisher
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("moiro_obj_tracking", 10);
        // Subscriber
        subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", 10, std::bind(&MoiroObjTracking::obj_tracking, this, std::placeholders::_1));
        // Timer
        timer_ = this->create_wall_timer(500ms, std::bind(&MoiroObjTracking::timer_callback, this));
        // Node Start Message
        RCLCPP_INFO(this->get_logger(), "MoiroObjTracking has been started.");
    } 
    
private:
    void obj_tracking(const sensor_msgs::msg::Image::SharedPtr msg){
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
        cv::Mat hsv_img;
        cv::cvtColor(img, hsv_img, cv::COLOR_BGR2HSV);
        cv::Mat mask_img;
        cv::inRange(hsv_img, lower, upper, mask_img);
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
        // std::vector<cv::Point> max_contour;
        // double max_area = 0;
        // for (auto contour : contours)
        // {
        //     double area = cv::contourArea(contour);
        //     if (max_area < area)
        //     {
        //         max_area = area;
        //         max_contour = contour;
        //     }
        // }
        cv::drawContours(img, contours, -1, cv::Scalar(0, 255, 0), 2);
        img_ = img;
        // image show
        cv::imshow("img", img);
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
    cv::Mat lower = (cv::Mat_<int>(1, 3) << 160, 100, 100);
    cv::Mat upper = (cv::Mat_<int>(1, 3) << 180, 255, 255);
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoiroObjTracking>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}