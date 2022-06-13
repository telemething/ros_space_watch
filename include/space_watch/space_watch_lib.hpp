#pragma once

#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include <opencv2/core.hpp>     // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/imgproc.hpp>  // Gaussian Blur

using namespace rclcpp;

namespace space_watch
{

class space_watch_lib : public Node
{
public:
    space_watch_lib(const NodeOptions& options);

private:

    cv::Scalar getMSSIM( const cv::Mat& i1, const cv::Mat& i2);
    double getPSNR(const cv::Mat& I1, const cv::Mat& I2);

    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg);
    void shutdown_callback();

    //Publisher<barcode_msgs::msg::ScanResult>::SharedPtr m_scan_result_pub;
    image_transport::Subscriber m_image_sub;

    rclcpp::Time last_time_;
    std::chrono::nanoseconds period_;
    double msgs_per_sec_ = 2.0;
};

} // namespace space_watch

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(space_watch::space_watch_lib)
