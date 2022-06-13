
#include "space_watch/space_watch_lib.hpp"
#include <functional>
#include "cv_bridge/cv_bridge.h"

namespace space_watch
{

space_watch_lib::space_watch_lib(const NodeOptions& options)
:   Node("space_watch_lib", options)
{
    on_shutdown([&]
    {
        space_watch_lib::shutdown_callback();
    });

    // init time
    period_ = rclcpp::Rate(msgs_per_sec_).period();
    last_time_ = this->now();

    using std::placeholders::_1;

    m_image_sub = image_transport::create_subscription(this, "image_raw",
        std::bind(&space_watch_lib::image_callback, this, _1),
        "raw", rmw_qos_profile_sensor_data);

    //m_scan_result_pub = this->create_publisher<barcode_msgs::msg::ScanResult>("barcodes",
    //    QoS(10));
}

cv::Scalar space_watch_lib::getMSSIM( const cv::Mat& i1, const cv::Mat& i2)
{
    const double C1 = 6.5025, C2 = 58.5225;
    /***************************** INITS **********************************/
    int d = CV_32F;
    cv::Mat I1, I2;
    i1.convertTo(I1, d);            // cannot calculate on one byte large values
    i2.convertTo(I2, d);
    cv::Mat I2_2   = I2.mul(I2);        // I2^2
    cv::Mat I1_2   = I1.mul(I1);        // I1^2
    cv::Mat I1_I2  = I1.mul(I2);        // I1 * I2
    /*************************** END INITS **********************************/
    cv::Mat mu1, mu2;                   // PRELIMINARY COMPUTING
    GaussianBlur(I1, mu1, cv::Size(11, 11), 1.5);
    GaussianBlur(I2, mu2, cv::Size(11, 11), 1.5);
    cv::Mat mu1_2   =   mu1.mul(mu1);
    cv::Mat mu2_2   =   mu2.mul(mu2);
    cv::Mat mu1_mu2 =   mu1.mul(mu2);
    cv::Mat sigma1_2, sigma2_2, sigma12;
    GaussianBlur(I1_2, sigma1_2, cv::Size(11, 11), 1.5);
    sigma1_2 -= mu1_2;
    GaussianBlur(I2_2, sigma2_2, cv::Size(11, 11), 1.5);
    sigma2_2 -= mu2_2;
    GaussianBlur(I1_I2, sigma12, cv::Size(11, 11), 1.5);
    sigma12 -= mu1_mu2;
    cv::Mat t1, t2, t3;
    t1 = 2 * mu1_mu2 + C1;
    t2 = 2 * sigma12 + C2;
    t3 = t1.mul(t2);                 // t3 = ((2*mu1_mu2 + C1).*(2*sigma12 + C2))
    t1 = mu1_2 + mu2_2 + C1;
    t2 = sigma1_2 + sigma2_2 + C2;
    t1 = t1.mul(t2);                 // t1 =((mu1_2 + mu2_2 + C1).*(sigma1_2 + sigma2_2 + C2))
    cv::Mat ssim_map;
    divide(t3, t1, ssim_map);        // ssim_map =  t3./t1;
    cv::Scalar mssim = mean(ssim_map);   // mssim = average of ssim map
    return mssim;
}

double space_watch_lib::getPSNR(const cv::Mat& I1, const cv::Mat& I2)
{
    cv::Mat s1;
    absdiff(I1, I2, s1);       // |I1 - I2|
    s1.convertTo(s1, CV_32F);  // cannot make a square on 8 bits
    s1 = s1.mul(s1);           // |I1 - I2|^2
    cv::Scalar s = sum(s1);        // sum elements per channel
    double sse = s.val[0] + s.val[1] + s.val[2]; // sum channels
    if( sse <= 1e-10) // for small values return zero
        return 0;
    else
    {
        double mse  = sse / (double)(I1.channels() * I1.total());
        double psnr = 10.0 * log10((255 * 255) / mse);
        return psnr;
    }
}

void space_watch_lib::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg)
{
    //****************************
    const auto & now = this->now();

    if (last_time_ > now) 
    {
      //RCLCPP_WARN(
      //  get_logger(), "Detected jump back in time, resetting throttle period to now for.");
      last_time_ = now;
    }

    if ((now - last_time_).nanoseconds() >= period_.count()) 
    {
      //pub_->publish(*msg);
      last_time_ = now;
    }
    else 
    {
      return;
    }

    //****************************

    const cv::Mat cv_image = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::MONO8)->image;


} // space_watch_lib::image_callback()

void space_watch_lib::shutdown_callback()
{

}

} // namespace space_watch
