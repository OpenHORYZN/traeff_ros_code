#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <algorithm>
#include <cstddef>
#include <cstdio>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <queue>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <unordered_map>
#include <utility>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "std_msgs/msg/empty.hpp"
#include <std_msgs/msg/empty.hpp>
#include "std_msgs/msg/float64.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
  #include <cv_bridge/cv_bridge.hpp>
#elif __has_include(<cv_bridge/cv_bridge.h>)
  #include <cv_bridge/cv_bridge.h>
#else
  #error "cv_bridge header not found"
#endif

using namespace nav_msgs::msg;
using namespace sensor_msgs::msg;


class MainNode : public rclcpp::Node {
    private:
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_realsense_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_pub_;
        rclcpp::Timer::SharedPtr base_timer_;

        Image::ConstSharedPtr latest_img_;

        double foto_period = 0;

        void realsense_cb(Image::ConstSharedPtr img_msg) {
            latest_img_ = img_msg;
        }
    public:
        MainNode() : rclcpp::Node("ManualNode"){
          this->declare_parameter("foto_period",15);

          foto_period = this->get_parameter("foto_period").as_double();
          this->img_sub_realsense_ = this->create_subscription(
            ""
          );

          // Call every 15 seconds and republish latest image
        }
}