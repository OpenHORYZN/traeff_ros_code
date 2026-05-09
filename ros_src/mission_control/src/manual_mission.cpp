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
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
    rclcpp::TimerBase::SharedPtr base_timer_;
    rclcpp::TimerBase::SharedPtr countdown_timer_;
    Image::ConstSharedPtr latest_img_;
    double foto_period   = 15.0;
    int    countdown_sec = 0;   // counts down to next snapshot
    int    snap_count    = 0;   // total snapshots published

    void realsense_cb(Image::ConstSharedPtr img_msg) {
        if (!latest_img_) {
            RCLCPP_INFO(this->get_logger(), "✓ First image received from camera.");
        }
        latest_img_ = img_msg;
    }

    // Fires every second just for countdown prints
    void countdown_cb() {
        if (countdown_sec > 0) {
            if (countdown_sec <= 5) {
                // Last 5 seconds — print every tick
                RCLCPP_INFO(this->get_logger(),
                    "📸 Next snapshot in %d second%s...",
                    countdown_sec, countdown_sec == 1 ? "" : "s");
            } else if (countdown_sec % 5 == 0) {
                // Earlier — print every 5 seconds
                RCLCPP_INFO(this->get_logger(),
                    "⏳ Next snapshot in %d seconds...", countdown_sec);
            }
            countdown_sec--;
        }
    }

    // Fires every foto_period seconds
    void timer_cb() {
        // Reset countdown for next cycle
        countdown_sec = static_cast<int>(foto_period);

        if (!latest_img_) {
            RCLCPP_WARN(this->get_logger(),
                "⚠  Timer fired but no image received yet — skipping. "
                "Next attempt in %.0f s.", foto_period);
            return;
        }

        snap_count++;
        RCLCPP_INFO(this->get_logger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        RCLCPP_INFO(this->get_logger(), "📷 Publishing snapshot #%d", snap_count);
        RCLCPP_INFO(this->get_logger(), "   stamp: %u.%u",
            latest_img_->header.stamp.sec,
            latest_img_->header.stamp.nanosec);
        RCLCPP_INFO(this->get_logger(), "   size:  %ux%u  encoding: %s",
            latest_img_->width,
            latest_img_->height,
            latest_img_->encoding.c_str());
        RCLCPP_INFO(this->get_logger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");

        img_pub_->publish(*latest_img_);
    }

public:
    MainNode() : rclcpp::Node("ManualNode") {
        this->declare_parameter("foto_period",        15.0);
        this->declare_parameter("image_input_topic",  "/camera/camera/color/image_raw");
        this->declare_parameter("image_output_topic", "/snapshot/image_raw");

        foto_period       = this->get_parameter("foto_period").as_double();
        auto input_topic  = this->get_parameter("image_input_topic").as_string();
        auto output_topic = this->get_parameter("image_output_topic").as_string();

        img_sub_realsense_ = this->create_subscription<sensor_msgs::msg::Image>(
            input_topic,
            rclcpp::SensorDataQoS(),
            std::bind(&MainNode::realsense_cb, this, std::placeholders::_1)
        );

        img_pub_ = this->create_publisher<sensor_msgs::msg::Image>(output_topic, 10);

        // Main snapshot timer
        base_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(foto_period),
            std::bind(&MainNode::timer_cb, this)
        );

        // 1-second countdown timer
        countdown_sec = static_cast<int>(foto_period);
        countdown_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MainNode::countdown_cb, this)
        );

        RCLCPP_INFO(this->get_logger(), "╔══════════════════════════════════════╗");
        RCLCPP_INFO(this->get_logger(), "║         ManualNode started           ║");
        RCLCPP_INFO(this->get_logger(), "╠══════════════════════════════════════╣");
        RCLCPP_INFO(this->get_logger(), "║  foto_period  : %.1f s", foto_period);
        RCLCPP_INFO(this->get_logger(), "║  input topic  : %s", input_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "║  output topic : %s", output_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "╚══════════════════════════════════════╝");
        RCLCPP_INFO(this->get_logger(), "⏳ First snapshot in %.0f seconds...", foto_period);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MainNode>());
    rclcpp::shutdown();
    return 0;
}