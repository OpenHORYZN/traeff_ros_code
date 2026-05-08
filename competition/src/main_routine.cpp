#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cstdio>
#include <functional>
#include <rclcpp/create_subscription.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>



class Scheduler : public rclcpp::Node{
  private:
    enum action{
      TAKEOFF,
      LAND,
      GO,
      STOP
    };

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;

    void odom_cb(nav_msgs::msg::Odometry::ConstSharedPtr odom_msg) {

    }
    
  public:

    Scheduler() : rclcpp::Node("MissionScheduler") {
      this->declare_parameter("odom_topic", "/mavros/local_position/odom");
      this->declare_parameter("velocity_topic", "/mavros/setpoint_attitude/cmd_vel");

      std::string odom_topic = this->get_parameter("odom_topic").as_string();
      std::string velocity_topic = this->get_parameter("velocity_topic").as_string();

      this->odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, rclcpp::SensorDataQoS(), std::bind(this, &Scheduler::odom_cb, std::placeholders::_1));
    
      this->twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        velocity_topic, rclcpp::SensorDataQoS());
      }

};

int main(int argc, char ** argv)
{
  
}
