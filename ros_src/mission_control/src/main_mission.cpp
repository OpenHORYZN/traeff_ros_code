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
#include <unordered_map>
#include <utility>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include "mavros_msgs/srv/command_tol.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "std_msgs/msg/empty.hpp"
#include <std_msgs/msg/empty.hpp>


#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>    
#include <opencv2/highgui.hpp>


#if __has_include(<cv_bridge/cv_bridge.hpp>)
  #include <cv_bridge/cv_bridge.hpp>
#elif __has_include(<cv_bridge/cv_bridge.h>)
  #include <cv_bridge/cv_bridge.h>
#else
  #error "cv_bridge header not found"
#endif


using namespace nav_msgs::msg;

class PID {
  private:
    float kp_,ki_,kd_;
    double P = 0, I = 0, D = 0;
    double prev_error = std::numeric_limits<float>::signaling_NaN(), prev_time = std::numeric_limits<float>::signaling_NaN(), dt = std::numeric_limits<float>::signaling_NaN();
    rclcpp::Clock::SharedPtr timer_;
    std::string name_;

  public:
    PID(std::string name, rclcpp::Clock::SharedPtr timer, float kp, float ki, float kd) : kp_(kp), ki_(ki), kd_(kd), timer_(timer), name_(name){
      RCLCPP_INFO_STREAM(rclcpp::get_logger(name.c_str()), "Started PID control loop '" << name << "' with PID coefficients: [" << kp << ", " << ki << ", " << kd << "]");
    }

    double get_action(double err) {
      if(std::isnan(prev_time) || std::isnan(prev_error)) {
        prev_time = timer_->now().seconds();
        prev_error = err;
        return 0.0;
      }
      double curr = timer_->now().seconds();
      dt = curr - prev_time;

      if(dt < 1e-6){
        RCLCPP_ERROR(rclcpp::get_logger(name_), "DT is too small, please check the frequency of your input data (dt > 1e-6)");
        throw std::runtime_error("dt too small");
      }

      P = err;
      I += err * dt;
      D = (err - prev_error) / dt;

      I = std::clamp(I,-1.0,1.0);

      prev_error = err;
      prev_time = curr;

      return P * kp_ + I * ki_ + D * kd_;
    }
};

class StateMachine {
  public:
    enum State{
      Landed,
      Landing,
      Flying,
      Hover
    };

    enum Action{
      Land,
      Takeoff,
      Pause,
      Resume,
      Touchdown
    };

    struct pair_hash {
        std::size_t operator()(const std::pair<int,int>& p) const {
            return std::hash<int>()(p.first) ^ (std::hash<int>()(p.second) << 1);
        }
    };

    State state;
    std::queue<int> nodes_left;
    std::unordered_map<int, std::pair<float, float>> tag_positions;
    std::unordered_map<std::pair<State,Action>, State, pair_hash> transitions;

    int get_next_node() {
        if (nodes_left.empty()) {
            return -1;
        }

        int next_node = nodes_left.front();
        nodes_left.pop();
        return next_node;
    }

    StateMachine() {
      state = Landed;

      // TODO replace with input from file
      nodes_left = std::queue<int>(std::deque<int>{
          101,102,103,104,105,106,107,108,109,110
      });

      // Relative positions to bottom left. (X,Y)
      tag_positions[101] = {5.0f,1.0f};
      tag_positions[102] = {1.5f,3.0f};
      tag_positions[103] = {5.0f,3.0f};
      tag_positions[104] = {8.5f, 3.0f};
      tag_positions[105] = {1.5f, 5.0f};
      tag_positions[106] = {5.0f,5.0f};
      tag_positions[107] = {8.5f,5.0f};
      tag_positions[108] = {1.5f,7.0f};
      tag_positions[109] = {5.0f,7.0f};
      tag_positions[110] = {8.5f,7.0f};

      // Add valid transitions A->B

      // Add transition graph
      transitions[{Landed,Takeoff}] = Hover;
      transitions[{Landing,Touchdown}] = Landed;
      transitions[{Hover,Land}] = Landing;
      transitions[{Hover,Resume}] = Flying;
      transitions[{Flying,Land}] = Landing;
      transitions[{Flying,Pause}] = Hover;

    }

    

    bool transition(Action action) {
      std::pair<State, Action> key = {this->state,action};

      if(transitions.find(key) != transitions.end()) {
        this->state = transitions[key];
        return true;
      }
      return false;
    }

};

class Executor : public rclcpp::Node{
  private:
    StateMachine stm;
    int current_node = -1;
    Odometry::ConstSharedPtr current_odometry = nullptr;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info = nullptr;

    std::shared_ptr<PID> x_pid, y_pid;

    double kp_x, ki_x, kd_x;
    double kp_y, ki_y, kd_y;

    double down_speed, flight_height;

    float marker_size;

    rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr detection_over_sub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr hover_ready_pub_;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;

    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;

    cv::Mat image_,gray_;
    std::shared_ptr<cv_bridge::CvImage> cv_bridge_image_;

    // Aruco Detector
    cv::Ptr<cv::aruco::Dictionary> dictionary_;


    void wait_for_services() {
        while (!takeoff_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "Waiting for MAVROS services...");
        }
    }

    void set_offboard_mode() {
        wait_for_services();

        auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        req->custom_mode = "OFFBOARD";

        auto future = mode_client_->async_send_request(req);

        RCLCPP_INFO(this->get_logger(), "Requested OFFBOARD mode");
    }

    void land() {
        wait_for_services();

        // Option 1: direct land command
        auto req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
        req->altitude = 0.0;
        req->latitude = 0.0;
        req->longitude = 0.0;
        req->min_pitch = 0.0;
        req->yaw = 0.0;

        land_client_->async_send_request(req);

        // Option 2 (more robust): switch mode
        auto mode_req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        mode_req->custom_mode = "AUTO.LAND";
        mode_client_->async_send_request(mode_req);

        RCLCPP_INFO(this->get_logger(), "Landing requested");
    }

    void takeoff(float altitude = 2.3) {
        wait_for_services();

        // Arm first
        auto arm_req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        arm_req->value = true;
        arm_client_->async_send_request(arm_req);

        // Set mode to GUIDED or OFFBOARD/AUTO depending on your setup
        auto mode_req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        mode_req->custom_mode = "AUTO.TAKEOFF";
        mode_client_->async_send_request(mode_req);

        // Takeoff command
        auto req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
        req->altitude = altitude;
        req->latitude = 0.0;
        req->longitude = 0.0;
        req->min_pitch = 0.0;
        req->yaw = 0.0;

        takeoff_client_->async_send_request(req);

        RCLCPP_INFO(this->get_logger(), "Takeoff requested");
    }

    void camera_info_cb_(sensor_msgs::msg::CameraInfo::ConstSharedPtr ci_msg) {
      camera_info = ci_msg;
    }

    void odom_cb(Odometry::ConstSharedPtr odom_msg) {
      current_odometry = odom_msg;
    }

    bool get_camera_params(cv::Mat &camera_matrix, cv::Mat &dist_coeffs) {
        if (!camera_info) {
            RCLCPP_WARN(this->get_logger(), "No camera info yet");
            return false;
        }

        // K = 3x3 intrinsic matrix
        camera_matrix = cv::Mat(3, 3, CV_64F, (void*)camera_info->k.data()).clone();

        // D = distortion coefficients
        dist_coeffs = cv::Mat(camera_info->d.size(), 1, CV_64F, (void*)camera_info->d.data()).clone();

        return true;
    }

    void image_cb(sensor_msgs::msg::Image::ConstSharedPtr img_msg) {
      if(stm.state != StateMachine::Flying && stm.state != StateMachine::Landing){
        return;
      }
      if(current_odometry == nullptr) {
        RCLCPP_INFO(get_logger(), "No odometry...");
        return;
      }
      cv_bridge_image_ = cv_bridge::toCvCopy(img_msg, std::string("bgr8"));
      image_ = cv_bridge_image_->image;
      
      cv::cvtColor(image_, gray_, cv::COLOR_BGR2GRAY);

      std::vector<int> ids;
      std::vector<std::vector<cv::Point2f>> corners;

      cv::aruco::detectMarkers(gray_, dictionary_, corners, ids);

      std::vector<cv::Vec3d> rvecs, tvecs;

      cv::Mat camera_matrix, dist_coeffs;

      if (!get_camera_params(camera_matrix, dist_coeffs)) {
          return;
      }

      cv::aruco::estimatePoseSingleMarkers(
          corners,
          marker_size,
          camera_matrix,
          dist_coeffs,
          rvecs,
          tvecs
      );

      if (!tvecs.empty()) {
        geometry_msgs::msg::TwistStamped vec_vel;
        vec_vel.twist.set__linear(geometry_msgs::msg::Vector3().set__x(0.0).set__y(0.0).set__z(0.0));
        vec_vel.twist.set__angular(geometry_msgs::msg::Vector3().set__x(0.0).set__y(0.0).set__z(0.0));

        // Control Loop
        bool found = false;
        size_t i = 0;
        for(; i < ids.size(); i++) {
          if(ids.at(i) == current_node){found = true;break;}
        }

        if(!found){
          RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),1000,"Tag with id %d was not found!",current_node);
          return;
        }

        if (i >= tvecs.size()) {
            RCLCPP_WARN(this->get_logger(), "Index mismatch ids/tvecs");
            return;
        }

        auto x_control = x_pid->get_action(tvecs[i][0]);
        auto y_control = y_pid->get_action(tvecs[i][1]);
        
        vec_vel.twist.linear.x = x_control;
        vec_vel.twist.linear.y = y_control;

        if(std::abs(x_control) < 0.1 && std::abs(y_control) < 0.1) {
          if(stm.state == StateMachine::Landing) {
            if(tvecs[i][2] <= 0.7) {
              stm.transition(StateMachine::Touchdown);
              this->land();
              return;
            }
            else{
              vec_vel.twist.linear.z = down_speed;
            }
          }
          else{
            stm.transition(StateMachine::Pause);
            return;
          }
        }
        
        vel_pub_->publish(vec_vel);
      }


    }

    void print_all_params() {
        auto params = this->list_parameters({}, 10);

        for (const auto & name : params.names) {
            rclcpp::Parameter param;
            if (this->get_parameter(name, param)) {
                RCLCPP_INFO(this->get_logger(),
                            "%s: %s",
                            name.c_str(),
                            param.value_to_string().c_str());
            }
        }
    }

    void detection_end_cb(std_msgs::msg::Empty::ConstSharedPtr) {
      current_node = stm.get_next_node();

      if(current_node == -1) {
        this->stm.transition(StateMachine::Land);
        return;
      }
      this->stm.transition(StateMachine::Resume);
    }

  public:
    Executor() : rclcpp::Node("MainMission") {

      // Setup
      this->declare_parameter("odometry_topic", "/mavros/local_position/odom");
      this->declare_parameter("image_topic", "/image");
      this->declare_parameter("camera_info_topic", "/image/info");
      this->declare_parameter("twist_topic", "/mavros/setpoint_attitude/cmd_vel");
      this->declare_parameter("marker_size", 0.15);

      this->declare_parameter("kp_x", 0.0);
      this->declare_parameter("ki_x", 0.0);
      this->declare_parameter("kd_x", 0.0);

      this->declare_parameter("kp_y", 0.0);
      this->declare_parameter("ki_y", 0.0);
      this->declare_parameter("kd_y", 0.0);

      this->declare_parameter("down_speed", -0.1);

      this->declare_parameter("flight_height", 2.3);

      kp_x = this->get_parameter("kp_x").as_double();
      ki_x = this->get_parameter("ki_x").as_double();
      kd_x = this->get_parameter("kd_x").as_double();

      kp_y = this->get_parameter("kp_y").as_double();
      ki_y = this->get_parameter("ki_y").as_double();
      kd_y = this->get_parameter("kd_y").as_double();

      down_speed = this->get_parameter("down_speed").as_double();
      
      flight_height = this->get_parameter("flight_height").as_double();

      x_pid = std::make_shared<PID>("X_PID",this->get_clock(),kp_x,ki_x,kd_x);
      y_pid = std::make_shared<PID>("Y_PID",this->get_clock(),kp_y,ki_y,kd_y);

      std::string odometry_topic = this->get_parameter("odometry_topic").as_string();
      std::string image_topic = this->get_parameter("image_topic").as_string();
      std::string twist_topic = this->get_parameter("twist_topic").as_string();
      std::string camera_info_topic = this->get_parameter("camera_info_topic").as_string();

      marker_size = (float)this->get_parameter("marker_size").as_double();

      auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();

      odom_sub_ = this->create_subscription<Odometry>(
          odometry_topic,
          qos,
          std::bind(&Executor::odom_cb, this, std::placeholders::_1)
      );

      image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
          image_topic,
          qos,
          std::bind(&Executor::image_cb, this, std::placeholders::_1)
      );

      camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
          camera_info_topic,
          qos,
          std::bind(&Executor::camera_info_cb_, this, std::placeholders::_1)
      );

      vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
          twist_topic,
          qos
      );
      
      hover_ready_pub_ = this->create_publisher<std_msgs::msg::Empty>("/main_mission/detection_start", 10);

      detection_over_sub_ = this->create_subscription<std_msgs::msg::Empty>("/main_mission/detection_end",10,std::bind(&Executor::detection_end_cb,this,std::placeholders::_1));

      dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);

      print_all_params();
    }

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Executor>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
