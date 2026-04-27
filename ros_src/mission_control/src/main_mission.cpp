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
#include <opencv2/highgui.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>
#include <opencv2/objdetect/aruco_board.hpp>
#include <opencv2/calib3d.hpp>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
  #include <cv_bridge/cv_bridge.hpp>
#elif __has_include(<cv_bridge/cv_bridge.h>)
  #include <cv_bridge/cv_bridge.h>
#else
  #error "cv_bridge header not found"
#endif

using namespace nav_msgs::msg;

// ─────────────────────────────────────────────────────────────────────────── //
//  PID
// ─────────────────────────────────────────────────────────────────────────── //
class PID {
  private:
    float kp_, ki_, kd_;
    double P = 0, I = 0, D = 0;
    double prev_error = std::numeric_limits<float>::signaling_NaN();
    double prev_time  = std::numeric_limits<float>::signaling_NaN();
    double dt         = std::numeric_limits<float>::signaling_NaN();
    rclcpp::Clock::SharedPtr timer_;
    std::string name_;

  public:
    PID(std::string name, rclcpp::Clock::SharedPtr timer,
        float kp, float ki, float kd)
      : kp_(kp), ki_(ki), kd_(kd), timer_(timer), name_(name)
    {
      RCLCPP_INFO_STREAM(rclcpp::get_logger(name.c_str()),
        "Started PID control loop '" << name
        << "' with PID coefficients: [" << kp << ", " << ki << ", " << kd << "]");
    }

    double get_action(double err) {
      if (std::isnan(prev_time) || std::isnan(prev_error)) {
        prev_time  = timer_->now().seconds();
        prev_error = err;
        return 0.0;
      }
      double curr = timer_->now().seconds();
      dt = curr - prev_time;

      if (dt < 1e-6) {
        RCLCPP_ERROR(rclcpp::get_logger(name_),
          "DT is too small — check input frequency (dt > 1e-6)");
        throw std::runtime_error("dt too small");
      }

      P  = err;
      I += err * dt;
      D  = (err - prev_error) / dt;
      I  = std::clamp(I, -1.0, 1.0);

      prev_error = err;
      prev_time  = curr;

      return P * kp_ + I * ki_ + D * kd_;
    }
};

// ─────────────────────────────────────────────────────────────────────────── //
//  State machine
// ─────────────────────────────────────────────────────────────────────────── //
class StateMachine {
  public:
    enum State  { Landed, Landing, Flying, Hover };
    enum Action { Land, Takeoff, Pause, Resume, Touchdown };

    struct pair_hash {
      std::size_t operator()(const std::pair<int,int>& p) const {
        return std::hash<int>()(p.first) ^ (std::hash<int>()(p.second) << 1);
      }
    };

    State state;
    std::queue<int> nodes_left;
    std::unordered_map<int, std::pair<float,float>> tag_positions;
    std::unordered_map<std::pair<State,Action>, State, pair_hash> transitions;

    int get_next_node() {
      if (nodes_left.empty()) return -1;
      int next = nodes_left.front();
      nodes_left.pop();
      return next;
    }

    StateMachine() {
      state = Landed;

      nodes_left = std::queue<int>(std::deque<int>{
        101,102,103,104,105,106,107,108,109,110,101
      });

      // Relative positions to bottom-left corner (X, Y)
      tag_positions[101] = {5.0f, 1.0f};
      tag_positions[102] = {1.5f, 3.0f};
      tag_positions[103] = {5.0f, 3.0f};
      tag_positions[104] = {8.5f, 3.0f};
      tag_positions[105] = {1.5f, 5.0f};
      tag_positions[106] = {5.0f, 5.0f};
      tag_positions[107] = {8.5f, 5.0f};
      tag_positions[108] = {1.5f, 7.0f};
      tag_positions[109] = {5.0f, 7.0f};
      tag_positions[110] = {8.5f, 7.0f};

      transitions[{Landed,  Takeoff  }] = Hover;
      transitions[{Landing, Touchdown}] = Landed;
      transitions[{Hover,   Land     }] = Landing;
      transitions[{Hover,   Resume   }] = Flying;
      transitions[{Flying,  Land     }] = Landing;
      transitions[{Flying,  Pause    }] = Hover;
    }

    bool transition(Action action) {
      auto key = std::make_pair(this->state, action);
      auto it  = transitions.find(key);
      if (it != transitions.end()) {
        this->state = it->second;
        return true;
      }
      return false;
    }
};

// ─────────────────────────────────────────────────────────────────────────── //
//  Main node
// ─────────────────────────────────────────────────────────────────────────── //
class Executor : public rclcpp::Node {
  private:
    StateMachine stm;
    int current_node = -1;

    // ── OFFBOARD startup sequencer ────────────────────────────────────── //
    // PX4 requires setpoints to stream BEFORE accepting the OFFBOARD switch.
    // We publish zero-velocity at 10 Hz for 2 s, then: OFFBOARD → arm → takeoff.
    rclcpp::TimerBase::SharedPtr startup_timer_;
    int startup_count_           = 0;
    static constexpr int STARTUP_TICKS = 20;   // 20 × 100 ms = 2 s

    // ── alignment timer ───────────────────────────────────────────────── //
    // Once centered over a marker, wait 5 s then auto-advance to next node.
    rclcpp::TimerBase::SharedPtr alignment_timer_;
    bool alignment_started_ = false;

    // ── state ─────────────────────────────────────────────────────────── //
    Odometry::ConstSharedPtr                      current_odometry = nullptr;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr  camera_info      = nullptr;

    std::shared_ptr<PID> x_pid, y_pid;

    double kp_x, ki_x, kd_x;
    double kp_y, ki_y, kd_y;
    double down_speed, flight_height;
    float  marker_size;

    // ── subscribers / publishers / clients ────────────────────────────── //
    rclcpp::Subscription<Odometry>::SharedPtr                     odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr      image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr         detection_over_sub_;

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr             hover_ready_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;

    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr  takeoff_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr  land_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr     mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;

    // ── vision ────────────────────────────────────────────────────────── //
    cv::Mat image_, gray_;
    std::shared_ptr<cv_bridge::CvImage> cv_bridge_image_;

    cv::aruco::Dictionary         dictionary_;
    cv::aruco::DetectorParameters detector_params_;
    cv::aruco::ArucoDetector      detector_;

    // ────────────────────────────────────────────────────────────────────── //
    //  Startup: stream zero setpoints → switch OFFBOARD → arm → takeoff
    // ────────────────────────────────────────────────────────────────────── //
    void startup_cb() {
      geometry_msgs::msg::TwistStamped zero;
      zero.header.stamp = this->now();
      vel_pub_->publish(zero);

      startup_count_++;
      if (startup_count_ < STARTUP_TICKS) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
          "Streaming setpoints... (%d / %d)", startup_count_, STARTUP_TICKS);
        return;
      }

      startup_timer_->cancel();

      RCLCPP_INFO(this->get_logger(),
        "Setpoint stream ready. Requesting OFFBOARD mode...");
      set_offboard_mode();

      RCLCPP_INFO(this->get_logger(), "Arming...");
      auto arm_req   = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
      arm_req->value = true;
      arm_client_->async_send_request(arm_req);

      current_node = stm.get_next_node();   // first node = 101
      if (current_node != -1) {
        RCLCPP_INFO(this->get_logger(),
          "Starting mission. First node: %d", current_node);
        if (stm.transition(StateMachine::Takeoff)) {
          this->takeoff(static_cast<float>(flight_height));
        }
      }
    }

    // ────────────────────────────────────────────────────────────────────── //
    //  MAVROS helpers
    // ────────────────────────────────────────────────────────────────────── //
    void wait_for_services() {
      while (!takeoff_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(this->get_logger(), "Waiting for MAVROS services...");
      }
    }

    void set_offboard_mode() {
      wait_for_services();
      auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
      req->custom_mode = "OFFBOARD";
      mode_client_->async_send_request(req);
      RCLCPP_INFO(this->get_logger(), "Requested OFFBOARD mode");
    }

    void land() {
      wait_for_services();

      auto req       = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
      req->altitude  = 0.0; req->latitude = 0.0; req->longitude = 0.0;
      req->min_pitch = 0.0; req->yaw = 0.0;
      land_client_->async_send_request(req);

      auto mode_req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
      mode_req->custom_mode = "AUTO.LAND";
      mode_client_->async_send_request(mode_req);

      RCLCPP_INFO(this->get_logger(), "Landing requested");
    }

    void takeoff(float altitude = 2.3f) {
      wait_for_services();

      auto mode_req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
      mode_req->custom_mode = "AUTO.TAKEOFF";
      mode_client_->async_send_request(mode_req);

      auto req       = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
      req->altitude  = altitude; req->latitude = 0.0; req->longitude = 0.0;
      req->min_pitch = 0.0; req->yaw = 0.0;
      takeoff_client_->async_send_request(req);

      RCLCPP_INFO(this->get_logger(), "Takeoff requested to %.1f m", altitude);
    }

    // ────────────────────────────────────────────────────────────────────── //
    //  Sensor callbacks
    // ────────────────────────────────────────────────────────────────────── //
    void camera_info_cb_(sensor_msgs::msg::CameraInfo::ConstSharedPtr ci_msg) {
      camera_info = ci_msg;
    }

    void odom_cb(Odometry::ConstSharedPtr odom_msg) {
      current_odometry = odom_msg;
    }

    bool get_camera_params(cv::Mat & camera_matrix, cv::Mat & dist_coeffs) {
      if (!camera_info) {
        RCLCPP_WARN(this->get_logger(), "No camera info yet");
        return false;
      }
      camera_matrix = cv::Mat(3, 3, CV_64F,
        const_cast<double*>(camera_info->k.data())).clone();
      dist_coeffs = cv::Mat(
        static_cast<int>(camera_info->d.size()), 1, CV_64F,
        const_cast<double*>(camera_info->d.data())).clone();
      return true;
    }

    void image_cb(sensor_msgs::msg::Image::ConstSharedPtr img_msg) {
      if (stm.state != StateMachine::Flying &&
          stm.state != StateMachine::Landing) return;

      if (!current_odometry) {
        RCLCPP_INFO(get_logger(), "No odometry...");
        return;
      }

      cv_bridge_image_ = cv_bridge::toCvCopy(img_msg, "bgr8");
      image_           = cv_bridge_image_->image;
      cv::cvtColor(image_, gray_, cv::COLOR_BGR2GRAY);

      // ── detect markers ──────────────────────────────────────────────── //
      std::vector<int>                       ids;
      std::vector<std::vector<cv::Point2f>>  corners, rejected;
      detector_.detectMarkers(gray_, corners, ids, rejected);

      if (ids.empty()) return;

      // ── pose via solvePnP (estimatePoseSingleMarkers removed in 4.8) ── //
      cv::Mat camera_matrix, dist_coeffs;
      if (!get_camera_params(camera_matrix, dist_coeffs)) return;

      const float h = marker_size / 2.f;
      std::vector<cv::Point3f> obj_pts = {
        {-h,  h, 0.f},
        { h,  h, 0.f},
        { h, -h, 0.f},
        {-h, -h, 0.f}
      };

      std::vector<cv::Vec3d> rvecs(ids.size()), tvecs(ids.size());
      for (size_t k = 0; k < ids.size(); ++k) {
        cv::solvePnP(obj_pts, corners[k],
                     camera_matrix, dist_coeffs,
                     rvecs[k], tvecs[k]);
      }

      // ── find target marker ──────────────────────────────────────────── //
      size_t i = 0;
      bool found = false;
      for (; i < ids.size(); ++i) {
        if (ids[i] == current_node) { found = true; break; }
      }
      if (!found) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "Tag with id %d was not found!", current_node);
        return;
      }
      if (i >= tvecs.size()) {
        RCLCPP_WARN(this->get_logger(), "Index mismatch ids/tvecs");
        return;
      }

      // ── velocity control ─────────────────────────────────────────────── //
      double x_cam = tvecs[i][0];
      double y_cam = tvecs[i][1];

      // Camera → FC frame (90° rotation)
      double x_fc =  y_cam;
      double y_fc = -x_cam;

      auto x_control = x_pid->get_action(x_fc);
      auto y_control = y_pid->get_action(y_fc);

      geometry_msgs::msg::TwistStamped vec_vel;
      vec_vel.header.stamp   = this->now();
      vec_vel.twist.linear.x = x_control;
      vec_vel.twist.linear.y = y_control;
      vec_vel.twist.linear.z = 0.0;

      if (std::abs(x_control) < 0.1 && std::abs(y_control) < 0.1) {
        if (stm.state == StateMachine::Landing) {
          if (tvecs[i][2] <= 0.7) {
            stm.transition(StateMachine::Touchdown);
            this->land();
            return;
          } else {
            vec_vel.twist.linear.z = down_speed;
          }
        } else {
          // Aligned over node — start 5 s hold timer if not already running
          if (!alignment_started_) {
            alignment_started_ = true;
            stm.transition(StateMachine::Pause);
            RCLCPP_INFO(this->get_logger(),
              "Aligned with node %d — advancing to next in 5 s", current_node);

            alignment_timer_ = this->create_wall_timer(
              std::chrono::seconds(5),
              [this]() {
                alignment_timer_->cancel();
                alignment_started_ = false;

                current_node = stm.get_next_node();
                if (current_node == -1) {
                  RCLCPP_INFO(this->get_logger(), "Mission complete. Landing...");
                  stm.transition(StateMachine::Land);
                  this->land();
                  return;
                }

                RCLCPP_INFO(this->get_logger(),
                  "Moving to next node: %d", current_node);
                stm.transition(StateMachine::Resume);
              });
          }
          return;
        }
      }

      vel_pub_->publish(vec_vel);
    }

    // ────────────────────────────────────────────────────────────────────── //
    //  Mission callbacks
    // ────────────────────────────────────────────────────────────────────── //
    void detection_end_cb(std_msgs::msg::Empty::ConstSharedPtr) {
      RCLCPP_INFO(this->get_logger(),
        "Detection finished for node %d", current_node);

      current_node = stm.get_next_node();

      if (current_node == -1) {
        RCLCPP_INFO(this->get_logger(), "Mission complete. Landing...");
        stm.transition(StateMachine::Land);
        this->land();
        return;
      }

      RCLCPP_INFO(this->get_logger(), "Next node: %d", current_node);
<<<<<<< HEAD

      if(!this->stm.transition(StateMachine::Resume)) {
        RCLCPP_WARN(this->get_logger(), "Invalid transition to Resume");
=======
      if (!stm.transition(StateMachine::Resume)) {
        RCLCPP_WARN(this->get_logger(), "Invalid transition to Resume");
      }
    }

    void print_all_params() {
      auto params = this->list_parameters({}, 10);
      for (const auto & name : params.names) {
        rclcpp::Parameter param;
        if (this->get_parameter(name, param)) {
          RCLCPP_INFO(this->get_logger(), "%s: %s",
            name.c_str(), param.value_to_string().c_str());
        }
>>>>>>> 94d46c8 (Adapted code to older opencv api)
      }
    }

  public:
    Executor()
      : rclcpp::Node("MainMission"),
        dictionary_     (cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250)),
        detector_params_(cv::aruco::DetectorParameters()),
        detector_       (dictionary_, detector_params_)
    {
      // ── parameters ────────────────────────────────────────────────── //
      this->declare_parameter("odometry_topic",    "/mavros/local_position/odom");
      this->declare_parameter("image_topic",       "/camera/camera/color/image_raw");
      this->declare_parameter("camera_info_topic", "/camera/camera/color/camera_info");
      this->declare_parameter("twist_topic",       "/mavros/setpoint_attitude/cmd_vel");
      this->declare_parameter("marker_size",       0.15);
      this->declare_parameter("kp_x",  0.0);
      this->declare_parameter("ki_x",  0.0);
      this->declare_parameter("kd_x",  0.0);
      this->declare_parameter("kp_y",  0.0);
      this->declare_parameter("ki_y",  0.0);
      this->declare_parameter("kd_y",  0.0);
      this->declare_parameter("down_speed",    -0.1);
      this->declare_parameter("flight_height",  2.3);

      kp_x = this->get_parameter("kp_x").as_double();
      ki_x = this->get_parameter("ki_x").as_double();
      kd_x = this->get_parameter("kd_x").as_double();
      kp_y = this->get_parameter("kp_y").as_double();
      ki_y = this->get_parameter("ki_y").as_double();
      kd_y = this->get_parameter("kd_y").as_double();

      down_speed    = this->get_parameter("down_speed").as_double();
      flight_height = this->get_parameter("flight_height").as_double();
      marker_size   = static_cast<float>(
        this->get_parameter("marker_size").as_double());

      x_pid = std::make_shared<PID>("X_PID", this->get_clock(), kp_x, ki_x, kd_x);
      y_pid = std::make_shared<PID>("Y_PID", this->get_clock(), kp_y, ki_y, kd_y);

      std::string odometry_topic    = this->get_parameter("odometry_topic").as_string();
      std::string image_topic       = this->get_parameter("image_topic").as_string();
      std::string twist_topic       = this->get_parameter("twist_topic").as_string();
      std::string camera_info_topic = this->get_parameter("camera_info_topic").as_string();

      auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();

      // ── subscriptions ─────────────────────────────────────────────── //
      odom_sub_ = this->create_subscription<Odometry>(
        odometry_topic, qos,
        std::bind(&Executor::odom_cb, this, std::placeholders::_1));

      image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic, qos,
        std::bind(&Executor::image_cb, this, std::placeholders::_1));

      camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic, qos,
        std::bind(&Executor::camera_info_cb_, this, std::placeholders::_1));

      detection_over_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "/main_mission/detection_end", 10,
        std::bind(&Executor::detection_end_cb, this, std::placeholders::_1));

      // ── publishers ────────────────────────────────────────────────── //
      // MAVROS cmd_vel requires RELIABLE — use a separate QoS for vel_pub_
      vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        twist_topic, rclcpp::QoS(10).reliable());

      hover_ready_pub_ = this->create_publisher<std_msgs::msg::Empty>(
        "/main_mission/detection_start", 10);

      // ── service clients ───────────────────────────────────────────── //
      takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>(
        "/mavros/cmd/takeoff");
      land_client_    = this->create_client<mavros_msgs::srv::CommandTOL>(
        "/mavros/cmd/land");
      mode_client_    = this->create_client<mavros_msgs::srv::SetMode>(
        "/mavros/set_mode");
      arm_client_     = this->create_client<mavros_msgs::srv::CommandBool>(
        "/mavros/cmd/arming");

      print_all_params();

      // ── kick off startup sequence ─────────────────────────────────── //
      RCLCPP_INFO(this->get_logger(),
        "Streaming zero setpoints for 2 s before requesting OFFBOARD...");

      startup_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&Executor::startup_cb, this));
    }
};

// ─────────────────────────────────────────────────────────────────────────── //
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Executor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}