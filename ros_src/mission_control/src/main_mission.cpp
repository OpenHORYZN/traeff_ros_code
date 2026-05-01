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
    enum State  { Landed, Landing, TakingOff, Flying, Hover };
    enum Action { Land, Takeoff, Pause, Resume, Touchdown, ReachAltitude };

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
        101,103,102,105,106,103,104,103,101
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

      transitions[{Landed,    Takeoff      }] = TakingOff;
      transitions[{TakingOff, ReachAltitude}] = Hover;
      transitions[{Landing,   Touchdown    }] = Landed;
      transitions[{Hover,     Land         }] = Landing;
      transitions[{Hover,     Resume       }] = Flying;
      transitions[{Flying,    Land         }] = Landing;
      transitions[{Flying,    Pause        }] = Hover;
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

    // ── startup state ─────────────────────────────────────────────────── //
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
    int  startup_count_      = 0;
    bool offboard_confirmed_ = false;
    bool armed_confirmed_    = false;
    bool arm_requested_      = false;
    bool takeoff_complete_   = false;
    static constexpr int STARTUP_TICKS = 20;

    // ── alignment timer ───────────────────────────────────────────────── //
    rclcpp::TimerBase::SharedPtr alignment_timer_;
    bool alignment_started_ = false;

    // ── sensor state ──────────────────────────────────────────────────── //
    Odometry::ConstSharedPtr                      current_odometry = nullptr;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr  camera_info      = nullptr;
    mavros_msgs::msg::State::ConstSharedPtr       vehicle_state    = nullptr;

    std::shared_ptr<PID> x_pid, y_pid;

    double kp_x, ki_x, kd_x;
    double kp_y, ki_y, kd_y;
    double up_speed, down_speed, flight_height, camera_yaw_deg_, aruco_tolerance;
    float  marker_size;

    // ── subscribers / publishers / clients ────────────────────────────── //
    rclcpp::Subscription<Odometry>::SharedPtr                     odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr      image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr         detection_over_sub_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr      state_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr       heading_sub_;

    double current_heading_deg_ = 0.0;
    bool   heading_received_    = false;

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr             hover_ready_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;

    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr     mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;

    // ── vision ────────────────────────────────────────────────────────── //
    cv::Mat image_, gray_;
    std::shared_ptr<cv_bridge::CvImage> cv_bridge_image_;

    cv::Ptr<cv::aruco::Dictionary>         dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;

    // ── last-known tag pose cache ──────────────────────────────────────── //
    // Keyed by tag ID so the cache survives node transitions gracefully.
    std::unordered_map<int, cv::Vec3d> last_known_tvec_;

    // ────────────────────────────────────────────────────────────────────── //
    //  Publish a zero-velocity setpoint (heartbeat)
    // ────────────────────────────────────────────────────────────────────── //
    void publish_zero() {
      geometry_msgs::msg::TwistStamped zero;
      zero.header.stamp = this->now();
      vel_pub_->publish(zero);
    }

    // ────────────────────────────────────────────────────────────────────── //
    //  10 Hz heartbeat — handles all startup phases then keeps PX4 in OFFBOARD
    // ────────────────────────────────────────────────────────────────────── //
    void heartbeat_cb() {
      // ── Phase 0: pre-stream before requesting OFFBOARD ────────────────── //
      if (startup_count_ < STARTUP_TICKS) {
        publish_zero();
        startup_count_++;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
          "[Phase 0] Pre-streaming setpoints (%d / %d)",
          startup_count_, STARTUP_TICKS);
        return;
      }

      // ── Phase 1: request OFFBOARD + arm, wait for confirmation ────────── //
      if (!offboard_confirmed_ || !armed_confirmed_) {
        publish_zero();

        if (!offboard_confirmed_) {
          auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
          req->custom_mode = "OFFBOARD";
          mode_client_->async_send_request(req);
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "[Phase 1] Waiting for OFFBOARD confirmation...");
        }

        if (offboard_confirmed_ && !armed_confirmed_ && !arm_requested_) {
          arm_requested_ = true;
          auto arm_req   = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
          arm_req->value = true;
          arm_client_->async_send_request(arm_req);
          RCLCPP_INFO(this->get_logger(),
            "[Phase 1] OFFBOARD confirmed — arm request sent, waiting...");
        } else if (offboard_confirmed_ && !armed_confirmed_) {
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "[Phase 1] Waiting for arm confirmation...");
        }
        return;
      }

      // ── Phase 2: climb to flight_height via velocity setpoint ─────────── //
      if (!takeoff_complete_) {
        if (!current_odometry) {
          publish_zero();
          return;
        }

        double current_z = current_odometry->pose.pose.position.z;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
          "[Phase 2] Climbing — z=%.2f / %.2f m", current_z, flight_height);

        if (current_z < flight_height - 0.15) {
          geometry_msgs::msg::TwistStamped cmd;
          cmd.header.stamp   = this->now();
          cmd.twist.linear.z = up_speed;
          vel_pub_->publish(cmd);
        } else {
          publish_zero();
          takeoff_complete_ = true;
          stm.transition(StateMachine::Takeoff);
          stm.transition(StateMachine::ReachAltitude);  // Landed→TakingOff→Hover

          current_node = stm.get_next_node();
          if (current_node != -1) {
            RCLCPP_INFO(this->get_logger(),
              "[Phase 2] Altitude reached. Starting mission, first node: %d",
              current_node);
            stm.transition(StateMachine::Resume);        // Hover→Flying
          } else {
            RCLCPP_WARN(this->get_logger(), "Node queue empty after takeoff!");
          }
        }
        return;
      }

      // ── Phase 3: mission running — keepalive only during Hover/Landing ── //
      if (stm.state == StateMachine::Hover || stm.state == StateMachine::Landing) {
        publish_zero();
      }
    }

    // ────────────────────────────────────────────────────────────────────── //
    //  MAVROS helpers
    // ────────────────────────────────────────────────────────────────────── //
    void wait_for_services() {
      while (!mode_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(this->get_logger(), "Waiting for MAVROS services...");
      }
    }

    void land() {
      wait_for_services();
      auto mode_req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
      mode_req->custom_mode = "AUTO.LAND";
      mode_client_->async_send_request(mode_req);
      RCLCPP_INFO(this->get_logger(), "Landing requested (AUTO.LAND)");
    }

    // ────────────────────────────────────────────────────────────────────── //
    //  Sensor callbacks
    // ────────────────────────────────────────────────────────────────────── //
    void state_cb(mavros_msgs::msg::State::ConstSharedPtr msg) {
      vehicle_state       = msg;
      offboard_confirmed_ = (msg->mode == "OFFBOARD");
      armed_confirmed_    = msg->armed;
    }

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
      geometry_msgs::msg::TwistStamped vec_vel;
      vec_vel.header.stamp   = this->now();
      vec_vel.twist.linear.x = 0.0;
      vec_vel.twist.linear.y = 0.0;
      vec_vel.twist.linear.z = 0.0;

      // ── state guard ───────────────────────────────────────────────────── //
      if (stm.state != StateMachine::Flying &&
          stm.state != StateMachine::Landing)
      {
        vel_pub_->publish(vec_vel);
        return;
      }

      if (!heading_received_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "No compass heading yet");
        vel_pub_->publish(vec_vel);
        return;
      }

      if (!current_odometry) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "No odometry");
        vel_pub_->publish(vec_vel);
        return;
      }

      if (current_node == -1) {
        vel_pub_->publish(vec_vel);
        return;
      }

      // ── image processing ──────────────────────────────────────────────── //
      cv_bridge_image_ = cv_bridge::toCvCopy(img_msg, "bgr8");
      image_ = cv_bridge_image_->image;
      cv::cvtColor(image_, gray_, cv::COLOR_BGR2GRAY);

      std::vector<int> ids;
      std::vector<std::vector<cv::Point2f>> corners, rejected;
      cv::aruco::detectMarkers(gray_, dictionary_, corners, ids, detector_params_);

      // ── solvePnP for all detected markers ─────────────────────────────── //
      cv::Mat camera_matrix, dist_coeffs;
      if (!get_camera_params(camera_matrix, dist_coeffs)) {
        vel_pub_->publish(vec_vel);
        return;
      }

      const float h = marker_size / 2.f;
      std::vector<cv::Point3f> obj_pts = {
        {-h,  h, 0.f}, { h,  h, 0.f},
        { h, -h, 0.f}, {-h, -h, 0.f}
      };

      std::vector<cv::Vec3d> rvecs(ids.size()), tvecs(ids.size());
      for (size_t k = 0; k < ids.size(); ++k) {
        cv::solvePnP(obj_pts, corners[k],
                     camera_matrix, dist_coeffs,
                     rvecs[k], tvecs[k]);

        // Cache every detected tag's tvec while we have it
        last_known_tvec_[ids[k]] = tvecs[k];
      }

      // ── resolve target tvec (live or cached) ──────────────────────────── //
      cv::Vec3d active_tvec;
      bool using_cached = false;

      // Try to find the target tag in this frame
      size_t i = 0;
      bool found = false;
      for (; i < ids.size(); ++i) {
        if (ids[i] == current_node) { found = true; break; }
      }

      if (found) {
        active_tvec = tvecs[i];
      } else {
        // Fall back to the last known position for this tag
        auto cache_it = last_known_tvec_.find(current_node);
        if (cache_it != last_known_tvec_.end()) {
          active_tvec  = cache_it->second;
          using_cached = true;
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
            "Tag %d lost — using last known position (x=%.3f y=%.3f z=%.3f)",
            current_node,
            active_tvec[0], active_tvec[1], active_tvec[2]);
        } else {
          // No live detection and no cache — nothing we can do
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
            "Tag %d not found and no cached position available", current_node);
          vel_pub_->publish(vec_vel);
          return;
        }
      }

      // ── coordinate transforms ─────────────────────────────────────────── //
      double x_cam = active_tvec[0];
      double y_cam = active_tvec[1];

      double x_cam_corr =  x_cam;
      double y_cam_corr = -y_cam;

      double cam_yaw = -camera_yaw_deg_ * M_PI / 180.0;

      double x_body =  x_cam_corr * cos(cam_yaw) + y_cam_corr * sin(cam_yaw);
      double y_body = -x_cam_corr * sin(cam_yaw) + y_cam_corr * cos(cam_yaw);

      double yaw = -current_heading_deg_ * M_PI / 180.0;

      double x_world =  x_body * cos(yaw) - y_body * sin(yaw);
      double y_world =  x_body * sin(yaw) + y_body * cos(yaw);

      // ── PID ───────────────────────────────────────────────────────────── //
      double x_control = x_pid->get_action(x_world);
      double y_control = y_pid->get_action(y_world);

      vec_vel.twist.linear.x = x_control;
      vec_vel.twist.linear.y = y_control;

      // ── alignment logic ───────────────────────────────────────────────── //
      // Only consider "aligned" when we have a live detection to avoid acting
      // on a stale cached pose that may be far from reality.
      bool aligned = !using_cached &&
                     (std::abs(x_cam) < aruco_tolerance &&
                      std::abs(y_cam) < aruco_tolerance);

      if (aligned) {
        if (stm.state == StateMachine::Landing) {
          if (active_tvec[2] <= 0.7) {
            stm.transition(StateMachine::Touchdown);
            this->land();
          } else {
            vec_vel.twist.linear.z = down_speed;
          }
        } else {
          if (!alignment_started_) {
            alignment_started_ = true;
            stm.transition(StateMachine::Pause);

            RCLCPP_INFO(this->get_logger(),
              "Aligned with node %d — advancing in 5 s", current_node);

            alignment_timer_ = this->create_wall_timer(
              std::chrono::seconds(5),
              [this]() {
                alignment_timer_->cancel();
                alignment_started_ = false;

                current_node = stm.get_next_node();
                if (current_node == -1) {
                  stm.transition(StateMachine::Land);
                  this->land();
                  return;
                }

                stm.transition(StateMachine::Resume);
              });
          }

          // Stop movement while aligned
          vec_vel.twist.linear.x = 0.0;
          vec_vel.twist.linear.y = 0.0;
        }
      }

      // ── ALWAYS publish ────────────────────────────────────────────────── //
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
      }
    }

    void heading_cb(const std_msgs::msg::Float64::SharedPtr msg) {
      current_heading_deg_ = msg->data;
      heading_received_    = true;
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Heading: %.2f deg", current_heading_deg_);
    }

  public:
    Executor()
      : rclcpp::Node("MainMission"),
        dictionary_     (cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250)),
        detector_params_(cv::aruco::DetectorParameters::create())
    {
      // ── parameters ────────────────────────────────────────────────── //
      this->declare_parameter("odometry_topic",    "/mavros/local_position/odom");
      this->declare_parameter("image_topic",       "/camera/camera/color/image_raw");
      this->declare_parameter("camera_info_topic", "/camera/camera/color/camera_info");
      this->declare_parameter("twist_topic",       "/mavros/setpoint_velocity/cmd_vel");
      this->declare_parameter("marker_size",       0.15);
      this->declare_parameter("kp_x",  0.0);
      this->declare_parameter("ki_x",  0.0);
      this->declare_parameter("kd_x",  0.0);
      this->declare_parameter("kp_y",  0.0);
      this->declare_parameter("ki_y",  0.0);
      this->declare_parameter("kd_y",  0.0);
      this->declare_parameter("up_speed",        0.3);
      this->declare_parameter("down_speed",      -0.1);
      this->declare_parameter("flight_height",    2.3);
      this->declare_parameter("aruco_tolerance",  0.2);
      this->declare_parameter("camera_yaw_deg",   0.0);

      kp_x = this->get_parameter("kp_x").as_double();
      ki_x = this->get_parameter("ki_x").as_double();
      kd_x = this->get_parameter("kd_x").as_double();
      kp_y = this->get_parameter("kp_y").as_double();
      ki_y = this->get_parameter("ki_y").as_double();
      kd_y = this->get_parameter("kd_y").as_double();

      camera_yaw_deg_ = this->get_parameter("camera_yaw_deg").as_double();
      up_speed        = this->get_parameter("up_speed").as_double();
      down_speed      = this->get_parameter("down_speed").as_double();
      flight_height   = this->get_parameter("flight_height").as_double();
      marker_size     = static_cast<float>(
        this->get_parameter("marker_size").as_double());
      aruco_tolerance = this->get_parameter("aruco_tolerance").as_double();

      x_pid = std::make_shared<PID>("X_PID", this->get_clock(), kp_x, ki_x, kd_x);
      y_pid = std::make_shared<PID>("Y_PID", this->get_clock(), kp_y, ki_y, kd_y);

      std::string odometry_topic    = this->get_parameter("odometry_topic").as_string();
      std::string image_topic       = this->get_parameter("image_topic").as_string();
      std::string twist_topic       = this->get_parameter("twist_topic").as_string();
      std::string camera_info_topic = this->get_parameter("camera_info_topic").as_string();

      auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();

      // ── subscriptions ─────────────────────────────────────────────── //
      state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
        "/mavros/state", 10,
        std::bind(&Executor::state_cb, this, std::placeholders::_1));

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

      heading_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/mavros/global_position/compass_hdg", qos,
        std::bind(&Executor::heading_cb, this, std::placeholders::_1));

      // ── publishers ────────────────────────────────────────────────── //
      vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        twist_topic, rclcpp::QoS(1).reliable());

      hover_ready_pub_ = this->create_publisher<std_msgs::msg::Empty>(
        "/main_mission/detection_start", 10);

      // ── service clients ───────────────────────────────────────────── //
      mode_client_ = this->create_client<mavros_msgs::srv::SetMode>(
        "/mavros/set_mode");
      arm_client_ = this->create_client<mavros_msgs::srv::CommandBool>(
        "/mavros/cmd/arming");

      print_all_params();

      RCLCPP_INFO(this->get_logger(),
        "Starting heartbeat. Pre-streaming setpoints for %.1f s...",
        STARTUP_TICKS * 0.1);

      heartbeat_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&Executor::heartbeat_cb, this));
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