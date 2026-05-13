#pragma once

// TODO
// - remove m_ before private members
// - add const to member functions
// - fix includes in all files
// - should we rclcpp::shutdown in construction instead?

// std
#include <atomic>
#include <condition_variable>
#include <cstdio>
#include <deque>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// ROS
#include <builtin_interfaces/msg/time.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <opencv2/core.hpp>

// Arena SDK
#include "ArenaApi.h"

class ArenaCameraNode : public rclcpp::Node
{
 public:
  ArenaCameraNode() : Node("arena_camera_node")
  {
    // Set stdout buffer size for ROS-defined size BUFSIZ.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    log_info(std::string("Creating \"") + this->get_name() + "\" node");
    parse_parameters_();
    initialize_();
    log_info(std::string("Created \"") + this->get_name() + "\" node");
  }

  ~ArenaCameraNode()
  {
    running_.store(false, std::memory_order_release);
    publish_queue_cv_.notify_all();
    if (acquisition_thread_.joinable()) {
      acquisition_thread_.join();
    }
    if (publish_worker_thread_.joinable()) {
      publish_worker_thread_.join();
    }
    log_info(std::string("Destroying \"") + this->get_name() + "\" node");
  }

  void log_debug(const std::string& msg)
  {
    RCLCPP_DEBUG(this->get_logger(), "%s", msg.c_str());
  }

  void log_info(const std::string& msg)
  {
    RCLCPP_INFO(this->get_logger(), "%s", msg.c_str());
  }

  void log_warn(const std::string& msg)
  {
    RCLCPP_WARN(this->get_logger(), "%s", msg.c_str());
  }

  void log_err(const std::string& msg)
  {
    RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
  }

 private:
  // Arena SDK
  std::shared_ptr<Arena::ISystem> m_pSystem;
  std::shared_ptr<Arena::IDevice> m_pDevice;

  // ROS publishers/subscribers/timers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr camera_diag_pub_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr record_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr session_path_sub_;

  rclcpp::TimerBase::SharedPtr m_wait_for_device_timer_callback_;

  // Raw recording state
  std::atomic<bool> record_raw_{false};
  std::string raw_save_root_;
  std::filesystem::path raw_save_dir_;

  // Camera selection
  std::string serial_;
  bool is_passed_serial_{false};

  // Publishing
  std::string topic_;
  struct PublishFrame {
    builtin_interfaces::msg::Time stamp;
    std::string frame_id;
    size_t width{0};
    size_t height{0};
    std::vector<uint8_t> bgra;
    std::chrono::steady_clock::time_point queued_at;
    double get_image_ms{0.0};
    double raw_save_ms{0.0};
    double arena_convert_ms{0.0};
    double full_frame_copy_ms{0.0};
  };
  std::atomic<bool> running_{true};
  std::thread acquisition_thread_;
  std::thread publish_worker_thread_;
  std::mutex publish_queue_mutex_;
  std::condition_variable publish_queue_cv_;
  std::deque<PublishFrame> publish_queue_;
  static constexpr size_t kMaxPublishQueueSize = 2;
  std::atomic<uint64_t> timing_frame_count_{0};
  std::atomic<uint64_t> publish_timing_frame_count_{0};

  // ROI
  size_t width_{0};
  bool is_passed_width{false};

  size_t height_{0};
  bool is_passed_height{false};

  // Gain/exposure parameters
  double gain_{-1.0};
  bool is_passed_gain_{false};

  std::string gain_auto_;
  bool is_passed_gain_auto_{false};

  double exposure_time_{-1.0};
  bool is_passed_exposure_time_{false};

  std::string exposure_auto_;
  bool is_passed_exposure_auto_{false};

  // Pixel format
  std::string pixelformat_pfnc_;
  std::string pixelformat_ros_;
  bool is_passed_pixelformat_ros_{false};

  // Trigger/acquisition mode
  bool hardware_trigger_{false};

  // QoS
  std::string pub_qos_history_;
  bool is_passed_pub_qos_history_{false};

  size_t pub_qos_history_depth_{0};
  bool is_passed_pub_qos_history_depth_{false};

  std::string pub_qos_reliability_;
  bool is_passed_pub_qos_reliability_{false};

  // Runtime parameter / diagnostics support
  std::mutex camera_param_mutex_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // Setup
  void parse_parameters_();
  void initialize_();
  void wait_for_device_timer_callback_();
  void run_();

  // Device creation/configuration
  Arena::IDevice* create_device_ros_();

  void set_nodes_();
  void set_nodes_load_default_profile_();
  void set_nodes_roi_();
  void set_nodes_gain_();
  void set_nodes_pixelformat_();
  void set_nodes_exposure_();
  void set_nodes_auto_exposure_gain_();
  void set_nodes_trigger_mode_();
  void set_nodes_test_pattern_image_();

  // Main acquisition pipeline
  void publish_images_();
  void resize_and_publish_worker_();
  void enqueue_publish_frame_(PublishFrame frame);
  cv::Mat resize_with_vpi_vic_(const PublishFrame& frame);

  // Raw recording
  void record_mode_callback_(const std_msgs::msg::String::SharedPtr msg);
  void session_path_callback_(const std_msgs::msg::String::SharedPtr msg);
  void save_raw_image_(Arena::IImage* pImage);
  std::filesystem::path make_raw_save_dir_();

  // Timestamp helpers
  builtin_interfaces::msg::Time timestamp_from_image_(Arena::IImage* pImage) const;
  std::string timestamp_filename_stem_(Arena::IImage* pImage) const;

  // Diagnostics / parameters
  rcl_interfaces::msg::SetParametersResult on_set_parameters_(
      const std::vector<rclcpp::Parameter>& params);

  void publish_camera_diagnostics_();
  void declare_tunable_parameters_();
  void apply_initial_tunable_parameters_();
};
