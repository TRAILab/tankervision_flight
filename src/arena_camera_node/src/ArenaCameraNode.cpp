#include <atomic>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

#include <cstring>    // memcopy
#include <stdexcept>  // std::runtime_err
#include <string>
#include <chrono>
#include <thread>

// OpenCV
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>

// ROS
#include "rmw/types.h"
#include <sensor_msgs/image_encodings.hpp>
#include "rclcpp/rclcpp.hpp"

// ArenaSDK
#include "ArenaCameraNode.h"
#include "light_arena/deviceinfo_helper.h"
#include "rclcpp_adapter/pixelformat_translation.h"
#include "rclcpp_adapter/quilty_of_service_translation.cpp"

void ArenaCameraNode::parse_parameters_()
{
  std::string nextParameterToDeclare = "";
  try {
    // NOTE: serial_ is treated as a string. If your launch config passes it as
    // an integer, convert at the call site or use declare_parameter<int> here.
    nextParameterToDeclare = "serial";
    serial_ = this->declare_parameter<std::string>("serial", "");
    is_passed_serial_ = serial_ != "";

    nextParameterToDeclare = "pixelformat";
    pixelformat_ros_ = this->declare_parameter("pixelformat", "");
    is_passed_pixelformat_ros_ = pixelformat_ros_ != "";

    nextParameterToDeclare = "width";
    width_ = this->declare_parameter("width", 0);
    is_passed_width = width_ > 0;

    nextParameterToDeclare = "height";
    height_ = this->declare_parameter("height", 0);
    is_passed_height = height_ > 0;

    nextParameterToDeclare = "gain";
    gain_ = this->declare_parameter("gain", -1.0);
    is_passed_gain_ = gain_ >= 0;

    nextParameterToDeclare = "gain_auto";
    gain_auto_ = this->declare_parameter<std::string>("gain_auto", "");
    is_passed_gain_auto_ = gain_auto_ != "";

    nextParameterToDeclare = "exposure_time";
    exposure_time_ = this->declare_parameter("exposure_time", -1.0);
    is_passed_exposure_time_ = exposure_time_ >= 0;

    nextParameterToDeclare = "exposure_auto";
    exposure_auto_ = this->declare_parameter<std::string>("exposure_auto", "");
    is_passed_exposure_auto_ = exposure_auto_ != "";

    nextParameterToDeclare = "trigger_mode";
    trigger_mode_activated_ = this->declare_parameter("trigger_mode", false);
    // no need to is_passed_trigger_mode_ because it is already a boolean

    nextParameterToDeclare = "topic";
    topic_ = this->declare_parameter(
        "topic", std::string("/") + this->get_name() + "/images");
    // no need to is_passed_topic_

    nextParameterToDeclare = "qos_history";
    pub_qos_history_ = this->declare_parameter("qos_history", "");
    is_passed_pub_qos_history_ = pub_qos_history_ != "";

    nextParameterToDeclare = "qos_history_depth";
    pub_qos_history_depth_ = this->declare_parameter("qos_history_depth", 0);
    is_passed_pub_qos_history_depth_ = pub_qos_history_depth_ > 0;

    nextParameterToDeclare = "qos_reliability";
    pub_qos_reliability_ = this->declare_parameter("qos_reliability", "");
    is_passed_pub_qos_reliability_ = pub_qos_reliability_ != "";

  } catch (rclcpp::ParameterTypeException& e) {
    log_err(nextParameterToDeclare + " argument");
    throw;
  }
}

void ArenaCameraNode::declare_tunable_parameters_()
{
  auto declare_if_new = [this](auto name, auto default_val) {
    if (!this->has_parameter(name))
      this->declare_parameter(name, default_val);
  };

  declare_if_new("exposure_auto",   exposure_auto_.empty()  ? "Off" : exposure_auto_);
  declare_if_new("exposure_time",   exposure_time_ >= 0.0   ? exposure_time_ : 20000.0);
  declare_if_new("gain_auto",       gain_auto_.empty()      ? "Off" : gain_auto_);
  declare_if_new("gain",            gain_ >= 0.0            ? gain_ : 0.0);

  // these are new so they're fine as-is
  declare_if_new("target_brightness",           128);
  declare_if_new("exposure_auto_lower_limit",   100.0);
  declare_if_new("exposure_auto_upper_limit",   30000.0);
  declare_if_new("exposure_auto_algorithm",     std::string("Mean"));
  declare_if_new("exposure_auto_damping",       50.0);
  declare_if_new("auto_exposure_aoi_enable",    false);
  declare_if_new("auto_exposure_aoi_width",     0);
  declare_if_new("auto_exposure_aoi_height",    0);
  declare_if_new("auto_exposure_aoi_offset_x",  0);
  declare_if_new("auto_exposure_aoi_offset_y",  0);
  declare_if_new("gamma",                       1.0);
  declare_if_new("acquisition_frame_rate_enable", false);
  declare_if_new("acquisition_frame_rate",      0.0);
}

void ArenaCameraNode::initialize_()
{
  using namespace std::chrono_literals;
  // ARENASDK ---------------------------------------------------------------
  // Custom deleter for system
  m_pSystem =
      std::shared_ptr<Arena::ISystem>(nullptr, [=](Arena::ISystem* pSystem) {
        if (pSystem) {  // this is an issue for multi devices
          Arena::CloseSystem(pSystem);
          log_info("System is destroyed");
        }
      });
  m_pSystem.reset(Arena::OpenSystem());

  // Custom deleter for device
  m_pDevice =
      std::shared_ptr<Arena::IDevice>(nullptr, [=](Arena::IDevice* pDevice) {
        if (m_pSystem && pDevice) {
          m_pSystem->DestroyDevice(pDevice);
          log_info("Device is destroyed");
        }
      });

  //
  // CHECK DEVICE CONNECTION ( timer ) --------------------------------------
  //
  // TODO
  // - Think of design that allow the node to start stream as soon as
  // it is initialized without waiting for spin to be called
  // - maybe change 1s to a smaller value
  m_wait_for_device_timer_callback_ = this->create_wall_timer(
      1s, std::bind(&ArenaCameraNode::wait_for_device_timer_callback_, this));

  //
  // TRIGGER (service) ------------------------------------------------------
  //
  using namespace std::placeholders;
  m_trigger_an_image_srv_ = this->create_service<std_srvs::srv::Trigger>(
      std::string(this->get_name()) + "/save_images_trigger",
      std::bind(&ArenaCameraNode::publish_an_image_on_trigger_, this, _1, _2));

  //
  // Publisher --------------------------------------------------------------
  //
  // m_pub_qos is rclcpp::SensorDataQoS has these defaults
  // https://github.com/ros2/rmw/blob/fb06b57975373b5a23691bb00eb39c07f1660ed7/rmw/include/rmw/qos_profiles.h#L25

  /*
  static const rmw_qos_profile_t rmw_qos_profile_sensor_data =
  {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    5, // history depth
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false // avoid ros namespace conventions
  };
  */
  rclcpp::SensorDataQoS pub_qos_;
  // QoS history
  if (is_passed_pub_qos_history_) {
    if (is_supported_qos_histroy_policy(pub_qos_history_)) {
      pub_qos_.history(
          K_CMDLN_PARAMETER_TO_QOS_HISTORY_POLICY[pub_qos_history_]);
    } else {
      log_err(pub_qos_history_ + " is not supported for this node");
      // TODO
      // should thorow instead??
      // should this keeps shutting down if for some reasons this node is kept
      // alive
      throw;
    }
  }
  // QoS depth
  if (is_passed_pub_qos_history_depth_ &&
      K_CMDLN_PARAMETER_TO_QOS_HISTORY_POLICY[pub_qos_history_] ==
          RMW_QOS_POLICY_HISTORY_KEEP_LAST) {
    // TODO
    // test err msg withwhen -1
    pub_qos_.keep_last(pub_qos_history_depth_);
  }

  // Qos reliability
  if (is_passed_pub_qos_reliability_) {
    if (is_supported_qos_reliability_policy(pub_qos_reliability_)) {
      pub_qos_.reliability(
          K_CMDLN_PARAMETER_TO_QOS_RELIABILITY_POLICY[pub_qos_reliability_]);
    } else {
      log_err(pub_qos_reliability_ + " is not supported for this node");
      throw;
    }
  }

  m_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      this->get_parameter("topic").as_string(), pub_qos_);

  // Create heartbeat publisher on topic: "/camera_heartbeat"
  heartbeat_pub_ = this->create_publisher<std_msgs::msg::String>(
      std::string("/camera_heartbeat"), 10);

  // Create Lucid diagnostics publisher
  camera_diag_pub_ = this->create_publisher<std_msgs::msg::String>(
      std::string("/camera/lucid_diagnostics"), 10);

  save_trigger_sub_ = this->create_subscription<std_msgs::msg::Empty>(
    "/save_images_trigger",
    rclcpp::QoS(10).reliable(),
    std::bind(&ArenaCameraNode::save_next_raw_callback_, this, std::placeholders::_1));

  raw_save_dir_ = make_raw_save_dir_();
  log_info("Subscribed to /save_images_trigger for next-frame raw saves");

  //  Declare Tunable Parameters
  declare_tunable_parameters_();
  apply_initial_tunable_parameters_();

  param_cb_handle_ = this->add_on_set_parameters_callback(
      std::bind(&ArenaCameraNode::on_set_parameters_, this, std::placeholders::_1));

  std::stringstream pub_qos_info;
  auto pub_qos_profile = pub_qos_.get_rmw_qos_profile();
  pub_qos_info
      << '\t' << "QoS history     = "
      << K_QOS_HISTORY_POLICY_TO_CMDLN_PARAMETER[pub_qos_profile.history]
      << '\n';
  pub_qos_info << "\t\t\t\t"
               << "QoS depth       = " << pub_qos_profile.depth << '\n';
  pub_qos_info << "\t\t\t\t"
               << "QoS reliability = "
               << K_QOS_RELIABILITY_POLICY_TO_CMDLN_PARAMETER[pub_qos_profile
                                                                  .reliability]
               << '\n';

  log_info(pub_qos_info.str());
}

std::filesystem::path ArenaCameraNode::make_raw_save_dir_()
{
  auto now = std::chrono::system_clock::now();
  std::time_t now_c = std::chrono::system_clock::to_time_t(now);

  std::tm local_tm{};
  localtime_r(&now_c, &local_tm);

  std::ostringstream date_stream;
  date_stream << std::put_time(&local_tm, "%Y-%m-%d");

  std::ostringstream session_stream;
  session_stream << "session_" << std::put_time(&local_tm, "%H-%M-%S");

  std::filesystem::path dir =
      std::filesystem::path(raw_save_root_) / date_stream.str() / session_stream.str();

  std::error_code ec;
  std::filesystem::create_directories(dir, ec);
  if (ec) {
    throw std::runtime_error(
        "Failed to create raw save directory: " + dir.string() + " : " + ec.message());
  }

  log_info("Created raw save directory: " + dir.string());
  return dir;
}

void ArenaCameraNode::apply_initial_tunable_parameters_()
{
  if (!m_pDevice) {
    return;
  }

  auto nodemap = m_pDevice->GetNodeMap();

  std::lock_guard<std::mutex> lock(camera_param_mutex_);

  Arena::SetNodeValue<GenICam::gcstring>(
      nodemap, "ExposureAuto",
      this->get_parameter("exposure_auto").as_string().c_str());

  if (this->get_parameter("exposure_auto").as_string() == "Off") {
    Arena::SetNodeValue<double>(
        nodemap, "ExposureTime",
        this->get_parameter("exposure_time").as_double());
  }

  Arena::SetNodeValue<GenICam::gcstring>(
      nodemap, "GainAuto",
      this->get_parameter("gain_auto").as_string().c_str());

  if (this->get_parameter("gain_auto").as_string() == "Off") {
    Arena::SetNodeValue<double>(
        nodemap, "Gain",
        this->get_parameter("gain").as_double());
  }

  Arena::SetNodeValue<int64_t>(nodemap, "TargetBrightness",
    static_cast<int64_t>(this->get_parameter("target_brightness").as_double()));

  Arena::SetNodeValue<double>(
      nodemap, "ExposureAutoLowerLimit",
      this->get_parameter("exposure_auto_lower_limit").as_double());

  Arena::SetNodeValue<double>(
      nodemap, "ExposureAutoUpperLimit",
      this->get_parameter("exposure_auto_upper_limit").as_double());

  Arena::SetNodeValue<GenICam::gcstring>(
      nodemap, "ExposureAutoAlgorithm",
      this->get_parameter("exposure_auto_algorithm").as_string().c_str());

  Arena::SetNodeValue<double>(
      nodemap, "ExposureAutoDamping",
      this->get_parameter("exposure_auto_damping").as_double());

  Arena::SetNodeValue<bool>(
      nodemap, "AutoExposureAOIEnable",
      this->get_parameter("auto_exposure_aoi_enable").as_bool());

  const int aoi_w = this->get_parameter("auto_exposure_aoi_width").as_int();
  const int aoi_h = this->get_parameter("auto_exposure_aoi_height").as_int();
  const int aoi_x = this->get_parameter("auto_exposure_aoi_offset_x").as_int();
  const int aoi_y = this->get_parameter("auto_exposure_aoi_offset_y").as_int();

  if (aoi_w > 0) Arena::SetNodeValue<int64_t>(nodemap, "AutoExposureAOIWidth", aoi_w);
  if (aoi_h > 0) Arena::SetNodeValue<int64_t>(nodemap, "AutoExposureAOIHeight", aoi_h);
  Arena::SetNodeValue<int64_t>(nodemap, "AutoExposureAOIOffsetX", aoi_x);
  Arena::SetNodeValue<int64_t>(nodemap, "AutoExposureAOIOffsetY", aoi_y);

  Arena::SetNodeValue<double>(
      nodemap, "Gamma",
      this->get_parameter("gamma").as_double());

  Arena::SetNodeValue<bool>(
      nodemap, "AcquisitionFrameRateEnable",
      this->get_parameter("acquisition_frame_rate_enable").as_bool());

  if (this->get_parameter("acquisition_frame_rate_enable").as_bool()) {
    const double fps = this->get_parameter("acquisition_frame_rate").as_double();
    if (fps > 0.0) {
      Arena::SetNodeValue<double>(nodemap, "AcquisitionFrameRate", fps);
    }
  }

  publish_camera_diagnostics_();
}


void ArenaCameraNode::wait_for_device_timer_callback_()
{
  // something happend while checking for cameras
  if (!rclcpp::ok()) {
    log_err("Interrupted while waiting for arena camera. Exiting.");
    rclcpp::shutdown();
  }

  // camera discovery
  m_pSystem->UpdateDevices(100);  // in millisec
  auto device_infos = m_pSystem->GetDevices();

  // no camera is connected
  if (!device_infos.size()) {
    log_info("No arena camera is connected. Waiting for device(s)...");
  }
  // at least on is found
  else {
    m_wait_for_device_timer_callback_->cancel();
    log_info(std::to_string(device_infos.size()) +
             " arena device(s) has been discoved.");
    run_();
  }
}

void ArenaCameraNode::save_next_raw_callback_(const std_msgs::msg::Empty::SharedPtr /*msg*/)
{
  save_next_raw_.store(true, std::memory_order_release);
  log_info("Received raw save trigger; next retrieved image will be saved in raw format");
}

rcl_interfaces::msg::SetParametersResult ArenaCameraNode::on_set_parameters_(
    const std::vector<rclcpp::Parameter>& params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  if (!m_pDevice) {
    result.successful = false;
    result.reason = "device not initialized";
    return result;
  }

  auto nodemap = m_pDevice->GetNodeMap();
  std::lock_guard<std::mutex> lock(camera_param_mutex_);

  try {
    for (const auto& param : params) {
      const auto& name = param.get_name();

      if (name == "exposure_auto") {
        Arena::SetNodeValue<GenICam::gcstring>(
            nodemap, "ExposureAuto", param.as_string().c_str());
      } else if (name == "exposure_time") {
        Arena::SetNodeValue<double>(nodemap, "ExposureTime", param.as_double());
      } else if (name == "gain_auto") {
        Arena::SetNodeValue<GenICam::gcstring>(
            nodemap, "GainAuto", param.as_string().c_str());
      } else if (name == "gain") {
        Arena::SetNodeValue<double>(nodemap, "Gain", param.as_double());
      } else if (name == "target_brightness") {
        Arena::SetNodeValue<int64_t>(nodemap, "TargetBrightness",
        static_cast<int64_t>(param.as_double()));
      } else if (name == "exposure_auto_lower_limit") {
        Arena::SetNodeValue<double>(nodemap, "ExposureAutoLowerLimit", param.as_double());
      } else if (name == "exposure_auto_upper_limit") {
        Arena::SetNodeValue<double>(nodemap, "ExposureAutoUpperLimit", param.as_double());
      } else if (name == "exposure_auto_algorithm") {
        Arena::SetNodeValue<GenICam::gcstring>(
            nodemap, "ExposureAutoAlgorithm", param.as_string().c_str());
      } else if (name == "exposure_auto_damping") {
        Arena::SetNodeValue<double>(nodemap, "ExposureAutoDamping", param.as_double());
      } else if (name == "auto_exposure_aoi_enable") {
        Arena::SetNodeValue<bool>(nodemap, "AutoExposureAOIEnable", param.as_bool());
      } else if (name == "auto_exposure_aoi_width") {
        Arena::SetNodeValue<int64_t>(nodemap, "AutoExposureAOIWidth", param.as_int());
      } else if (name == "auto_exposure_aoi_height") {
        Arena::SetNodeValue<int64_t>(nodemap, "AutoExposureAOIHeight", param.as_int());
      } else if (name == "auto_exposure_aoi_offset_x") {
        Arena::SetNodeValue<int64_t>(nodemap, "AutoExposureAOIOffsetX", param.as_int());
      } else if (name == "auto_exposure_aoi_offset_y") {
        Arena::SetNodeValue<int64_t>(nodemap, "AutoExposureAOIOffsetY", param.as_int());
      } else if (name == "gamma") {
        Arena::SetNodeValue<double>(nodemap, "Gamma", param.as_double());
      } else if (name == "acquisition_frame_rate_enable") {
        Arena::SetNodeValue<bool>(nodemap, "AcquisitionFrameRateEnable", param.as_bool());
      } else if (name == "acquisition_frame_rate") {
        Arena::SetNodeValue<double>(nodemap, "AcquisitionFrameRate", param.as_double());
      }
    }

    publish_camera_diagnostics_();
  } catch (const std::exception& e) {
    result.successful = false;
    result.reason = e.what();
  } catch (const GenICam::GenericException& e) {
    result.successful = false;
    result.reason = e.what();
  }

  return result;
}

void ArenaCameraNode::run_()
{
  auto device = create_device_ros_();
  m_pDevice.reset(device);
  set_nodes_();
  m_pDevice->StartStream();

  if (!trigger_mode_activated_) {
    std::thread(&ArenaCameraNode::publish_images_, this).detach();
  } else {
    // executor remains free for callbacks/services
  }
}

void ArenaCameraNode::save_raw_image_(Arena::IImage* pImage)
{
  if (!pImage) {
    log_warn("save_raw_image_: pImage is null");
    return;
  }

  const auto t0 = std::chrono::steady_clock::now();

  const uint64_t frame_id = pImage->GetFrameId();
  const uint64_t width = pImage->GetWidth();
  const uint64_t height = pImage->GetHeight();
  const uint64_t bits_per_pixel = pImage->GetBitsPerPixel();
  const size_t bytes_per_pixel = static_cast<size_t>((bits_per_pixel + 7) / 8);
  const size_t data_size = static_cast<size_t>(width) *
                           static_cast<size_t>(height) *
                           bytes_per_pixel;

  const auto now = std::chrono::system_clock::now();
  const auto now_time_t = std::chrono::system_clock::to_time_t(now);
  const auto now_us =
      std::chrono::duration_cast<std::chrono::microseconds>(
          now.time_since_epoch()).count() % 1000000;

  std::tm tm_buf{};
  localtime_r(&now_time_t, &tm_buf);

  std::ostringstream name;
  name << raw_save_dir_
       << "/frame_" << frame_id
       << "_" << std::put_time(&tm_buf, "%Y%m%d_%H%M%S")
       << "_" << std::setw(6) << std::setfill('0') << now_us
       << "_" << width << "x" << height
       << "_" << bits_per_pixel << "bpp.raw";

  const void* src = pImage->GetData();
  if (!src) {
    log_warn("save_raw_image_: image data pointer is null");
    return;
  }

  std::ofstream ofs(name.str(), std::ios::binary);
  if (!ofs) {
    log_warn("Failed to open raw output file: " + name.str());
    return;
  }

  ofs.write(reinterpret_cast<const char*>(src), static_cast<std::streamsize>(data_size));
  ofs.close();

  const auto t1 = std::chrono::steady_clock::now();
  const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();

  log_info(
      "Saved raw image: " + name.str() +
      " (" + std::to_string(data_size) + " bytes, " +
      std::to_string(ms) + " ms)");
}

void ArenaCameraNode::publish_images_()
{
  Arena::IImage* pImage = nullptr;

  while (rclcpp::ok()) {
    try {
      pImage = m_pDevice->GetImage(1000);

      if (save_next_raw_.exchange(false, std::memory_order_acq_rel)) {
        try {
          save_raw_image_(pImage);
        } catch (const std::exception& e) {
          log_warn(std::string("Failed to save raw image: ") + e.what());
        } catch (...) {
          log_warn("Failed to save raw image: unknown exception");
        }
      }

      const size_t width = pImage->GetWidth();
      const size_t height = pImage->GetHeight();

      if (width != 5320 || height != 4600) {
        log_warn(
          "Unexpected image size: " + std::to_string(width) + "x" +
          std::to_string(height) + ", expected 5320x4600");
      }

      log_info("Pixel format: " + std::to_string(static_cast<uint64_t>(pImage->GetPixelFormat())));
      Arena::IImage* converted = nullptr;
      try {
        converted = Arena::ImageFactory::Convert(pImage, PFNC_BGR8);

        const size_t conv_width = converted->GetWidth();
        const size_t conv_height = converted->GetHeight();

        cv::Mat bgr(
          static_cast<int>(conv_height),
          static_cast<int>(conv_width),
          CV_8UC3,
          (void*)converted->GetData()
        );

        cv::Mat bgr_half;
        cv::resize(
          bgr,
          bgr_half,
          cv::Size(static_cast<int>(conv_width / 2), static_cast<int>(conv_height / 2)),
          0.0,
          0.0,
          cv::INTER_AREA);

        auto p_image_msg = std::make_unique<sensor_msgs::msg::Image>();
        p_image_msg->header.stamp = this->now();
        p_image_msg->header.frame_id = std::to_string(pImage->GetFrameId());
        p_image_msg->height = bgr_half.rows;
        p_image_msg->width = bgr_half.cols;
        p_image_msg->encoding = sensor_msgs::image_encodings::BGR8;
        p_image_msg->is_bigendian = 0;
        p_image_msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(bgr_half.step);

        const size_t data_size = bgr_half.total() * bgr_half.elemSize();
        p_image_msg->data.resize(data_size);
        std::memcpy(p_image_msg->data.data(), bgr_half.data, data_size);

        m_pub_->publish(std::move(p_image_msg));

        this->m_pDevice->RequeueBuffer(pImage);
        pImage = nullptr;

        Arena::ImageFactory::Destroy(converted);
        converted = nullptr;
      } catch (...) {
        if (converted) {
          Arena::ImageFactory::Destroy(converted);
          converted = nullptr;
        }
        throw;
      }


    } catch (std::exception& e) {
      if (pImage) {
        this->m_pDevice->RequeueBuffer(pImage);
        pImage = nullptr;
      }

      log_warn(
        std::string("Exception occurred while publishing an image\n") +
        e.what());
    }
  }
}

void ArenaCameraNode::msg_form_image_(Arena::IImage* pImage,
                                      sensor_msgs::msg::Image& image_msg)
{
  try {
    if (!pImage) {
      throw std::runtime_error("pImage pointer is null.");
    }
    // 1 ) Header
    //      - stamp.sec
    //      - stamp.nanosec
    //      - Frame ID
    image_msg.header.stamp.sec =
       static_cast<uint32_t>(pImage->GetTimestampNs() / 1000000000);
    image_msg.header.stamp.nanosec =
       static_cast<uint32_t>(pImage->GetTimestampNs() % 1000000000);
    image_msg.header.frame_id = std::to_string(pImage->GetFrameId());

    //
    // 2 ) Height
    //
    image_msg.height = height_;

    //
    // 3 ) Width
    //
    uint64_t device_width = pImage->GetWidth();
    if (device_width != static_cast<uint64_t>(width_)) {
      log_warn("Device reported width (" + std::to_string(device_width) +
               ") does not match expected width (" + std::to_string(width_) +
               "). Using expected width.");
    }
    image_msg.width = width_;

    //
    // 4 ) encoding
    //
    image_msg.encoding = pixelformat_ros_;

    //
    // 5 ) is_big_endian
    //
    // TODO what to do if unknown
    image_msg.is_bigendian = pImage->GetPixelEndianness() ==
                             Arena::EPixelEndianness::PixelEndiannessBig;
    //
    // 6 ) step
    //
    // TODO could be optimized by moving it out
    auto bits_per_pixel = pImage->GetBitsPerPixel();
    if (bits_per_pixel <= 0) {
      throw std::runtime_error("Invalid bits per pixel reported by device.");
    }
    auto pixel_length_in_bytes = bits_per_pixel / 8;
    if (pixel_length_in_bytes <= 0) {
      throw std::runtime_error("Computed pixel length in bytes is invalid.");
    }
    auto width_length_in_bytes = device_width * pixel_length_in_bytes;
    image_msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(width_length_in_bytes);

    //
    // 7) data
    //
    auto image_data_length_in_bytes = width_length_in_bytes * height_;
    if (image_data_length_in_bytes == 0) {
      throw std::runtime_error("Computed image data length is zero.");
    }
    image_msg.data.resize(image_data_length_in_bytes);

    // Safety check: ensure the source data pointer is valid.
    const void* src_ptr = pImage->GetData();
    if (!src_ptr) {
      throw std::runtime_error("pImage->GetData() returned a null pointer.");
    }

    // Perform the copy using memcpy.
    std::memcpy(&image_msg.data[0], src_ptr, image_data_length_in_bytes);
  } catch (...) {
    log_warn(
        "Failed to create Image ROS MSG. Published Image Msg might be "
        "corrupted");
  }
}

void ArenaCameraNode::publish_camera_diagnostics_()
{
  if (!m_pDevice) return;

  auto nodemap = m_pDevice->GetNodeMap();
  std::ostringstream ss;
  ss << "{";

  auto get_double = [&](const char* key, const char* node) {
    try {
      ss << "\"" << key << "\":" << Arena::GetNodeValue<double>(nodemap, node);
    } catch (...) {
      ss << "\"" << key << "\":null";
    }
  };
  auto get_string = [&](const char* key, const char* node) {
    try {
      ss << "\"" << key << "\":\"" << std::string(Arena::GetNodeValue<GenICam::gcstring>(nodemap, node)) << "\"";
    } catch (...) {
      ss << "\"" << key << "\":null";
    }
  };
  auto get_bool = [&](const char* key, const char* node) {
    try {
      ss << "\"" << key << "\":" << (Arena::GetNodeValue<bool>(nodemap, node) ? "true" : "false");
    } catch (...) {
      ss << "\"" << key << "\":null";
    }
  };
  auto get_int = [&](const char* key, const char* node) {
    try {
      ss << "\"" << key << "\":" << Arena::GetNodeValue<int64_t>(nodemap, node);
    } catch (...) {
      ss << "\"" << key << "\":null";
    }
  };

  get_string("exposure_auto",            "ExposureAuto");            ss << ",";
  get_double("exposure_time",            "ExposureTime");            ss << ",";
  get_string("gain_auto",                "GainAuto");                ss << ",";
  get_double("gain",                     "Gain");                    ss << ",";
  get_int   ("target_brightness",        "TargetBrightness");        ss << ",";
  get_double("exposure_auto_lower_limit","ExposureAutoLowerLimit");  ss << ",";
  get_double("exposure_auto_upper_limit","ExposureAutoUpperLimit");  ss << ",";
  get_string("exposure_auto_algorithm",  "ExposureAutoAlgorithm");   ss << ",";
  get_double("exposure_auto_damping",    "ExposureAutoDamping");     ss << ",";
  get_bool  ("auto_exposure_aoi_enable", "AutoExposureAOIEnable");   ss << ",";
  get_int   ("auto_exposure_aoi_width",  "AutoExposureAOIWidth");    ss << ",";
  get_int   ("auto_exposure_aoi_height", "AutoExposureAOIHeight");   ss << ",";
  get_int   ("auto_exposure_aoi_offset_x","AutoExposureAOIOffsetX"); ss << ",";
  get_int   ("auto_exposure_aoi_offset_y","AutoExposureAOIOffsetY"); ss << ",";
  get_double("gamma",                    "Gamma");                   ss << ",";
  get_bool  ("acquisition_frame_rate_enable","AcquisitionFrameRateEnable"); ss << ",";
  get_double("acquisition_frame_rate",   "AcquisitionFrameRate");    ss << ",";
  get_double("calculated_mean",          "CalculatedMean");          ss << ",";
  get_double("calculated_median",        "CalculatedMedian");

  ss << "}";

  std_msgs::msg::String msg;
  msg.data = ss.str();
  camera_diag_pub_->publish(msg);
}

void ArenaCameraNode::publish_an_image_on_trigger_(
    std::shared_ptr<std_srvs::srv::Trigger::Request> request /*unused*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!trigger_mode_activated_) {
    std::string msg =
        "Failed to trigger image because the device is not in trigger mode."
        "run `ros2 run arena_camera_node run --ros-args -p trigger_mode:=true`";
    log_warn(msg);
    response->message = msg;
    response->success = false;
  }

  log_info("A client triggered an image request");

  Arena::IImage* pImage = nullptr;
  try {
    // trigger
    bool triggerArmed = false;
    auto waitForTriggerCount = 10;
    do {
      // infinite loop when I step in (sometimes)
      triggerArmed =
          Arena::GetNodeValue<bool>(m_pDevice->GetNodeMap(), "TriggerArmed");

      if (triggerArmed == false && (waitForTriggerCount % 10) == 0) {
        log_info("waiting for trigger to be armed");
      }

    } while (triggerArmed == false);

    log_debug("trigger is armed; triggering an image");
    Arena::ExecuteNode(m_pDevice->GetNodeMap(), "TriggerSoftware");

    // get image
    auto p_image_msg = std::make_unique<sensor_msgs::msg::Image>();

    log_debug("getting an image");
    pImage = m_pDevice->GetImage(1000);
    auto msg = std::string("image ") + std::to_string(pImage->GetFrameId()) +
               " published to " + topic_;
    msg_form_image_(pImage, *p_image_msg);
    m_pub_->publish(std::move(p_image_msg));
    response->message = msg;
    response->success = true;

    log_info(msg);
    this->m_pDevice->RequeueBuffer(pImage);

  }

  catch (std::exception& e) {
    if (pImage) {
      this->m_pDevice->RequeueBuffer(pImage);
      pImage = nullptr;
    }
    auto msg =
        std::string("Exception occurred while grabbing an image\n") + e.what();
    log_warn(msg);
    response->message = msg;
    response->success = false;
  }

  catch (GenICam::GenericException& e) {
    if (pImage) {
      this->m_pDevice->RequeueBuffer(pImage);
      pImage = nullptr;
    }
    auto msg =
        std::string("GenICam Exception occurred while grabbing an image\n") +
        e.what();
    log_warn(msg);
    response->message = msg;
    response->success = false;
  }
}

Arena::IDevice* ArenaCameraNode::create_device_ros_()
{
  m_pSystem->UpdateDevices(100);  // in millisec
  auto device_infos = m_pSystem->GetDevices();
  if (!device_infos.size()) {
    // TODO: handel disconnection
    throw std::runtime_error(
        "camera(s) were disconnected after they were discovered");
  }

  auto index = 0;
  if (is_passed_serial_) {
    index = DeviceInfoHelper::get_index_of_serial(device_infos, serial_);
  }

  auto pDevice = m_pSystem->CreateDevice(device_infos.at(index));
  log_info(std::string("device created ") +
           DeviceInfoHelper::info(device_infos.at(index)));
  return pDevice;
}

void ArenaCameraNode::set_nodes_()
{
  Arena::SetNodeValue<int64_t>(m_pDevice->GetNodeMap(), "PacketResendWindowFrameCount", 8);
  log_info("\tPacket resend window: 8");
  Arena::SetNodeValue<int64_t>(m_pDevice->GetNodeMap(), "DeviceLinkThroughputReserve", 10);
  log_info("\tEthernet link reserve: 10");

  set_nodes_load_default_profile_();
  set_nodes_roi_();
  set_nodes_gain_();
  set_nodes_pixelformat_();
  set_nodes_exposure_();
  set_nodes_trigger_mode_();

  // configure Auto Negotiate Packet Size and Packet Resend
  Arena::SetNodeValue<bool>(m_pDevice->GetTLStreamNodeMap(), "StreamAutoNegotiatePacketSize", true);
  Arena::SetNodeValue<bool>(m_pDevice->GetTLStreamNodeMap(), "StreamPacketResendEnable", true);

  //set_nodes_test_pattern_image_();

  // PTP: enable and wait until camera is in Slave mode (60-second timeout)
  Arena::SetNodeValue(m_pDevice->GetNodeMap(), "PtpEnable", true);
  Arena::SetNodeValue(m_pDevice->GetNodeMap(), "PtpSlaveOnly", true);
  {
    const auto ptp_timeout = std::chrono::seconds(60);
    const auto ptp_poll_interval = std::chrono::seconds(5);
    const auto ptp_deadline = std::chrono::steady_clock::now() + ptp_timeout;

    GenICam::gcstring currPtpStatus =
        Arena::GetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(), "PtpStatus");

    while (currPtpStatus != "Slave") {
      if (std::chrono::steady_clock::now() >= ptp_deadline) {
        log_warn("PTP slave sync timed out after 60 seconds (last status: " +
                 std::string(currPtpStatus) + "). Continuing without PTP sync.");
        break;
      }
      log_warn("NOT IN SLAVE MODE: " + std::string(currPtpStatus));
      std::this_thread::sleep_for(ptp_poll_interval);
      currPtpStatus =
          Arena::GetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(), "PtpStatus");
    }

    if (currPtpStatus == "Slave") {
      log_info("PTP status: " + std::string(currPtpStatus));
    }
  }
}

void ArenaCameraNode::set_nodes_load_default_profile_()
{
  auto nodemap = m_pDevice->GetNodeMap();
  // device run on default profile all the time if no args are passed
  // otherwise, overwise only these params
  Arena::SetNodeValue<GenICam::gcstring>(nodemap, "UserSetSelector", "Default");
  // execute the profile
  Arena::ExecuteNode(nodemap, "UserSetLoad");
  log_info("\tdefault profile is loaded");
}

void ArenaCameraNode::set_nodes_roi_()
{
  auto nodemap = m_pDevice->GetNodeMap();

  // Width -------------------------------------------------
  if (is_passed_width) {
    Arena::SetNodeValue<int64_t>(nodemap, "Width", width_);
  } else {
    width_ = Arena::GetNodeValue<int64_t>(nodemap, "Width");
  }

  // Height ------------------------------------------------
  if (is_passed_height) {
    Arena::SetNodeValue<int64_t>(nodemap, "Height", height_);
  } else {
    height_ = Arena::GetNodeValue<int64_t>(nodemap, "Height");
  }

  // TODO only if it was passed by ros arg
  log_info(std::string("\tROI set to ") + std::to_string(width_) + "X" +
           std::to_string(height_));
}

void ArenaCameraNode::set_nodes_gain_()
{
  auto nodemap = m_pDevice->GetNodeMap();
  if (is_passed_gain_auto_) {
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "GainAuto", gain_auto_.c_str());
    log_info(std::string("\tGainAuto set to ") + gain_auto_);
  } else if (is_passed_gain_) {
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "GainAuto", "Off");
    Arena::SetNodeValue<double>(nodemap, "Gain", gain_);
    log_info(std::string("\tGain set to ") + std::to_string(gain_));
  }
}

void ArenaCameraNode::set_nodes_pixelformat_()
{
  auto nodemap = m_pDevice->GetNodeMap();
  // TODO ---------------------------------------------------------------------
  // PIXEL FORMAT HANDLEING

  if (is_passed_pixelformat_ros_) {
    pixelformat_pfnc_ = K_ROS2_PIXELFORMAT_TO_PFNC[pixelformat_ros_];
    if (pixelformat_pfnc_.empty()) {
      throw std::invalid_argument("pixelformat is not supported!");
    }

    try {
      Arena::SetNodeValue<GenICam::gcstring>(nodemap, "PixelFormat",
                                             pixelformat_pfnc_.c_str());
      log_info(std::string("\tPixelFormat set to ") + pixelformat_pfnc_);

    } catch (GenICam::GenericException& e) {
      // TODO
      // an rcl expectation might be expected
      auto x = std::string("pixelformat is not supported by this camera");
      x.append(e.what());
      throw std::invalid_argument(x);
    }
  } else {
    pixelformat_pfnc_ =
        Arena::GetNodeValue<GenICam::gcstring>(nodemap, "PixelFormat");
    pixelformat_ros_ = K_PFNC_TO_ROS2_PIXELFORMAT[pixelformat_pfnc_];

    if (pixelformat_ros_.empty()) {
      log_warn(
          "the device current pixelfromat value is not supported by ROS2. "
          "please use --ros-args -p pixelformat:=\"<supported pixelformat>\".");
      // TODO
      // print list of supported pixelformats
    }
  }
}

void ArenaCameraNode::set_nodes_exposure_()
{
  auto nodemap = m_pDevice->GetNodeMap();
  if (is_passed_exposure_auto_) {
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "ExposureAuto", exposure_auto_.c_str());
    log_info(std::string("\tExposureAuto set to ") + exposure_auto_);
  } else if (is_passed_exposure_time_) {
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "ExposureAuto", "Off");
    Arena::SetNodeValue<double>(nodemap, "ExposureTime", exposure_time_);
    log_info(std::string("\tExposureTime set to ") + std::to_string(exposure_time_));
  }
}

void ArenaCameraNode::set_nodes_trigger_mode_()
{
  auto nodemap = m_pDevice->GetNodeMap();

  if (trigger_mode_activated_) {
    if (exposure_time_ < 0) {
      log_warn(
          "\tavoid long waits wating for triggered images by providing proper "
          "exposure_time.");
    }

    // 1) Make sure TriggerMode is off while we set up the trigger
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerMode", "Off");

    // 2) Select the line for hardware triggering
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "LineSelector", "Line2");
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "LineMode", "Input");
    // If your camera needs it, also set "LineFormat" = "TTL" or "OptoCoupled"

    // 3) Configure trigger type
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerSelector", "FrameStart");
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerSource", "Line2");
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerActivation", "RisingEdge");

    // 4) Finally, enable trigger mode
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerMode", "On");

    auto msg =
        std::string(
            "\ttrigger_mode is activated. To trigger an image run `ros2 run ") +
        this->get_name() + " trigger_image`";
    log_warn(msg);
  }
  // unset device from being in trigger mode if user did not pass trigger
  // mode parameter because the trigger nodes are not rest when loading
  // the user default profile
  else {
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerMode", "Off");
  }
}

// just for debugging
void ArenaCameraNode::set_nodes_test_pattern_image_()
{
  auto nodemap = m_pDevice->GetNodeMap();
  Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TestPattern", "Pattern3");
}