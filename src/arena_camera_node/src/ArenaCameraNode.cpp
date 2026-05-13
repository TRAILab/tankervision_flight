#include <atomic>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <vpi/OpenCVInterop.hpp>
#include <vpi/Status.h>
#include <vpi/Stream.h>
#include <vpi/algo/Rescale.h>
#include "rmw/types.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/string.hpp>
#include "ArenaCameraNode.h"
#include "light_arena/deviceinfo_helper.h"
#include "rclcpp_adapter/pixelformat_translation.h"
#include "rclcpp_adapter/quilty_of_service_translation.cpp"

namespace
{
void check_vpi_status(VPIStatus status, const char* statement)
{
  if (status == VPI_SUCCESS) {
    return;
  }

  char buffer[VPI_MAX_STATUS_MESSAGE_LENGTH] = {};
  vpiGetLastStatusMessage(buffer, sizeof(buffer));
  std::ostringstream ss;
  ss << statement << " failed: " << vpiStatusGetName(status);
  if (buffer[0] != '\0') {
    ss << ": " << buffer;
  }
  throw std::runtime_error(ss.str());
}

#define CHECK_VPI(STMT) check_vpi_status((STMT), #STMT)

struct VpiStreamGuard {
  VPIStream stream{nullptr};
  ~VpiStreamGuard()
  {
    if (stream) {
      vpiStreamSync(stream);
      vpiStreamDestroy(stream);
    }
  }
};

struct VpiImageGuard {
  VPIImage image{nullptr};
  ~VpiImageGuard()
  {
    if (image) {
      vpiImageDestroy(image);
    }
  }
};

struct VpiImageLockGuard {
  VPIImage image{nullptr};
  bool locked{false};
  ~VpiImageLockGuard()
  {
    if (locked && image) {
      vpiImageUnlock(image);
    }
  }
};
}  // namespace

void ArenaCameraNode::parse_parameters_()
{
  std::string nextParameterToDeclare = "";
  try {
    nextParameterToDeclare = "serial";
    serial_ = this->declare_parameter<std::string>("serial", "");
    is_passed_serial_ = serial_ != "";

    nextParameterToDeclare = "pixelformat";
    pixelformat_ros_ = this->declare_parameter<std::string>("pixelformat", "");
    is_passed_pixelformat_ros_ = pixelformat_ros_ != "";

    nextParameterToDeclare = "width";
    width_ = static_cast<size_t>(this->declare_parameter<int>("width", 0));
    is_passed_width = width_ > 0;

    nextParameterToDeclare = "height";
    height_ = static_cast<size_t>(this->declare_parameter<int>("height", 0));
    is_passed_height = height_ > 0;

    nextParameterToDeclare = "gain";
    gain_ = this->declare_parameter<double>("gain", -1.0);
    is_passed_gain_ = gain_ >= 0.0;

    nextParameterToDeclare = "gain_auto";
    gain_auto_ = this->declare_parameter<std::string>("gain_auto", "");
    is_passed_gain_auto_ = gain_auto_ != "";

    nextParameterToDeclare = "exposure_time";
    exposure_time_ = this->declare_parameter<double>("exposure_time", -1.0);
    is_passed_exposure_time_ = exposure_time_ >= 0.0;

    nextParameterToDeclare = "exposure_auto";
    exposure_auto_ = this->declare_parameter<std::string>("exposure_auto", "");
    is_passed_exposure_auto_ = exposure_auto_ != "";

    nextParameterToDeclare = "hardware_trigger";
    hardware_trigger_ = this->declare_parameter<bool>("hardware_trigger", false);

    nextParameterToDeclare = "topic";
    topic_ = this->declare_parameter<std::string>(
        "topic", std::string("/") + this->get_name() + "/images");

    nextParameterToDeclare = "qos_history";
    pub_qos_history_ = this->declare_parameter<std::string>("qos_history", "");
    is_passed_pub_qos_history_ = pub_qos_history_ != "";

    nextParameterToDeclare = "qos_history_depth";
    pub_qos_history_depth_ = static_cast<size_t>(this->declare_parameter<int>("qos_history_depth", 0));
    is_passed_pub_qos_history_depth_ = pub_qos_history_depth_ > 0;

    nextParameterToDeclare = "qos_reliability";
    pub_qos_reliability_ = this->declare_parameter<std::string>("qos_reliability", "");
    is_passed_pub_qos_reliability_ = pub_qos_reliability_ != "";

    nextParameterToDeclare = "raw_save_root";
    raw_save_root_ = this->declare_parameter<std::string>("raw_save_root", "/mnt/storage");

  } catch (rclcpp::ParameterTypeException& e) {
    log_err(nextParameterToDeclare + " argument");
    throw;
  }
}

void ArenaCameraNode::declare_tunable_parameters_()
{
  auto declare_if_new = [this](auto name, auto default_val) {
    if (!this->has_parameter(name)) {
      this->declare_parameter(name, default_val);
    }
  };

  declare_if_new("exposure_auto",              exposure_auto_.empty() ? std::string("Continuous") : exposure_auto_);
  declare_if_new("exposure_time",              exposure_time_ >= 0.0 ? exposure_time_ : 20000.0);
  declare_if_new("gain_auto",                  gain_auto_.empty() ? std::string("Continuous") : gain_auto_);
  declare_if_new("gain",                       gain_ >= 0.0 ? gain_ : 0.0);
  declare_if_new("target_brightness",          (int64_t)70);
  declare_if_new("exposure_auto_lower_limit",  100.0);
  declare_if_new("exposure_auto_upper_limit",  30000.0);
  declare_if_new("exposure_auto_algorithm",    std::string("Mean"));
  declare_if_new("exposure_auto_damping",      89.8);
  declare_if_new("auto_exposure_aoi_enable",   false);
  declare_if_new("auto_exposure_aoi_width",    (int64_t)0);
  declare_if_new("auto_exposure_aoi_height",   (int64_t)0);
  declare_if_new("auto_exposure_aoi_offset_x", (int64_t)0);
  declare_if_new("auto_exposure_aoi_offset_y", (int64_t)0);
  declare_if_new("gamma",                      0.5);
  declare_if_new("acquisition_frame_rate_enable", false);
  declare_if_new("acquisition_frame_rate",     0.0);
}

void ArenaCameraNode::initialize_()
{
  using namespace std::chrono_literals;

  m_pSystem = std::shared_ptr<Arena::ISystem>(nullptr, [=](Arena::ISystem* pSystem) {
    if (pSystem) {
      Arena::CloseSystem(pSystem);
      log_info("System is destroyed");
    }
  });
  m_pSystem.reset(Arena::OpenSystem());

  m_pDevice = std::shared_ptr<Arena::IDevice>(nullptr, [=](Arena::IDevice* pDevice) {
    if (m_pSystem && pDevice) {
      m_pSystem->DestroyDevice(pDevice);
      log_info("Device is destroyed");
    }
  });

  m_wait_for_device_timer_callback_ = this->create_wall_timer(
      1s, std::bind(&ArenaCameraNode::wait_for_device_timer_callback_, this));

  rclcpp::SensorDataQoS pub_qos_;
  if (is_passed_pub_qos_history_) {
    if (is_supported_qos_histroy_policy(pub_qos_history_)) {
      pub_qos_.history(K_CMDLN_PARAMETER_TO_QOS_HISTORY_POLICY[pub_qos_history_]);
    } else {
      log_err(pub_qos_history_ + " is not supported for this node");
      throw std::runtime_error(pub_qos_history_ + " is not supported for this node");
    }
  }
  if (is_passed_pub_qos_history_depth_ &&
      K_CMDLN_PARAMETER_TO_QOS_HISTORY_POLICY[pub_qos_history_] ==
          RMW_QOS_POLICY_HISTORY_KEEP_LAST) {
    pub_qos_.keep_last(pub_qos_history_depth_);
  }
  if (is_passed_pub_qos_reliability_) {
    if (is_supported_qos_reliability_policy(pub_qos_reliability_)) {
      pub_qos_.reliability(
          K_CMDLN_PARAMETER_TO_QOS_RELIABILITY_POLICY[pub_qos_reliability_]);
    } else {
      log_err(pub_qos_reliability_ + " is not supported for this node");
      throw std::runtime_error(pub_qos_reliability_ + " is not supported for this node");
    }
  }

  m_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      this->get_parameter("topic").as_string(), pub_qos_);

  heartbeat_pub_ = this->create_publisher<std_msgs::msg::String>(
      std::string("/camera_heartbeat"), 10);

  camera_diag_pub_ = this->create_publisher<std_msgs::msg::String>(
      std::string("/camera/lucid_diagnostics"), 10);

  record_mode_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/camera/record_mode", rclcpp::QoS(10).reliable(),
      std::bind(
          &ArenaCameraNode::record_mode_callback_, this, std::placeholders::_1));

  {
    auto qos = rclcpp::QoS(1)
        .reliability(rclcpp::ReliabilityPolicy::Reliable)
        .durability(rclcpp::DurabilityPolicy::TransientLocal);
    session_path_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/tankervision/session_path", qos,
        std::bind(
            &ArenaCameraNode::session_path_callback_, this, std::placeholders::_1));
  }

  log_info(
      "Subscribed to /camera/record_mode. Send 'record' to save raw images, "
      "'standby' to stop saving raw images. Waiting for /tankervision/session_path.");

  declare_tunable_parameters_();
  apply_initial_tunable_parameters_();

  param_cb_handle_ = this->add_on_set_parameters_callback(
      std::bind(
          &ArenaCameraNode::on_set_parameters_, this, std::placeholders::_1));

  std::stringstream pub_qos_info;
  auto pub_qos_profile = pub_qos_.get_rmw_qos_profile();
  pub_qos_info << '\t' << "QoS history     = "
               << K_QOS_HISTORY_POLICY_TO_CMDLN_PARAMETER[pub_qos_profile.history] << '\n';
  pub_qos_info << "\t\t\t\t" << "QoS depth       = " << pub_qos_profile.depth << '\n';
  pub_qos_info << "\t\t\t\t" << "QoS reliability = "
               << K_QOS_RELIABILITY_POLICY_TO_CMDLN_PARAMETER[pub_qos_profile.reliability] << '\n';
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
  publish_camera_diagnostics_();
}

void ArenaCameraNode::wait_for_device_timer_callback_()
{
  if (!rclcpp::ok()) {
    log_err("Interrupted while waiting for arena camera. Exiting.");
    rclcpp::shutdown();
    return;
  }

  m_pSystem->UpdateDevices(100);
  auto device_infos = m_pSystem->GetDevices();
  if (!device_infos.size()) {
    log_info("No arena camera is connected. Waiting for device(s)...");
    return;
  }

  m_wait_for_device_timer_callback_->cancel();
  log_info(
      std::to_string(device_infos.size()) + " arena device(s) has been discovered.");
  run_();
}

void ArenaCameraNode::record_mode_callback_(const std_msgs::msg::String::SharedPtr msg)
{
  if (!msg) {
    log_warn("Received null /camera/record_mode message");
    return;
  }
  const std::string mode = msg->data;
  if (mode == "record" || mode == "Record" || mode == "RECORD") {
    record_raw_.store(true, std::memory_order_release);
    log_info("Camera raw recording enabled");
  } else if (mode == "standby" || mode == "Standby" || mode == "STANDBY") {
    record_raw_.store(false, std::memory_order_release);
    log_info("Camera raw recording disabled / standby");
  } else {
    log_warn(
        "Unknown /camera/record_mode value: '" + mode + "'. Expected 'record' or 'standby'.");
  }
}

void ArenaCameraNode::session_path_callback_(const std_msgs::msg::String::SharedPtr msg)
{
  if (!msg || msg->data.empty()) return;
  std::filesystem::path new_dir = std::filesystem::path(msg->data);
  std::error_code ec;
  std::filesystem::create_directories(new_dir, ec);
  if (ec) {
    log_warn("session_path_callback_: could not create directory: " + new_dir.string());
    return;
  }
  raw_save_dir_ = new_dir;
  log_info("Raw save directory updated to session path: " + new_dir.string());
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
      if (name == "exposure_auto" || name == "exposure_time" ||
          name == "gain_auto" || name == "gain" ||
          name == "target_brightness" || name == "gamma") {
        result.successful = false;
        result.reason = "ExposureAuto, GainAuto, TargetBrightness, and Gamma are hardcoded";
        return result;
      }
      if (name == "exposure_auto_lower_limit") {
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
      } else if (name == "acquisition_frame_rate_enable") {
        if (hardware_trigger_) {
          log_warn("Ignoring acquisition_frame_rate_enable while hardware_trigger is enabled");
        } else {
          Arena::SetNodeValue<bool>(nodemap, "AcquisitionFrameRateEnable", param.as_bool());
        }
      } else if (name == "acquisition_frame_rate") {
        if (hardware_trigger_) {
          log_warn("Ignoring acquisition_frame_rate while hardware_trigger is enabled");
        } else {
          Arena::SetNodeValue<double>(nodemap, "AcquisitionFrameRate", param.as_double());
        }
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
  publish_worker_thread_ = std::thread(&ArenaCameraNode::resize_and_publish_worker_, this);
  acquisition_thread_ = std::thread(&ArenaCameraNode::publish_images_, this);
}

builtin_interfaces::msg::Time ArenaCameraNode::timestamp_from_image_(
    Arena::IImage* pImage) const
{
  builtin_interfaces::msg::Time stamp;
  if (!pImage) { return stamp; }
  const uint64_t timestamp_ns = pImage->GetTimestampNs();
  stamp.sec    = static_cast<int32_t>(timestamp_ns / 1000000000ULL);
  stamp.nanosec = static_cast<uint32_t>(timestamp_ns % 1000000000ULL);
  return stamp;
}

std::string ArenaCameraNode::timestamp_filename_stem_(Arena::IImage* pImage) const
{
  if (!pImage) { return "null_timestamp"; }
  const uint64_t timestamp_ns = pImage->GetTimestampNs();
  const uint64_t sec  = timestamp_ns / 1000000000ULL;
  const uint64_t nsec = timestamp_ns % 1000000000ULL;
  std::ostringstream ss;
  ss << sec << "_" << std::setw(9) << std::setfill('0') << nsec;
  return ss.str();
}

void ArenaCameraNode::save_raw_image_(Arena::IImage* pImage)
{
  if (!pImage) {
    log_warn("save_raw_image_: pImage is null");
    return;
  }
  if (raw_save_dir_.empty()) {
    log_warn("save_raw_image_: session path not yet received, skipping");
    return;
  }

  const auto t0 = std::chrono::steady_clock::now();
  const uint64_t frame_id       = pImage->GetFrameId();
  const uint64_t width          = pImage->GetWidth();
  const uint64_t height         = pImage->GetHeight();
  const uint64_t bits_per_pixel = pImage->GetBitsPerPixel();
  const size_t bytes_per_pixel  = static_cast<size_t>((bits_per_pixel + 7ULL) / 8ULL);
  const size_t data_size =
      static_cast<size_t>(width) * static_cast<size_t>(height) * bytes_per_pixel;

  std::ostringstream filename;
  filename << timestamp_filename_stem_(pImage)
           << "_frame_" << frame_id
           << "_" << width << "x" << height
           << "_" << bits_per_pixel << "bpp.raw";

  const std::filesystem::path output_path = raw_save_dir_ / filename.str();
  const void* src = pImage->GetData();
  if (!src) {
    log_warn("save_raw_image_: image data pointer is null");
    return;
  }

  std::ofstream ofs(output_path, std::ios::binary);
  if (!ofs) {
    log_warn("Failed to open raw output file: " + output_path.string());
    return;
  }
  ofs.write(reinterpret_cast<const char*>(src), static_cast<std::streamsize>(data_size));
  ofs.close();

  const auto t1 = std::chrono::steady_clock::now();
  const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
  log_info(
      "Saved raw image: " + output_path.string() +
      " (" + std::to_string(data_size) + " bytes, " + std::to_string(ms) + " ms)");
}

void ArenaCameraNode::publish_images_()
{
  Arena::IImage* pImage = nullptr;
  while (rclcpp::ok() && running_.load(std::memory_order_acquire)) {
    try {
      pImage = m_pDevice->GetImage(1000);
      const auto image_stamp = timestamp_from_image_(pImage);
      const std::string frame_id = std::to_string(pImage->GetFrameId());

      if (record_raw_.load(std::memory_order_acquire)) {
        try {
          save_raw_image_(pImage);
        } catch (const std::exception& e) {
          log_warn(std::string("Failed to save raw image: ") + e.what());
        } catch (...) {
          log_warn("Failed to save raw image: unknown exception");
        }
      }

      const size_t width  = pImage->GetWidth();
      const size_t height = pImage->GetHeight();
      if (width != 5320 || height != 4600) {
        log_warn(
            "Unexpected image size: " + std::to_string(width) + "x" + std::to_string(height) +
            ", expected 5320x4600");
      }

      Arena::IImage* converted = nullptr;
      try {
        converted = Arena::ImageFactory::Convert(pImage, PfncFormat::BGRa8);
        const size_t conv_width  = converted->GetWidth();
        const size_t conv_height = converted->GetHeight();

        cv::Mat bgra(
            static_cast<int>(conv_height),
            static_cast<int>(conv_width),
            CV_8UC4,
            const_cast<uint8_t*>(converted->GetData()));

        PublishFrame frame;
        frame.stamp = image_stamp;
        frame.frame_id = frame_id;
        frame.width = conv_width;
        frame.height = conv_height;
        const size_t img_data_size = bgra.total() * bgra.elemSize();
        frame.bgra.resize(img_data_size);
        std::memcpy(frame.bgra.data(), bgra.data, img_data_size);

        Arena::ImageFactory::Destroy(converted);
        converted = nullptr;
        m_pDevice->RequeueBuffer(pImage);
        pImage = nullptr;
        enqueue_publish_frame_(std::move(frame));
      } catch (...) {
        if (converted) {
          Arena::ImageFactory::Destroy(converted);
          converted = nullptr;
        }
        throw;
      }
    } catch (const GenICam::GenericException& e) {
      if (pImage) {
        m_pDevice->RequeueBuffer(pImage);
        pImage = nullptr;
      }
      const std::string msg = e.what();
      if (hardware_trigger_ &&
          (msg.find("timeout") != std::string::npos ||
           msg.find("Timeout") != std::string::npos ||
           msg.find("Timed out") != std::string::npos ||
           msg.find("timed out") != std::string::npos)) {
        log_debug("Timed out waiting for hardware trigger; continuing to wait");
        continue;
      }
      log_warn(std::string("GenICam exception occurred while publishing an image\n") + msg);
      continue;
    } catch (const std::exception& e) {
      if (pImage) {
        m_pDevice->RequeueBuffer(pImage);
        pImage = nullptr;
      }
      log_warn(std::string("Exception occurred while publishing an image\n") + e.what());
      continue;
    } catch (...) {
      if (pImage) {
        m_pDevice->RequeueBuffer(pImage);
        pImage = nullptr;
      }
      log_warn("Unknown exception occurred while publishing an image");
      continue;
    }
  }
}

void ArenaCameraNode::enqueue_publish_frame_(PublishFrame frame)
{
  {
    std::lock_guard<std::mutex> lock(publish_queue_mutex_);
    while (publish_queue_.size() >= kMaxPublishQueueSize) {
      publish_queue_.pop_front();
    }
    publish_queue_.push_back(std::move(frame));
  }
  publish_queue_cv_.notify_one();
}

void ArenaCameraNode::resize_and_publish_worker_()
{
  while (true) {
    PublishFrame frame;
    {
      std::unique_lock<std::mutex> lock(publish_queue_mutex_);
      publish_queue_cv_.wait(lock, [this]() {
        return !publish_queue_.empty() || !running_.load(std::memory_order_acquire) || !rclcpp::ok();
      });
      if (publish_queue_.empty()) {
        if (!running_.load(std::memory_order_acquire) || !rclcpp::ok()) {
          break;
        }
        continue;
      }
      frame = std::move(publish_queue_.front());
      publish_queue_.pop_front();
    }

    if (frame.bgra.empty() || frame.width == 0 || frame.height == 0) {
      continue;
    }

    cv::Mat bgr_downscaled;
    try {
      bgr_downscaled = resize_with_vpi_vic_(frame);
    } catch (const std::exception& e) {
      log_warn(std::string("VPI VIC resize failed: ") + e.what());
      continue;
    }

    auto p_image_msg = std::make_unique<sensor_msgs::msg::Image>();
    p_image_msg->header.stamp    = frame.stamp;
    p_image_msg->header.frame_id = frame.frame_id;
    p_image_msg->height   = bgr_downscaled.rows;
    p_image_msg->width    = bgr_downscaled.cols;
    p_image_msg->encoding = sensor_msgs::image_encodings::BGR8;
    p_image_msg->is_bigendian = 0;
    p_image_msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(bgr_downscaled.step);
    const size_t img_data_size = bgr_downscaled.total() * bgr_downscaled.elemSize();
    p_image_msg->data.resize(img_data_size);
    std::memcpy(p_image_msg->data.data(), bgr_downscaled.data, img_data_size);

    m_pub_->publish(std::move(p_image_msg));
    std_msgs::msg::String hb;
    hb.data = "heartbeat";
    heartbeat_pub_->publish(hb);
  }
}

cv::Mat ArenaCameraNode::resize_with_vpi_vic_(const PublishFrame& frame)
{
  const int input_width = static_cast<int>(frame.width);
  const int input_height = static_cast<int>(frame.height);
  const int output_width = input_width / 8;
  const int output_height = input_height / 8;

  cv::Mat input_bgra(input_height, input_width, CV_8UC4, const_cast<uint8_t*>(frame.bgra.data()));
  cv::Mat output_bgra(output_height, output_width, CV_8UC4);
  cv::Mat output_bgr(output_height, output_width, CV_8UC3);

  VpiStreamGuard stream;
  CHECK_VPI(vpiStreamCreate(VPI_BACKEND_VIC, &stream.stream));

  VpiImageGuard input_bgra_vpi;
  CHECK_VPI(vpiImageCreateWrapperOpenCVMat(
      input_bgra, VPI_IMAGE_FORMAT_BGRA8, VPI_BACKEND_VIC, &input_bgra_vpi.image));

  VpiImageGuard output_bgra_vpi;
  CHECK_VPI(vpiImageCreate(
      output_width,
      output_height,
      VPI_IMAGE_FORMAT_BGRA8,
      VPI_BACKEND_VIC | VPI_BACKEND_CPU,
      &output_bgra_vpi.image));

  CHECK_VPI(vpiSubmitRescale(
      stream.stream,
      VPI_BACKEND_VIC,
      input_bgra_vpi.image,
      output_bgra_vpi.image,
      VPI_INTERP_LINEAR,
      VPI_BORDER_CLAMP,
      0));

  CHECK_VPI(vpiStreamSync(stream.stream));
  VPIImageData output_data = {};
  CHECK_VPI(vpiImageLockData(
      output_bgra_vpi.image, VPI_LOCK_READ, VPI_IMAGE_BUFFER_HOST_PITCH_LINEAR, &output_data));
  VpiImageLockGuard output_lock;
  output_lock.image = output_bgra_vpi.image;
  output_lock.locked = true;

  cv::Mat output_view;
  CHECK_VPI(vpiImageDataExportOpenCVMat(output_data, &output_view));
  output_view.copyTo(output_bgra);
  cv::cvtColor(output_bgra, output_bgr, cv::COLOR_BGRA2BGR);
  return output_bgr;
}

void ArenaCameraNode::publish_camera_diagnostics_()
{
  if (!m_pDevice) { return; }
  auto nodemap = m_pDevice->GetNodeMap();
  std::ostringstream ss;
  ss << "{";

  auto get_double = [&](const char* key, const char* node) {
    try { ss << "\"" << key << "\":" << Arena::GetNodeValue<double>(nodemap, node); }
    catch (...) { ss << "\"" << key << "\":null"; }
  };
  auto get_string = [&](const char* key, const char* node) {
    try {
      ss << "\"" << key << "\":\""
         << std::string(Arena::GetNodeValue<GenICam::gcstring>(nodemap, node)) << "\"";
    } catch (...) { ss << "\"" << key << "\":null"; }
  };
  auto get_bool = [&](const char* key, const char* node) {
    try {
      ss << "\"" << key << "\":"
         << (Arena::GetNodeValue<bool>(nodemap, node) ? "true" : "false");
    } catch (...) { ss << "\"" << key << "\":null"; }
  };
  auto get_int = [&](const char* key, const char* node) {
    try { ss << "\"" << key << "\":" << Arena::GetNodeValue<int64_t>(nodemap, node); }
    catch (...) { ss << "\"" << key << "\":null"; }
  };

  get_string("exposure_auto",             "ExposureAuto");        ss << ",";
  get_double("exposure_time",             "ExposureTime");        ss << ",";
  get_string("gain_auto",                 "GainAuto");            ss << ",";
  get_double("gain",                      "Gain");                ss << ",";
  get_int   ("target_brightness",         "TargetBrightness");    ss << ",";
  get_double("exposure_auto_lower_limit", "ExposureAutoLowerLimit"); ss << ",";
  get_double("exposure_auto_upper_limit", "ExposureAutoUpperLimit"); ss << ",";
  get_string("exposure_auto_algorithm",   "ExposureAutoAlgorithm"); ss << ",";
  get_double("exposure_auto_damping",     "ExposureAutoDamping"); ss << ",";
  get_bool  ("auto_exposure_aoi_enable",  "AutoExposureAOIEnable"); ss << ",";
  get_int   ("auto_exposure_aoi_width",   "AutoExposureAOIWidth"); ss << ",";
  get_int   ("auto_exposure_aoi_height",  "AutoExposureAOIHeight"); ss << ",";
  get_int   ("auto_exposure_aoi_offset_x","AutoExposureAOIOffsetX"); ss << ",";
  get_int   ("auto_exposure_aoi_offset_y","AutoExposureAOIOffsetY"); ss << ",";
  get_double("gamma",                     "Gamma");               ss << ",";
  get_bool  ("acquisition_frame_rate_enable","AcquisitionFrameRateEnable"); ss << ",";
  get_double("acquisition_frame_rate",    "AcquisitionFrameRate"); ss << ",";
  get_bool  ("hardware_trigger",          "TriggerMode");         ss << ",";
  get_double("calculated_mean",           "CalculatedMean");      ss << ",";
  get_double("calculated_median",         "CalculatedMedian");

  ss << "}";
  std_msgs::msg::String msg;
  msg.data = ss.str();
  camera_diag_pub_->publish(msg);
}

Arena::IDevice* ArenaCameraNode::create_device_ros_()
{
  m_pSystem->UpdateDevices(100);
  auto device_infos = m_pSystem->GetDevices();
  if (!device_infos.size()) {
    throw std::runtime_error(
        "camera(s) were disconnected after they were discovered");
  }
  auto index = 0;
  if (is_passed_serial_) {
    index = DeviceInfoHelper::get_index_of_serial(device_infos, serial_);
  }
  auto pDevice = m_pSystem->CreateDevice(device_infos.at(index));
  log_info(
      std::string("device created ") + DeviceInfoHelper::info(device_infos.at(index)));
  return pDevice;
}

void ArenaCameraNode::set_nodes_()
{
  auto nodemap = m_pDevice->GetNodeMap();

  Arena::SetNodeValue<int64_t>(nodemap, "PacketResendWindowFrameCount", 8);
  log_info("\tPacket resend window: 8");

  Arena::SetNodeValue<int64_t>(nodemap, "DeviceLinkThroughputReserve", 10);
  log_info("\tEthernet link reserve: 10");

  set_nodes_load_default_profile_();
  set_nodes_roi_();
  set_nodes_auto_exposure_gain_();
  set_nodes_pixelformat_();

  if (!hardware_trigger_) {
    try {
      const bool frame_rate_enable =
          this->get_parameter("acquisition_frame_rate_enable").as_bool();
      const double frame_rate =
          this->get_parameter("acquisition_frame_rate").as_double();
      Arena::SetNodeValue<bool>(nodemap, "AcquisitionFrameRateEnable", frame_rate_enable);
      if (frame_rate_enable) {
        Arena::SetNodeValue<double>(nodemap, "AcquisitionFrameRate", frame_rate);
        log_info("\tAcquisitionFrameRate set to " + std::to_string(frame_rate) + " Hz");
      } else {
        log_info("\tAcquisitionFrameRate disabled");
      }
    } catch (const GenICam::GenericException& e) {
      log_warn(std::string("Failed to configure acquisition frame rate: ") + e.what());
    } catch (const std::exception& e) {
      log_warn(std::string("Failed to configure acquisition frame rate: ") + e.what());
    }
  } else {
    try {
      Arena::SetNodeValue<bool>(nodemap, "AcquisitionFrameRateEnable", false);
    } catch (...) {}
    log_info("\tAcquisitionFrameRate ignored because hardware_trigger is enabled");
  }

  set_nodes_trigger_mode_();

  Arena::SetNodeValue<bool>(
      m_pDevice->GetTLStreamNodeMap(), "StreamAutoNegotiatePacketSize", true);
  Arena::SetNodeValue<bool>(
      m_pDevice->GetTLStreamNodeMap(), "StreamPacketResendEnable", true);

  Arena::SetNodeValue(nodemap, "PtpEnable", true);
  Arena::SetNodeValue(nodemap, "PtpSlaveOnly", true);

  {
    const auto ptp_timeout       = std::chrono::seconds(60);
    const auto ptp_poll_interval = std::chrono::seconds(5);
    const auto ptp_deadline      = std::chrono::steady_clock::now() + ptp_timeout;

    GenICam::gcstring currPtpStatus =
        Arena::GetNodeValue<GenICam::gcstring>(nodemap, "PtpStatus");

    while (currPtpStatus != "Slave") {
      if (std::chrono::steady_clock::now() >= ptp_deadline) {
        log_warn(
            "PTP slave sync timed out after 60 seconds (last status: " +
            std::string(currPtpStatus) + "). Continuing without PTP sync.");
        break;
      }
      log_info("PTP not yet in slave mode: " + std::string(currPtpStatus));
      std::this_thread::sleep_for(ptp_poll_interval);
      currPtpStatus = Arena::GetNodeValue<GenICam::gcstring>(nodemap, "PtpStatus");
    }
    if (currPtpStatus == "Slave") {
      log_info("PTP status: " + std::string(currPtpStatus));
    }
  }
}

// ── set_nodes_auto_exposure_gain_ ─────────────────────────────────────────────
// Reads all exposure/gain parameters from ROS params (set from tankervision.yaml
// via flight.launch.py). TargetBrightness and Gamma are wrapped in try/catch
// because some camera firmware versions do not expose these nodes.
void ArenaCameraNode::set_nodes_auto_exposure_gain_()
{
  auto nodemap = m_pDevice->GetNodeMap();

  const std::string exposure_auto = this->get_parameter("exposure_auto").as_string();
  const std::string gain_auto     = this->get_parameter("gain_auto").as_string();
  const int64_t target_brightness = this->get_parameter("target_brightness").as_int();
  const double gamma              = this->get_parameter("gamma").as_double();
  const double exp_lower          = this->get_parameter("exposure_auto_lower_limit").as_double();
  const double exp_upper          = this->get_parameter("exposure_auto_upper_limit").as_double();
  const std::string exp_algorithm = this->get_parameter("exposure_auto_algorithm").as_string();
  const double exp_damping        = this->get_parameter("exposure_auto_damping").as_double();

  // Set ExposureAuto and GainAuto first — required before
  // TargetBrightness and Gamma become accessible
  Arena::SetNodeValue<GenICam::gcstring>(nodemap, "ExposureAuto", exposure_auto.c_str());
  log_info("\tExposureAuto set to " + exposure_auto);

  Arena::SetNodeValue<GenICam::gcstring>(nodemap, "GainAuto", gain_auto.c_str());
  log_info("\tGainAuto set to " + gain_auto);

  try {
    Arena::SetNodeValue<int64_t>(nodemap, "TargetBrightness", target_brightness);
    log_info("\tTargetBrightness set to " + std::to_string(target_brightness));
  } catch (const GenICam::GenericException& e) {
    log_warn(std::string("\tTargetBrightness not accessible: ") + e.what());
  }

  try {
    Arena::SetNodeValue<double>(nodemap, "Gamma", gamma);
    log_info("\tGamma set to " + std::to_string(gamma));
  } catch (const GenICam::GenericException& e) {
    log_warn(std::string("\tGamma not accessible: ") + e.what());
  }

  try {
    Arena::SetNodeValue<double>(nodemap, "ExposureAutoLowerLimit", exp_lower);
    Arena::SetNodeValue<double>(nodemap, "ExposureAutoUpperLimit", exp_upper);
    Arena::SetNodeValue<GenICam::gcstring>(
        nodemap, "ExposureAutoAlgorithm", exp_algorithm.c_str());
    Arena::SetNodeValue<double>(nodemap, "ExposureAutoDamping", exp_damping);
    log_info(
        "\tExposureAuto limits: " + std::to_string(exp_lower) +
        " - " + std::to_string(exp_upper));
    log_info("\tExposureAutoAlgorithm: " + exp_algorithm);
    log_info("\tExposureAutoDamping: " + std::to_string(exp_damping));
  } catch (const GenICam::GenericException& e) {
    log_warn(std::string("\tExposureAuto tuning params not accessible: ") + e.what());
  }
}

void ArenaCameraNode::set_nodes_load_default_profile_()
{
  auto nodemap = m_pDevice->GetNodeMap();
  Arena::SetNodeValue<GenICam::gcstring>(nodemap, "UserSetSelector", "Default");
  Arena::ExecuteNode(nodemap, "UserSetLoad");
  log_info("\tdefault profile is loaded");
}

void ArenaCameraNode::set_nodes_roi_()
{
  auto nodemap = m_pDevice->GetNodeMap();
  if (is_passed_width) {
    Arena::SetNodeValue<int64_t>(nodemap, "Width", static_cast<int64_t>(width_));
  } else {
    width_ = static_cast<size_t>(Arena::GetNodeValue<int64_t>(nodemap, "Width"));
  }
  if (is_passed_height) {
    Arena::SetNodeValue<int64_t>(nodemap, "Height", static_cast<int64_t>(height_));
  } else {
    height_ = static_cast<size_t>(Arena::GetNodeValue<int64_t>(nodemap, "Height"));
  }
  log_info(
      std::string("\tROI set to ") + std::to_string(width_) + "X" + std::to_string(height_));
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
  if (is_passed_pixelformat_ros_) {
    pixelformat_pfnc_ = K_ROS2_PIXELFORMAT_TO_PFNC[pixelformat_ros_];
    if (pixelformat_pfnc_.empty()) {
      throw std::invalid_argument("pixelformat is not supported!");
    }
    try {
      Arena::SetNodeValue<GenICam::gcstring>(
          nodemap, "PixelFormat", pixelformat_pfnc_.c_str());
      log_info(std::string("\tPixelFormat set to ") + pixelformat_pfnc_);
    } catch (GenICam::GenericException& e) {
      auto x = std::string("pixelformat is not supported by this camera");
      x.append(e.what());
      throw std::invalid_argument(x);
    }
  } else {
    pixelformat_pfnc_ = Arena::GetNodeValue<GenICam::gcstring>(nodemap, "PixelFormat");
    pixelformat_ros_  = K_PFNC_TO_ROS2_PIXELFORMAT[pixelformat_pfnc_];
    if (pixelformat_ros_.empty()) {
      log_warn(
          "the device current pixelfromat value is not supported by ROS2. "
          "please use --ros-args -p pixelformat:=\"<supported pixelformat>\".");
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
  if (hardware_trigger_) {
    if (exposure_time_ < 0.0) {
      log_warn(
          "\tavoid long waits waiting for triggered images by providing proper "
          "exposure_time.");
    }
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerMode", "Off");
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "LineSelector", "Line2");
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "LineMode", "Input");
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerSelector", "FrameStart");
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerSource", "Line2");
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerActivation", "FallingEdge");
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerMode", "On");
    log_warn(
        "\thardware_trigger is enabled: camera waits for Line2 falling-edge "
        "FrameStart triggers");
  } else {
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerMode", "Off");
    log_info("\thardware_trigger is disabled: camera streams continuously");
  }
}

void ArenaCameraNode::set_nodes_test_pattern_image_()
{
  auto nodemap = m_pDevice->GetNodeMap();
  Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TestPattern", "Pattern3");
}
