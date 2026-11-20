#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/registration.h>

#include <thread>
#include <string>
#include <vector>
#include <atomic>
#include <cmath> // For M_PI in sine wave generation

// Parameter descriptors (restart-applied: this node reads params once).
static rcl_interfaces::msg::ParameterDescriptor desc(const std::string & text) {
  rcl_interfaces::msg::ParameterDescriptor d;
  d.description = text;
  d.additional_constraints = "restart";
  return d;
}
static rcl_interfaces::msg::ParameterDescriptor desc_f(const std::string & text, double lo, double hi) {
  auto d = desc(text);
  rcl_interfaces::msg::FloatingPointRange r;
  r.from_value = lo; r.to_value = hi; r.step = 0.0;
  d.floating_point_range.push_back(r);
  return d;
}
static rcl_interfaces::msg::ParameterDescriptor desc_i(const std::string & text, int lo, int hi) {
  auto d = desc(text);
  rcl_interfaces::msg::IntegerRange r;
  r.from_value = lo; r.to_value = hi; r.step = 1;
  d.integer_range.push_back(r);
  return d;
}

class KinectNode : public rclcpp::Node {
public:
  KinectNode()
  : rclcpp::Node("kinect_node"), running_(true)
  {
    // Parameters
    // Descriptors make these self-describing (ros2 param describe). All are read once
    // at startup, hence "restart".
    this->declare_parameter<std::string>("serial", "",
        desc("Kinect serial (empty = first device)"));
    this->declare_parameter<std::string>("pipeline", "cl",
        desc("Depth pipeline: cl (Apple GPU) or cpu"));
    this->declare_parameter<bool>("publish_ir", false,
        desc("Publish the infrared stream"));
    this->declare_parameter<double>("rgb_scale", 0.5,   // 0.5 = 960x540, 0.33 = 640x360, 1.0 = full res
        desc_f("Color resolution scale (0.5 = 960x540) — the big CPU dial", 0.25, 1.0));
    this->declare_parameter<double>("target_fps", 15.0,  // rate limit: 15fps halves bandwidth vs 30fps
        desc_f("Camera frame rate cap (fps)", 5.0, 30.0));
    this->declare_parameter<int>("jpeg_quality", 80,     // JPEG quality for compressed RGB (1-100)
        desc_i("JPEG quality of the color stream", 30, 100));

    serial_   = this->get_parameter("serial").as_string();
    pipeline_ = this->get_parameter("pipeline").as_string();
    publish_ir_ = this->get_parameter("publish_ir").as_bool();
    rgb_scale_ = this->get_parameter("rgb_scale").as_double();
    target_fps_ = this->get_parameter("target_fps").as_double();
    jpeg_quality_ = this->get_parameter("jpeg_quality").as_int();

    // Publishers
    rgb_pub_   = this->create_publisher<sensor_msgs::msg::Image>("kinect/rgb", 1);
    compressed_rgb_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("kinect/rgb/compressed", 1);
    registered_rgb_pub_ = this->create_publisher<sensor_msgs::msg::Image>("kinect/rgb/registered", 1);
    depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>("kinect/depth", 1);
    // Depth re-projected onto the COLOR image (same size as the published RGB,
    // pixel-aligned): lets RGB detections (pose keypoints) read their distance.
    color_depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>("kinect/depth/color_aligned", 1);
    camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/rgb/camera_info", 1);
    depth_camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/depth/camera_info", 1);
    if (publish_ir_) {
      ir_pub_ = this->create_publisher<sensor_msgs::msg::Image>("kinect/ir", 1);
    }

    RCLCPP_INFO(this->get_logger(), "Publishing: raw + JPEG compressed RGB @ %.0f fps, JPEG quality=%d",
               target_fps_, jpeg_quality_);

    // Select pipeline - OpenCL is best for macOS Apple Silicon
    if (pipeline_ == "cpu") {
      pipeline_impl_ = std::unique_ptr<libfreenect2::PacketPipeline>(new libfreenect2::CpuPacketPipeline());
    } else {
      pipeline_impl_ = std::unique_ptr<libfreenect2::PacketPipeline>(new libfreenect2::OpenCLPacketPipeline());
    }

    // Enumerate devices
    int ndev = freenect2_.enumerateDevices();
    if (ndev == 0) {
      RCLCPP_FATAL(this->get_logger(), "No Kinect v2 device found");
      rclcpp::shutdown();
      return;
    }

    if (serial_.empty()) {
      serial_ = freenect2_.getDeviceSerialNumber(0);
    }

    dev_ = freenect2_.openDevice(serial_, pipeline_impl_.get());
    if (!dev_) {
      RCLCPP_FATAL(this->get_logger(), "Failed to open device serial=%s", serial_.c_str());
      rclcpp::shutdown();
      return;
    }

    // Listener setup
    listener_ = std::make_shared<libfreenect2::SyncMultiFrameListener>(
      libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);

    dev_->setColorFrameListener(listener_.get());
    dev_->setIrAndDepthFrameListener(listener_.get());

    if (!dev_->start()) {
      RCLCPP_FATAL(this->get_logger(), "Failed to start streams");
      rclcpp::shutdown();
      return;
    }

    color_params_ = dev_->getColorCameraParams();
    ir_params_ = dev_->getIrCameraParams();

    // Registration maps color pixels to depth coordinates (and vice versa)
    registration_ = std::make_unique<libfreenect2::Registration>(
      dev_->getIrCameraParams(), dev_->getColorCameraParams());

    RCLCPP_INFO(this->get_logger(), "Kinect v2 started with pipeline=%s", pipeline_.c_str());

    // Processing loop (audio is now handled by separate Python node)
    loop_thread_ = std::thread([this]() { this->processLoop(); });
  }

  ~KinectNode() override {
    running_ = false;
    if (loop_thread_.joinable()) loop_thread_.join();
    if (dev_) {
      dev_->stop();
      dev_->close();
    }
  }

private:
  void processLoop() {
    using clock = std::chrono::steady_clock;
    auto frame_interval = std::chrono::duration<double>(1.0 / target_fps_);
    auto last_publish = clock::now() - frame_interval; // publish first frame immediately

    // Diagnostic: received-vs-published rate every 5 s — separates "the sensor
    // delivers slowly" from "the publish path is slow" (fps hunt, 2026-09-28)
    int diag_received = 0, diag_published = 0;
    auto diag_last = clock::now();

    while (rclcpp::ok() && running_) {
      libfreenect2::FrameMap frames;
      if (!listener_->waitForNewFrame(frames, 1000)) {
        continue;
      }
      diag_received++;

      auto now_time = clock::now();
      auto diag_dt = std::chrono::duration<double>(now_time - diag_last).count();
      if (diag_dt >= 5.0) {
        RCLCPP_INFO(get_logger(), "[fps] received %.1f/s published %.1f/s",
                    diag_received / diag_dt, diag_published / diag_dt);
        diag_received = 0; diag_published = 0; diag_last = now_time;
      }

      // Rate limiting: skip frames that arrive faster than target_fps_
      if (now_time - last_publish < frame_interval) {
        listener_->release(frames);
        continue;
      }
      last_publish = now_time;
      diag_published++;

      auto color = frames[libfreenect2::Frame::Color];
      auto depth = frames[libfreenect2::Frame::Depth];
      auto ir    = frames[libfreenect2::Frame::Ir];
      auto stamp = now();

      publishColor(color, stamp);
      publishDepth(depth, stamp);
      publishRegisteredRgb(color, depth, stamp);
      publishColorAlignedDepth(color, depth, stamp);
      publishCameraInfo(stamp);
      publishDepthCameraInfo(stamp);
      if (publish_ir_) publishIr(ir, stamp);

      listener_->release(frames);
    }
  }

  void publishFullRes(libfreenect2::Frame* rgb) {
    sensor_msgs::msg::Image msg;
    msg.header.stamp = now();
    msg.header.frame_id = "kinect_rgb_optical_frame";
    msg.height = rgb->height;
    msg.width  = rgb->width;
    
    // CHANGE: Use bgra8 to match libfreenect2's 4-channel output
    msg.encoding = "bgra8"; 
    msg.step = msg.width * 4; // 4 bytes per pixel (B, G, R, X)
    
    msg.data.assign(rgb->data, rgb->data + (msg.step * msg.height));
    rgb_pub_->publish(msg);
}

  void publishColor(libfreenect2::Frame* rgb, const rclcpp::Time & stamp) {
    // Create a CV Mat from the raw buffer (BGRA format from libfreenect2)
    cv::Mat full_frame(rgb->height, rgb->width, CV_8UC4, rgb->data);
    
    // 1. Downsample based on rgb_scale parameter. INTER_LINEAR, not INTER_AREA:
    // at the fixed 0.5 scale they are visually equivalent, and AREA's cost on a
    // 1080p frame was the main reason the serial loop topped out at ~11 fps
    // whatever target_fps asked for (measured 2026-09-28).
    int new_width = static_cast<int>(rgb->width * rgb_scale_);
    int new_height = static_cast<int>(rgb->height * rgb_scale_);
    cv::Mat small_bgra;
    cv::resize(full_frame, small_bgra, cv::Size(new_width, new_height), 0, 0, cv::INTER_LINEAR);

    // 2. Convert BGRA to BGR (removes alpha, saves ~25% data)
    cv::Mat bgr_frame;
    cv::cvtColor(small_bgra, bgr_frame, cv::COLOR_BGRA2BGR);

    // 3. Publish JPEG compressed (primary path: ~1.5MB → ~50-100KB)
    if (compressed_rgb_pub_->get_subscription_count() > 0) {
      std::vector<uchar> jpeg_buf;
      std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
      cv::imencode(".jpg", bgr_frame, jpeg_buf, params);

      sensor_msgs::msg::CompressedImage cmsg;
      cmsg.header.stamp = stamp;
      cmsg.header.frame_id = "kinect_rgb_optical_frame";
      cmsg.format = "jpeg";
      cmsg.data.assign(jpeg_buf.begin(), jpeg_buf.end());
      compressed_rgb_pub_->publish(cmsg);
    }

    // 4. Publish raw only if something subscribes (lazy — avoids 1.5MB serialization)
    if (rgb_pub_->get_subscription_count() > 0) {
      auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", bgr_frame).toImageMsg();
      msg->header.stamp = stamp;
      msg->header.frame_id = "kinect_rgb_optical_frame";
      rgb_pub_->publish(*msg);
    }
  }


  void publishDepth(libfreenect2::Frame* depth, const rclcpp::Time & stamp) {
    // Publish depth at half the RGB rate (every other frame)
    if (++depth_frame_counter_ % 2 != 0) return;
    if (depth_pub_->get_subscription_count() == 0) return;

    // Convert float32 depth (mm) → uint16 (halves message size: 870KB → 435KB)
    cv::Mat depth_float(depth->height, depth->width, CV_32FC1, depth->data);
    cv::Mat depth_u16;
    depth_float.convertTo(depth_u16, CV_16UC1);  // mm float → mm uint16

    sensor_msgs::msg::Image msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "kinect_depth_optical_frame";
    msg.height = depth_u16.rows;
    msg.width  = depth_u16.cols;
    msg.encoding = "mono16";
    msg.step = msg.width * sizeof(uint16_t);
    msg.is_bigendian = false;

    msg.data.resize(msg.step * msg.height);
    std::memcpy(msg.data.data(), depth_u16.data, msg.data.size());

    depth_pub_->publish(msg);
  }

  void publishCameraInfo(const rclcpp::Time & stamp) {
    sensor_msgs::msg::CameraInfo info;
    info.header.stamp = stamp;
    info.header.frame_id = "kinect_rgb_optical_frame";

    // Calculate info dimensions based on rgb_scale
    info.width = static_cast<uint32_t>(1920 * rgb_scale_);
    info.height = static_cast<uint32_t>(1080 * rgb_scale_);
    info.distortion_model = "plumb_bob";
    info.d = {0.0, 0.0, 0.0, 0.0, 0.0};

    const double scale_x = rgb_scale_;
    const double scale_y = rgb_scale_;
    const double fx = static_cast<double>(color_params_.fx) * scale_x;
    const double fy = static_cast<double>(color_params_.fy) * scale_y;
    const double cx = static_cast<double>(color_params_.cx) * scale_x;
    const double cy = static_cast<double>(color_params_.cy) * scale_y;

    info.k = {
      fx, 0.0, cx,
      0.0, fy, cy,
      0.0, 0.0, 1.0
    };
    info.r = {
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0
    };
    info.p = {
      fx, 0.0, cx, 0.0,
      0.0, fy, cy, 0.0,
      0.0, 0.0, 1.0, 0.0
    };

    camera_info_pub_->publish(info);
  }

  void publishDepthCameraInfo(const rclcpp::Time & stamp) {
    if (depth_camera_info_pub_->get_subscription_count() == 0) return;

    sensor_msgs::msg::CameraInfo info;
    info.header.stamp = stamp;
    info.header.frame_id = "kinect_depth_optical_frame";

    // Kinect v2 native depth resolution
    info.width = 512;
    info.height = 424;
    info.distortion_model = "plumb_bob";
    info.d = {
      static_cast<double>(ir_params_.k1),
      static_cast<double>(ir_params_.k2),
      static_cast<double>(ir_params_.p1),
      static_cast<double>(ir_params_.p2),
      static_cast<double>(ir_params_.k3)
    };

    const double fx = static_cast<double>(ir_params_.fx);
    const double fy = static_cast<double>(ir_params_.fy);
    const double cx = static_cast<double>(ir_params_.cx);
    const double cy = static_cast<double>(ir_params_.cy);

    info.k = {
      fx, 0.0, cx,
      0.0, fy, cy,
      0.0, 0.0, 1.0
    };
    info.r = {
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0
    };
    info.p = {
      fx, 0.0, cx, 0.0,
      0.0, fy, cy, 0.0,
      0.0, 0.0, 1.0, 0.0
    };

    depth_camera_info_pub_->publish(info);
  }

  void publishRegisteredRgb(libfreenect2::Frame* color, libfreenect2::Frame* depth,
                            const rclcpp::Time & stamp) {
    if (registered_rgb_pub_->get_subscription_count() == 0) return;

    // Use libfreenect2 Registration to map color→depth coordinates
    libfreenect2::Frame undistorted(512, 424, 4);
    libfreenect2::Frame registered(512, 424, 4);
    registration_->apply(color, depth, &undistorted, &registered);

    // registered is 512×424 BGRA, pixel-aligned to the depth frame
    cv::Mat reg_bgra(registered.height, registered.width, CV_8UC4, registered.data);
    cv::Mat reg_bgr;
    cv::cvtColor(reg_bgra, reg_bgr, cv::COLOR_BGRA2BGR);

    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", reg_bgr).toImageMsg();
    msg->header.stamp = stamp;
    msg->header.frame_id = "kinect_depth_optical_frame";
    registered_rgb_pub_->publish(*msg);
  }

  void publishColorAlignedDepth(libfreenect2::Frame* color, libfreenect2::Frame* depth,
                                const rclcpp::Time & stamp) {
    if (color_depth_pub_->get_subscription_count() == 0) return;
    if (++color_depth_counter_ % 2 != 0) return;   // half rate, like /kinect/depth

    // bigdepth: 1920x1082 float mm in color-camera pixels (rows 0 and 1081 are
    // padding). Unmapped pixels are +inf -> 0 (= no reading).
    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);
    libfreenect2::Frame bigdepth(1920, 1082, 4);
    registration_->apply(color, depth, &undistorted, &registered, true, &bigdepth);
    cv::Mat big(1082, 1920, CV_32FC1, bigdepth.data);
    cv::Mat valid = big.rowRange(1, 1081).clone();
    cv::patchNaNs(valid, 0.0);
    cv::threshold(valid, valid, 65000.0, 0.0, cv::THRESH_TOZERO_INV);   // inf -> 0
    cv::Mat small;
    cv::resize(valid, small, cv::Size(static_cast<int>(1920 * rgb_scale_),
                                      static_cast<int>(1080 * rgb_scale_)),
               0, 0, cv::INTER_NEAREST);   // never blend depths across edges
    cv::Mat u16;
    small.convertTo(u16, CV_16UC1);

    sensor_msgs::msg::Image msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "kinect_rgb_optical_frame";
    msg.height = u16.rows;
    msg.width = u16.cols;
    msg.encoding = "mono16";
    msg.step = msg.width * sizeof(uint16_t);
    msg.is_bigendian = false;
    msg.data.resize(msg.step * msg.height);
    std::memcpy(msg.data.data(), u16.data, msg.data.size());
    color_depth_pub_->publish(msg);
  }

  void publishIr(libfreenect2::Frame* ir, const rclcpp::Time & stamp) {
    if (ir_pub_->get_subscription_count() == 0) return;

    // libfreenect2 IR is float32 — convert to uint16 for proper encoding
    cv::Mat ir_float(ir->height, ir->width, CV_32FC1, ir->data);
    cv::Mat ir_u16;
    ir_float.convertTo(ir_u16, CV_16UC1);  // float → uint16

    sensor_msgs::msg::Image msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "kinect_ir_optical_frame";
    msg.height = ir_u16.rows;
    msg.width  = ir_u16.cols;
    msg.encoding = "mono16";
    msg.step = msg.width * sizeof(uint16_t);
    msg.is_bigendian = false;

    msg.data.resize(msg.step * msg.height);
    std::memcpy(msg.data.data(), ir_u16.data, msg.data.size());

    ir_pub_->publish(msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_rgb_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr registered_rgb_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_depth_pub_;
  int color_depth_counter_ = 0;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ir_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_camera_info_pub_;

  libfreenect2::Freenect2 freenect2_;
  libfreenect2::Freenect2Device* dev_{nullptr};
  std::shared_ptr<libfreenect2::SyncMultiFrameListener> listener_;
  std::unique_ptr<libfreenect2::PacketPipeline> pipeline_impl_;
  libfreenect2::Freenect2Device::ColorCameraParams color_params_;
  libfreenect2::Freenect2Device::IrCameraParams ir_params_;
  std::unique_ptr<libfreenect2::Registration> registration_;

  std::string serial_;
  std::string pipeline_;
  double rgb_scale_{0.5};
  bool publish_ir_{false};
  double target_fps_{15.0};
  int jpeg_quality_{80};
  uint64_t depth_frame_counter_{0};

  std::thread loop_thread_;
  std::atomic<bool> running_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KinectNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
