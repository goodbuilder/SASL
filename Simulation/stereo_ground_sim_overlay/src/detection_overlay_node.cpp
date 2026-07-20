#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

namespace
{
struct Detection
{
  bool valid{false};
  float x1{0.0F};
  float y1{0.0F};
  float x2{0.0F};
  float y2{0.0F};
  float confidence{0.0F};
  float cx{0.0F};
  float cy{0.0F};
};

}  // namespace

class DetectionOverlayNode : public rclcpp::Node
{
public:
  DetectionOverlayNode()
  : Node("detection_overlay_node")
  {
    overlay_publish_hz_ = declare_parameter<double>("overlay_publish_hz", 20.0);
    if (overlay_publish_hz_ < 0.0) {
      overlay_publish_hz_ = 0.0;
    }
    if (overlay_publish_hz_ > 0.0) {
      min_publish_period_ = std::chrono::duration<double>(1.0 / overlay_publish_hz_);
    }

    auto camera_qos = rclcpp::SensorDataQoS().keep_last(1);
    left_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/left_camera/image", camera_qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        updateCachedOverlay("left", msg);
      });
    right_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/right_camera/image", camera_qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        updateCachedOverlay("right", msg);
      });
    detections_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/stereo/detections", 10,
      [this](std_msgs::msg::Float32MultiArray::ConstSharedPtr msg) {
        if (msg->data.size() < 16) {
          return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        left_detection_ = valuesToDetection(msg->data.data());
        right_detection_ = valuesToDetection(msg->data.data() + 8);
      });

    left_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "/left_camera/detection_image", rclcpp::SensorDataQoS().keep_last(1));
    right_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "/right_camera/detection_image", rclcpp::SensorDataQoS().keep_last(1));

    if (overlay_publish_hz_ > 0.0) {
      const auto alternating_period = std::chrono::duration<double>(
        min_publish_period_.count() * 0.5);
      timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(alternating_period),
        [this]() { publishNextOverlay(); });
    }

    RCLCPP_INFO(
      get_logger(), "C++ detection overlay node started in alternating timer mode (overlay_publish_hz=%.1f)",
      overlay_publish_hz_);
  }

private:
  static Detection valuesToDetection(const float * values)
  {
    Detection detection;
    if (values[0] < 0.5F) {
      return detection;
    }
    if (!std::isfinite(values[1]) || !std::isfinite(values[2]) ||
        !std::isfinite(values[3]) || !std::isfinite(values[4])) {
      return detection;
    }
    detection.valid = true;
    detection.x1 = values[1];
    detection.y1 = values[2];
    detection.x2 = values[3];
    detection.y2 = values[4];
    detection.confidence = std::isfinite(values[5]) ? values[5] : 0.0F;
    detection.cx = std::isfinite(values[6]) ? values[6] : 0.5F * (values[1] + values[3]);
    detection.cy = std::isfinite(values[7]) ? values[7] : 0.5F * (values[2] + values[4]);
    return detection;
  }

  void updateCachedOverlay(
    const char * side,
    const sensor_msgs::msg::Image::ConstSharedPtr & source)
  {
    if (!source) {
      return;
    }

    if (source->encoding != "rgb8") {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "%s overlay only supports rgb8 input; got '%s'", side, source->encoding.c_str());
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (std::string(side) == "left") {
      last_left_image_ = source;
    } else {
      last_right_image_ = source;
    }
  }

  void publishNextOverlay()
  {
    const bool publish_left = publish_left_next_;
    publish_left_next_ = !publish_left_next_;
    if (publish_left) {
      publishOverlay("left", left_pub_);
    } else {
      publishOverlay("right", right_pub_);
    }
  }

  void publishOverlay(
    const char * side,
    const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr & publisher)
  {
    sensor_msgs::msg::Image::ConstSharedPtr source;
    Detection detection;
    bool pair_has_finite_ray_inputs{false};
    {
      std::lock_guard<std::mutex> lock(mutex_);
      pair_has_finite_ray_inputs =
        left_detection_.valid && right_detection_.valid &&
        std::isfinite(left_detection_.cx) && std::isfinite(right_detection_.cx) &&
        std::isfinite(left_detection_.cy) && std::isfinite(right_detection_.cy);
      if (std::string(side) == "left") {
        source = last_left_image_;
        detection = left_detection_;
      } else {
        source = last_right_image_;
        detection = right_detection_;
      }
    }
    if (!source) {
      return;
    }
    (void)side;
    auto output = std::make_unique<sensor_msgs::msg::Image>(*source);
    drawDetection(*output, detection, pair_has_finite_ray_inputs);
    publisher->publish(std::move(output));
  }

  void drawDetection(
    sensor_msgs::msg::Image & image,
    const Detection & detection,
    bool pair_has_finite_ray_inputs)
  {
    if (image.height == 0 || image.width == 0 || image.step < image.width * 3 ||
        image.data.size() < static_cast<size_t>(image.step) * image.height) {
      return;
    }

    if (!detection.valid) {
      drawMissingBorder(image);
      return;
    }

    const int width = static_cast<int>(image.width);
    const int height = static_cast<int>(image.height);
    const bool confidence_ok = detection.confidence >= 0.25F;
    const std::array<uint8_t, 3> green{0, 255, 0};
    const std::array<uint8_t, 3> yellow{255, 210, 0};
    const std::array<uint8_t, 3> orange{255, 96, 0};
    const auto color =
      pair_has_finite_ray_inputs && confidence_ok ? green :
      pair_has_finite_ray_inputs ? yellow : orange;

    int x1 = clampInt(std::lround(detection.x1), 0, width - 1);
    int x2 = clampInt(std::lround(detection.x2), 0, width - 1);
    int y1 = clampInt(std::lround(detection.y1), 0, height - 1);
    int y2 = clampInt(std::lround(detection.y2), 0, height - 1);
    if (x2 < x1) {
      std::swap(x1, x2);
    }
    if (y2 < y1) {
      std::swap(y1, y2);
    }

    constexpr int thickness = 4;
    fillRect(image, x1, y1, x2, std::min(y1 + thickness - 1, height - 1), color);
    fillRect(image, x1, std::max(y2 - thickness + 1, 0), x2, y2, color);
    fillRect(image, x1, y1, std::min(x1 + thickness - 1, width - 1), y2, color);
    fillRect(image, std::max(x2 - thickness + 1, 0), y1, x2, y2, color);
    const int cx = clampInt(std::lround(detection.cx), 0, width - 1);
    const int cy = clampInt(std::lround(detection.cy), 0, height - 1);
    drawCross(image, cx, cy, 10, color);
    fillRect(image, 0, 0, width - 1, std::min(5, height - 1), color);
  }

  void drawMissingBorder(sensor_msgs::msg::Image & image)
  {
    const int width = static_cast<int>(image.width);
    const int height = static_cast<int>(image.height);
    const std::array<uint8_t, 3> red{255, 0, 0};
    constexpr int thickness = 6;
    fillRect(image, 0, 0, width - 1, std::min(thickness - 1, height - 1), red);
    fillRect(image, 0, std::max(height - thickness, 0), width - 1, height - 1, red);
    fillRect(image, 0, 0, std::min(thickness - 1, width - 1), height - 1, red);
    fillRect(image, std::max(width - thickness, 0), 0, width - 1, height - 1, red);
  }

  void fillRect(
    sensor_msgs::msg::Image & image,
    int x1,
    int y1,
    int x2,
    int y2,
    const std::array<uint8_t, 3> & color)
  {
    if (x2 < x1 || y2 < y1) {
      return;
    }
    for (int y = y1; y <= y2; ++y) {
      auto row = image.data.data() + static_cast<size_t>(y) * image.step;
      for (int x = x1; x <= x2; ++x) {
        auto pixel = row + static_cast<size_t>(x) * 3;
        pixel[0] = color[0];
        pixel[1] = color[1];
        pixel[2] = color[2];
      }
    }
  }

  void drawCross(
    sensor_msgs::msg::Image & image,
    int cx,
    int cy,
    int radius,
    const std::array<uint8_t, 3> & color)
  {
    const int width = static_cast<int>(image.width);
    const int height = static_cast<int>(image.height);
    const int x1 = clampInt(cx - radius, 0, width - 1);
    const int x2 = clampInt(cx + radius, 0, width - 1);
    const int y1 = clampInt(cy - radius, 0, height - 1);
    const int y2 = clampInt(cy + radius, 0, height - 1);
    fillRect(image, x1, std::max(cy - 1, 0), x2, std::min(cy + 1, height - 1), color);
    fillRect(image, std::max(cx - 1, 0), y1, std::min(cx + 1, width - 1), y2, color);
  }

  static int clampInt(long value, int lower, int upper)
  {
    return std::max(lower, std::min(static_cast<int>(value), upper));
  }

  std::mutex mutex_;
  double overlay_publish_hz_{20.0};
  std::chrono::duration<double> min_publish_period_{0.05};
  sensor_msgs::msg::Image::ConstSharedPtr last_left_image_;
  sensor_msgs::msg::Image::ConstSharedPtr last_right_image_;
  Detection left_detection_;
  Detection right_detection_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr detections_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_pub_;
  bool publish_left_next_{true};
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DetectionOverlayNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
