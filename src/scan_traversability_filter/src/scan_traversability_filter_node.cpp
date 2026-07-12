// Copyright 2026 neepu
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/TypeDefs.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "scan_traversability_filter/height_filter.hpp"

class ScanTraversabilityFilter : public rclcpp::Node
{
public:
  ScanTraversabilityFilter()
  : Node("scan_traversability_filter")
  {
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/registered_scan");
    grid_map_topic_ = declare_parameter<std::string>(
      "grid_map_topic", "/elevation_mapping_node/elevation_map_raw");
    output_topic_ = declare_parameter<std::string>("output_topic", "/terrain_map");
    traversability_layer_ = declare_parameter<std::string>(
      "traversability_layer", "traversability");
    output_frame_id_ = declare_parameter<std::string>("output_frame_id", "map");
    unknown_as_obstacle_ = declare_parameter<bool>("unknown_as_obstacle", false);
    min_risk_to_publish_ = declare_parameter<double>("min_risk_to_publish", 0.0);
    clamp_traversability_ = declare_parameter<bool>("clamp_traversability", true);
    enable_height_filter_ = declare_parameter<bool>("enable_height_filter", true);
    height_reference_frame_ = declare_parameter<std::string>(
      "height_reference_frame", "body");
    height_filter_parameters_.max_height_range = declare_parameter<double>(
      "max_height_range", 0.3);
    height_filter_parameters_.ramped_height_range_a = declare_parameter<double>(
      "ramped_height_range_a", 0.5);
    height_filter_parameters_.ramped_height_range_b = declare_parameter<double>(
      "ramped_height_range_b", 0.8);
    height_filter_parameters_.ramped_height_range_c = declare_parameter<double>(
      "ramped_height_range_c", 0.35);
    tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 0.05);

    validateHeightFilterParameters();
    if (enable_height_filter_) {
      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    auto qos = rclcpp::SensorDataQoS();
    grid_map_sub_ = create_subscription<grid_map_msgs::msg::GridMap>(
      grid_map_topic_, qos,
      std::bind(&ScanTraversabilityFilter::gridMapCallback, this, std::placeholders::_1));
    scan_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      scan_topic_, qos,
      std::bind(&ScanTraversabilityFilter::scanCallback, this, std::placeholders::_1));
    terrain_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 5);

    RCLCPP_INFO(
      get_logger(),
      "scan_traversability_filter: %s + %s[%s] -> %s",
      scan_topic_.c_str(), grid_map_topic_.c_str(), traversability_layer_.c_str(),
      output_topic_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "height filter: %s, reference=%s, max=%.3f, ramp=(%.3f, %.3f, %.3f)",
      enable_height_filter_ ? "enabled" : "disabled", height_reference_frame_.c_str(),
      height_filter_parameters_.max_height_range,
      height_filter_parameters_.ramped_height_range_a,
      height_filter_parameters_.ramped_height_range_b,
      height_filter_parameters_.ramped_height_range_c);
  }

private:
  void validateHeightFilterParameters() const
  {
    if (!enable_height_filter_) {
      return;
    }
    if (height_reference_frame_.empty()) {
      throw std::invalid_argument("height_reference_frame must not be empty");
    }
    if (height_filter_parameters_.max_height_range < 0.0 ||
      height_filter_parameters_.ramped_height_range_a < 0.0 ||
      height_filter_parameters_.ramped_height_range_b < 0.0 ||
      height_filter_parameters_.ramped_height_range_c < 0.0)
    {
      throw std::invalid_argument("height filter ranges and slope must be non-negative");
    }
    if (tf_timeout_sec_ < 0.0) {
      throw std::invalid_argument("tf_timeout_sec must be non-negative");
    }
  }

  std::optional<geometry_msgs::msg::TransformStamped> lookupHeightReference(
    const std::string & target_frame,
    const builtin_interfaces::msg::Time & stamp)
  {
    const auto timeout = rclcpp::Duration::from_seconds(tf_timeout_sec_);
    std::string stamped_error;

    try {
      return tf_buffer_->lookupTransform(
        target_frame, height_reference_frame_, rclcpp::Time(stamp), timeout);
    } catch (const tf2::TransformException & exception) {
      stamped_error = exception.what();
    }

    try {
      return tf_buffer_->lookupTransform(
        target_frame, height_reference_frame_, tf2::TimePointZero,
        tf2::durationFromSec(tf_timeout_sec_));
    } catch (const tf2::TransformException & exception) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Cannot apply height filter: transform %s <- %s is unavailable "
        "(stamped: %s; latest: %s). Dropping scan.",
        target_frame.c_str(), height_reference_frame_.c_str(),
        stamped_error.c_str(), exception.what());
      return std::nullopt;
    }
  }

  void gridMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg)
  {
    auto next_map = std::make_shared<grid_map::GridMap>();
    grid_map::GridMapRosConverter::fromMessage(*msg, *next_map);

    if (!next_map->exists(traversability_layer_)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "GridMap does not contain traversability layer '%s'.",
        traversability_layer_.c_str());
      return;
    }

    std::lock_guard<std::mutex> lock(map_mutex_);
    latest_map_ = next_map;
  }

  void scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr scan_msg)
  {
    std::shared_ptr<grid_map::GridMap> map;
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      map = latest_map_;
    }

    if (!map) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "No GridMap received yet; skipping scan.");
      return;
    }

    const std::string scan_frame = scan_msg->header.frame_id;
    const std::string map_frame = map->getFrameId();
    if (scan_frame.empty() || map_frame.empty() || scan_frame != map_frame) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Scan frame '%s' does not match GridMap frame '%s'; dropping scan.",
        scan_frame.c_str(), map_frame.c_str());
      return;
    }

    double reference_x = 0.0;
    double reference_y = 0.0;
    double reference_z = 0.0;
    if (enable_height_filter_) {
      const auto reference = lookupHeightReference(scan_frame, scan_msg->header.stamp);
      if (!reference) {
        return;
      }
      reference_x = reference->transform.translation.x;
      reference_y = reference->transform.translation.y;
      reference_z = reference->transform.translation.z;
    }

    sensor_msgs::msg::PointCloud2 terrain_msg;
    terrain_msg.header = scan_msg->header;
    if (!output_frame_id_.empty()) {
      terrain_msg.header.frame_id = output_frame_id_;
    }

    sensor_msgs::PointCloud2Modifier modifier(terrain_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.setPointCloud2Fields(
      4,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(scan_msg->width * scan_msg->height);

    sensor_msgs::PointCloud2ConstIterator<float> in_x(*scan_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> in_y(*scan_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> in_z(*scan_msg, "z");

    sensor_msgs::PointCloud2Iterator<float> out_x(terrain_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(terrain_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(terrain_msg, "z");
    sensor_msgs::PointCloud2Iterator<float> out_intensity(terrain_msg, "intensity");

    size_t output_count = 0;
    for (; in_x != in_x.end(); ++in_x, ++in_y, ++in_z) {
      const double x = *in_x;
      const double y = *in_y;
      const double z = *in_z;

      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }

      if (enable_height_filter_ && !scan_traversability_filter::isWithinHeightLimit(
          x, y, z, reference_x, reference_y, reference_z, height_filter_parameters_))
      {
        continue;
      }

      const grid_map::Position position(x, y);
      float traversability = 0.0F;
      if (!sampleTraversability(*map, position, traversability)) {
        if (!unknown_as_obstacle_) {
          continue;
        }
        traversability = 0.0F;
      }

      if (clamp_traversability_) {
        traversability = std::clamp(traversability, 0.0F, 1.0F);
      }

      const float risk = 1.0F - traversability;
      if (risk < min_risk_to_publish_) {
        continue;
      }

      *out_x = static_cast<float>(x);
      *out_y = static_cast<float>(y);
      *out_z = static_cast<float>(z);
      *out_intensity = risk;

      ++out_x;
      ++out_y;
      ++out_z;
      ++out_intensity;
      ++output_count;
    }

    modifier.resize(output_count);
    terrain_msg.height = 1;
    terrain_msg.width = static_cast<uint32_t>(output_count);
    terrain_msg.is_dense = true;
    terrain_pub_->publish(terrain_msg);
  }

  bool sampleTraversability(
    const grid_map::GridMap & map,
    const grid_map::Position & position,
    float & value) const
  {
    if (!map.isInside(position)) {
      return false;
    }

    const auto traversability = map.atPosition(traversability_layer_, position);
    if (!std::isfinite(traversability)) {
      return false;
    }

    value = static_cast<float>(traversability);
    return true;
  }

  std::string scan_topic_;
  std::string grid_map_topic_;
  std::string output_topic_;
  std::string traversability_layer_;
  std::string output_frame_id_;
  bool unknown_as_obstacle_;
  double min_risk_to_publish_;
  bool clamp_traversability_;
  bool enable_height_filter_;
  std::string height_reference_frame_;
  scan_traversability_filter::HeightFilterParameters height_filter_parameters_;
  double tf_timeout_sec_;

  std::mutex map_mutex_;
  std::shared_ptr<grid_map::GridMap> latest_map_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr terrain_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanTraversabilityFilter>());
  rclcpp::shutdown();
  return 0;
}
