// Copyright 2026 neepu
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file or at
// https://developers.google.com/open-source/licenses/bsd

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace
{

constexpr float kNan = std::numeric_limits<float>::quiet_NaN();

float clampFloat(const float value, const float low, const float high)
{
  return std::min(std::max(value, low), high);
}

bool isFinite(const float value)
{
  return std::isfinite(value);
}

float smoothStep(const float low, const float high, const float value)
{
  if (!std::isfinite(value)) {
    return kNan;
  }
  if (high <= low) {
    return value >= high ? 1.0f : 0.0f;
  }

  const float t = clampFloat((value - low) / (high - low), 0.0f, 1.0f);
  return t * t * (3.0f - 2.0f * t);
}

}  // namespace

class GridMapTerrainNode : public rclcpp::Node
{
public:
  GridMapTerrainNode()
  : Node("grid_map_terrain_node")
  {
    declareParameters();
    readParameters();
    initializeMap();

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(20),
      std::bind(&GridMapTerrainNode::odomCallback, this, std::placeholders::_1));

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&GridMapTerrainNode::cloudCallback, this, std::placeholders::_1));

    terrain_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      terrain_map_topic_, rclcpp::QoS(2));
    elevation_pub_ = create_publisher<grid_map_msgs::msg::GridMap>(
      elevation_map_topic_, rclcpp::QoS(1));
    traversability_pub_ = create_publisher<grid_map_msgs::msg::GridMap>(
      traversability_map_topic_, rclcpp::QoS(1));
    elevation_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      elevation_map_cloud_topic_, rclcpp::QoS(1));
    traversability_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      traversability_map_cloud_topic_, rclcpp::QoS(1));
    terrain_risk_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      terrain_risk_cloud_topic_, rclcpp::QoS(1));
    dynamic_obstacle_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      dynamic_obstacle_cloud_topic_, rclcpp::QoS(1));
    terrain_height_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      terrain_height_cloud_topic_, rclcpp::QoS(1));

    RCLCPP_INFO(
      get_logger(),
      "grid_map terrain node ready: cloud=%s odom=%s terrain=%s frame=%s",
      input_cloud_topic_.c_str(), odom_topic_.c_str(), terrain_map_topic_.c_str(),
      map_frame_.c_str());
  }

private:
  struct CellAccumulator
  {
    std::vector<float> z_values;
    float max_z{-std::numeric_limits<float>::infinity()};
  };

  struct DynamicCell
  {
    float score{0.0f};
    double last_seen{-std::numeric_limits<double>::infinity()};
    double last_update{-std::numeric_limits<double>::infinity()};
  };

  void declareParameters()
  {
    declare_parameter<std::string>("input_cloud_topic", "/registered_scan");
    declare_parameter<std::string>("odom_topic", "/state_estimation");
    declare_parameter<std::string>("terrain_map_topic", "/terrain_map");
    declare_parameter<std::string>("elevation_map_topic", "/elevation_map");
    declare_parameter<std::string>("traversability_map_topic", "/traversability_map");
    declare_parameter<std::string>("elevation_map_cloud_topic", "/elevation_map_cloud");
    declare_parameter<std::string>("traversability_map_cloud_topic", "/traversability_map_cloud");
    declare_parameter<std::string>("terrain_risk_cloud_topic", "/terrain_risk_cloud");
    declare_parameter<std::string>("dynamic_obstacle_cloud_topic", "/dynamic_obstacle_cloud");
    declare_parameter<std::string>("terrain_height_cloud_topic", "/terrain_height_cloud");
    declare_parameter<std::string>("map_frame", "map");

    declare_parameter<double>("map_length_x", 6.0);
    declare_parameter<double>("map_length_y", 6.0);
    declare_parameter<double>("resolution", 0.05);
    declare_parameter<double>("publish_rate", 10.0);

    declare_parameter<double>("min_rel_z", -0.6);
    declare_parameter<double>("max_rel_z", 1.0);
    declare_parameter<double>("dis_ratio_z", 0.2);
    declare_parameter<double>("min_ground_relative_height", -0.05);
    declare_parameter<double>("max_ground_relative_height", 0.6);
    declare_parameter<int>("min_points_per_cell", 3);
    declare_parameter<double>("elevation_quantile", 0.25);

    declare_parameter<int>("normal_radius_cells", 2);
    declare_parameter<int>("roughness_radius_cells", 2);
    declare_parameter<int>("step_radius_cells", 2);

    declare_parameter<double>("height_safe", 0.03);
    declare_parameter<double>("height_block", 0.18);
    declare_parameter<double>("slope_safe", 0.35);
    declare_parameter<double>("slope_block", 0.70);
    declare_parameter<double>("roughness_safe", 0.02);
    declare_parameter<double>("roughness_block", 0.08);
    declare_parameter<double>("step_safe", 0.04);
    declare_parameter<double>("step_block", 0.18);
    declare_parameter<double>("unknown_traversability", 0.45);
    declare_parameter<int>("min_density_for_confidence", 2);
    declare_parameter<double>("risk_gamma", 1.0);
    declare_parameter<double>("risk_obstacle_threshold", 0.65);

    declare_parameter<bool>("dynamic_enabled", true);
    declare_parameter<double>("dynamic_hit_gain", 0.35);
    declare_parameter<double>("dynamic_free_decay", 0.55);
    declare_parameter<double>("dynamic_time_decay", 0.85);
    declare_parameter<double>("dynamic_keep_time", 3.0);
    declare_parameter<double>("dynamic_min_height_risk", 0.35);
    declare_parameter<double>("dynamic_risk_threshold", 0.65);
    declare_parameter<double>("dynamic_decay_time", 0.8);

    declare_parameter<bool>("publish_only_blocking_points", false);
    declare_parameter<double>("blocking_intensity_threshold", 0.15);
    declare_parameter<bool>("publish_debug_maps", true);
    declare_parameter<bool>("publish_debug_clouds", true);
  }

  void readParameters()
  {
    get_parameter("input_cloud_topic", input_cloud_topic_);
    get_parameter("odom_topic", odom_topic_);
    get_parameter("terrain_map_topic", terrain_map_topic_);
    get_parameter("elevation_map_topic", elevation_map_topic_);
    get_parameter("traversability_map_topic", traversability_map_topic_);
    get_parameter("elevation_map_cloud_topic", elevation_map_cloud_topic_);
    get_parameter("traversability_map_cloud_topic", traversability_map_cloud_topic_);
    get_parameter("terrain_risk_cloud_topic", terrain_risk_cloud_topic_);
    get_parameter("dynamic_obstacle_cloud_topic", dynamic_obstacle_cloud_topic_);
    get_parameter("terrain_height_cloud_topic", terrain_height_cloud_topic_);
    get_parameter("map_frame", map_frame_);

    get_parameter("map_length_x", map_length_x_);
    get_parameter("map_length_y", map_length_y_);
    get_parameter("resolution", resolution_);
    get_parameter("publish_rate", publish_rate_);

    get_parameter("min_rel_z", min_rel_z_);
    get_parameter("max_rel_z", max_rel_z_);
    get_parameter("dis_ratio_z", dis_ratio_z_);
    get_parameter("min_ground_relative_height", min_ground_relative_height_);
    get_parameter("max_ground_relative_height", max_ground_relative_height_);
    get_parameter("min_points_per_cell", min_points_per_cell_);
    get_parameter("elevation_quantile", elevation_quantile_);

    get_parameter("normal_radius_cells", normal_radius_cells_);
    get_parameter("roughness_radius_cells", roughness_radius_cells_);
    get_parameter("step_radius_cells", step_radius_cells_);

    get_parameter("height_safe", height_safe_);
    get_parameter("height_block", height_block_);
    get_parameter("slope_safe", slope_safe_);
    get_parameter("slope_block", slope_block_);
    get_parameter("roughness_safe", roughness_safe_);
    get_parameter("roughness_block", roughness_block_);
    get_parameter("step_safe", step_safe_);
    get_parameter("step_block", step_block_);
    get_parameter("unknown_traversability", unknown_traversability_);
    get_parameter("min_density_for_confidence", min_density_for_confidence_);
    get_parameter("risk_gamma", risk_gamma_);
    get_parameter("risk_obstacle_threshold", risk_obstacle_threshold_);

    get_parameter("dynamic_enabled", dynamic_enabled_);
    get_parameter("dynamic_hit_gain", dynamic_hit_gain_);
    get_parameter("dynamic_free_decay", dynamic_free_decay_);
    get_parameter("dynamic_time_decay", dynamic_time_decay_);
    get_parameter("dynamic_keep_time", dynamic_keep_time_);
    get_parameter("dynamic_min_height_risk", dynamic_min_height_risk_);
    get_parameter("dynamic_risk_threshold", dynamic_risk_threshold_);
    get_parameter("dynamic_decay_time", dynamic_decay_time_);

    get_parameter("publish_only_blocking_points", publish_only_blocking_points_);
    get_parameter("blocking_intensity_threshold", blocking_intensity_threshold_);
    get_parameter("publish_debug_maps", publish_debug_maps_);
    get_parameter("publish_debug_clouds", publish_debug_clouds_);

    elevation_quantile_ = clampFloat(elevation_quantile_, 0.0f, 1.0f);
    if (slope_block_ <= slope_safe_) {
      slope_block_ = slope_safe_ + 0.01;
    }
    if (height_block_ <= height_safe_) {
      height_block_ = height_safe_ + 0.01;
    }
    if (roughness_block_ <= roughness_safe_) {
      roughness_block_ = roughness_safe_ + 0.01;
    }
    if (step_block_ <= step_safe_) {
      step_block_ = step_safe_ + 0.01;
    }
    if (publish_rate_ <= 0.0) {
      publish_rate_ = 10.0;
    }
    if (dis_ratio_z_ < 0.0) {
      dis_ratio_z_ = 0.0;
    }
    if (max_ground_relative_height_ <= min_ground_relative_height_) {
      max_ground_relative_height_ = min_ground_relative_height_ + 0.01;
    }
    unknown_traversability_ = clampFloat(unknown_traversability_, 0.0f, 1.0f);
    risk_obstacle_threshold_ = clampFloat(risk_obstacle_threshold_, 0.0f, 1.0f);
    dynamic_hit_gain_ = clampFloat(dynamic_hit_gain_, 0.0f, 1.0f);
    dynamic_free_decay_ = clampFloat(dynamic_free_decay_, 0.0f, 1.0f);
    dynamic_time_decay_ = clampFloat(dynamic_time_decay_, 0.0f, 1.0f);
    dynamic_min_height_risk_ = clampFloat(dynamic_min_height_risk_, 0.0f, 1.0f);
    dynamic_risk_threshold_ = clampFloat(dynamic_risk_threshold_, 0.0f, 1.0f);
    if (min_density_for_confidence_ < 1) {
      min_density_for_confidence_ = 1;
    }
    if (risk_gamma_ <= 0.0) {
      risk_gamma_ = 1.0;
    }
    if (dynamic_keep_time_ <= 0.0) {
      dynamic_keep_time_ = 3.0;
    }
    if (dynamic_decay_time_ <= 0.0) {
      dynamic_decay_time_ = 0.8;
    }
  }

  void initializeMap()
  {
    const std::vector<std::string> layers = {
      "elevation",
      "max_height",
      "height_gap",
      "density",
      "elevation_inpainted",
      "elevation_smooth",
      "normal_vectors_z",
      "slope",
      "roughness",
      "step_height",
      "height_risk",
      "slope_risk",
      "roughness_risk",
      "step_risk",
      "density_confidence",
      "traversability",
      "terrain_risk",
      "risk",
      "final_risk",
      "static_obstacle",
      "dynamic_obstacle",
      "dynamic_risk",
      "intensity"};

    map_ = grid_map::GridMap(layers);
    map_.setFrameId(map_frame_);
    map_.setGeometry(
      grid_map::Length(map_length_x_, map_length_y_), resolution_,
      grid_map::Position(0.0, 0.0));
    map_.setBasicLayers({"elevation"});

    dynamic_cache_.clear();
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;
    robot_z_ = msg->pose.pose.position.z;
    latest_odom_stamp_ = msg->header.stamp;
    have_odom_ = true;
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!have_odom_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for odometry before publishing terrain map.");
      return;
    }

    const double now = rclcpp::Time(msg->header.stamp).seconds();
    if (now - last_publish_time_ < 1.0 / publish_rate_) {
      return;
    }
    last_publish_time_ = now;

    map_.setTimestamp(rclcpp::Time(msg->header.stamp).nanoseconds());
    map_.setFrameId(map_frame_);
    map_.setPosition(grid_map::Position(robot_x_, robot_y_));
    clearLayers();

    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(*msg, cloud);

    std::vector<CellAccumulator> accumulators(map_.getSize().prod());
    pcl::PointCloud<pcl::PointXYZI> annotation_cloud;
    accumulateCloud(cloud, accumulators, annotation_cloud);
    fillElevationLayers(accumulators);
    fillUnknownElevation();
    updateHeightGapLayers(annotation_cloud);
    computeTerrainLayers();
    updateDynamicObstacles(now);
    composeIntensity();
    publishTerrainCloud(annotation_cloud, msg->header.stamp);
    publishDebugMaps(msg->header.stamp);
  }

  void clearLayers()
  {
    for (const auto & layer : map_.getLayers()) {
      map_[layer].setConstant(kNan);
    }
  }

  size_t linearIndex(const grid_map::Index & index) const
  {
    return static_cast<size_t>(index(0) * map_.getSize()(1) + index(1));
  }

  void accumulateCloud(
    const pcl::PointCloud<pcl::PointXYZI> & cloud,
    std::vector<CellAccumulator> & accumulators,
    pcl::PointCloud<pcl::PointXYZI> & annotation_cloud)
  {
    const double half_x = map_length_x_ * 0.5;
    const double half_y = map_length_y_ * 0.5;
    annotation_cloud.clear();
    annotation_cloud.reserve(cloud.points.size());

    for (const auto & point : cloud.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      if (std::abs(point.x - robot_x_) > half_x || std::abs(point.y - robot_y_) > half_y) {
        continue;
      }
      const double dis = std::hypot(point.x - robot_x_, point.y - robot_y_);
      const double rel_z = point.z - robot_z_;
      if (
        rel_z < min_rel_z_ - dis_ratio_z_ * dis ||
        rel_z > max_rel_z_ + dis_ratio_z_ * dis)
      {
        continue;
      }

      grid_map::Index index;
      if (!map_.getIndex(grid_map::Position(point.x, point.y), index)) {
        continue;
      }

      auto & cell = accumulators[linearIndex(index)];
      cell.z_values.push_back(point.z);
      annotation_cloud.push_back(point);
    }
  }

  float quantile(std::vector<float> values, const float q) const
  {
    if (values.empty()) {
      return kNan;
    }
    std::sort(values.begin(), values.end());
    const auto id = static_cast<size_t>(
      clampFloat(q, 0.0f, 1.0f) * static_cast<float>(values.size() - 1));
    return values[id];
  }

  void fillElevationLayers(const std::vector<CellAccumulator> & accumulators)
  {
    for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator) {
      const grid_map::Index index(*iterator);
      const auto & cell = accumulators[linearIndex(index)];
      const auto point_count = static_cast<int>(cell.z_values.size());

      map_.at("density", index) = static_cast<float>(point_count);
      if (point_count < min_points_per_cell_) {
        continue;
      }

      const float elevation = quantile(cell.z_values, elevation_quantile_);
      if (!isFinite(elevation)) {
        continue;
      }

      map_.at("elevation", index) = elevation;
    }
  }

  void updateHeightGapLayers(const pcl::PointCloud<pcl::PointXYZI> & annotation_cloud)
  {
    for (const auto & point : annotation_cloud.points) {
      grid_map::Index index;
      if (!map_.getIndex(grid_map::Position(point.x, point.y), index)) {
        continue;
      }

      const float elevation = map_.at("elevation_inpainted", index);
      if (!isFinite(elevation)) {
        continue;
      }

      const float relative_height = point.z - elevation;
      if (
        relative_height < min_ground_relative_height_ ||
        relative_height > max_ground_relative_height_ ||
        relative_height < 0.0f)
      {
        continue;
      }

      const float current_height_gap = map_.at("height_gap", index);
      if (!isFinite(current_height_gap) || relative_height > current_height_gap) {
        map_.at("height_gap", index) = relative_height;
        map_.at("max_height", index) = point.z;
      }
    }
  }

  void fillUnknownElevation()
  {
    map_["elevation_inpainted"] = map_["elevation"];

    for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator) {
      const grid_map::Index index(*iterator);
      if (isFinite(map_.at("elevation_inpainted", index))) {
        continue;
      }

      float sum = 0.0f;
      int count = 0;
      forEachNeighbor(
        index, 2, [&](const grid_map::Index & neighbor) {
          const float value = map_.at("elevation", neighbor);
          if (isFinite(value)) {
            sum += value;
            ++count;
          }
        });

      if (count > 0) {
        map_.at("elevation_inpainted", index) = sum / static_cast<float>(count);
      }
    }
  }

  template<typename Callback>
  void forEachNeighbor(const grid_map::Index & center, const int radius, Callback callback) const
  {
    const auto size = map_.getSize();
    for (int dx = -radius; dx <= radius; ++dx) {
      for (int dy = -radius; dy <= radius; ++dy) {
        const grid_map::Index index(center(0) + dx, center(1) + dy);
        if (index(0) < 0 || index(1) < 0 || index(0) >= size(0) || index(1) >= size(1)) {
          continue;
        }
        callback(index);
      }
    }
  }

  void computeTerrainLayers()
  {
    computeSmoothElevation();
    computeSlope();
    computeRoughness();
    computeStepHeight();
    computeTraversabilityAndStaticObstacles();
  }

  void computeSmoothElevation()
  {
    for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator) {
      const grid_map::Index index(*iterator);
      float sum = 0.0f;
      int count = 0;
      forEachNeighbor(
        index, roughness_radius_cells_, [&](const grid_map::Index & neighbor) {
          const float value = map_.at("elevation_inpainted", neighbor);
          if (isFinite(value)) {
            sum += value;
            ++count;
          }
        });
      if (count > 0) {
        map_.at("elevation_smooth", index) = sum / static_cast<float>(count);
      }
    }
  }

  void computeSlope()
  {
    const int r = std::max(1, normal_radius_cells_);
    const double span = 2.0 * r * resolution_;

    for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator) {
      const grid_map::Index index(*iterator);
      const grid_map::Index ix_low(index(0) - r, index(1));
      const grid_map::Index ix_high(index(0) + r, index(1));
      const grid_map::Index iy_low(index(0), index(1) - r);
      const grid_map::Index iy_high(index(0), index(1) + r);
      const auto size = map_.getSize();
      if (
        ix_low(0) < 0 || ix_high(0) >= size(0) ||
        iy_low(1) < 0 || iy_high(1) >= size(1))
      {
        continue;
      }

      const float z_x_low = map_.at("elevation_inpainted", ix_low);
      const float z_x_high = map_.at("elevation_inpainted", ix_high);
      const float z_y_low = map_.at("elevation_inpainted", iy_low);
      const float z_y_high = map_.at("elevation_inpainted", iy_high);
      if (!isFinite(z_x_low) || !isFinite(z_x_high) || !isFinite(z_y_low) || !isFinite(z_y_high)) {
        continue;
      }

      const float dz_dx = static_cast<float>((z_x_high - z_x_low) / span);
      const float dz_dy = static_cast<float>((z_y_high - z_y_low) / span);
      const float normal_z = 1.0f / std::sqrt(1.0f + dz_dx * dz_dx + dz_dy * dz_dy);
      const float slope = std::acos(clampFloat(normal_z, 0.0f, 1.0f));
      map_.at("normal_vectors_z", index) = normal_z;
      map_.at("slope", index) = slope;
    }
  }

  void computeRoughness()
  {
    for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator) {
      const grid_map::Index index(*iterator);
      const float elevation = map_.at("elevation_inpainted", index);
      const float smooth = map_.at("elevation_smooth", index);
      if (isFinite(elevation) && isFinite(smooth)) {
        map_.at("roughness", index) = std::abs(elevation - smooth);
      }
    }
  }

  void computeStepHeight()
  {
    for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator) {
      const grid_map::Index index(*iterator);
      float min_z = std::numeric_limits<float>::infinity();
      float max_z = -std::numeric_limits<float>::infinity();
      int count = 0;
      forEachNeighbor(
        index, step_radius_cells_, [&](const grid_map::Index & neighbor) {
          const float value = map_.at("elevation_inpainted", neighbor);
          if (isFinite(value)) {
            min_z = std::min(min_z, value);
            max_z = std::max(max_z, value);
            ++count;
          }
        });
      if (count > 0) {
        map_.at("step_height", index) = std::max(0.0f, max_z - min_z);
      }
    }
  }

  void computeTraversabilityAndStaticObstacles()
  {
    for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator) {
      const grid_map::Index index(*iterator);
      const float slope = map_.at("slope", index);
      const float roughness = map_.at("roughness", index);
      const float step = map_.at("step_height", index);
      const float height_gap = map_.at("height_gap", index);
      const float density = map_.at("density", index);

      const float density_confidence = isFinite(density) ?
        clampFloat(
          density / static_cast<float>(std::max(1, min_density_for_confidence_)),
          0.0f, 1.0f) :
        0.0f;
      map_.at("density_confidence", index) = density_confidence;

      const float height_risk = isFinite(height_gap) ?
        smoothStep(
          static_cast<float>(height_safe_),
          static_cast<float>(height_block_),
          std::max(0.0f, height_gap)) :
        0.0f;
      map_.at("height_risk", index) = height_risk;

      const bool has_shape =
        isFinite(slope) && isFinite(roughness) && isFinite(step);

      if (!has_shape) {
        const float terrain_traversability =
          density_confidence * (1.0f - height_risk) +
          (1.0f - density_confidence) * static_cast<float>(unknown_traversability_);
        const float terrain_risk = 1.0f - clampFloat(terrain_traversability, 0.0f, 1.0f);
        map_.at("traversability", index) = 1.0f - terrain_risk;
        map_.at("terrain_risk", index) = terrain_risk;
        map_.at("static_obstacle", index) =
          terrain_risk >= risk_obstacle_threshold_ ? 1.0f : 0.0f;
        continue;
      }

      const float slope_risk = smoothStep(
        static_cast<float>(slope_safe_), static_cast<float>(slope_block_), slope);
      const float roughness_risk = smoothStep(
        static_cast<float>(roughness_safe_), static_cast<float>(roughness_block_), roughness);
      const float step_risk = smoothStep(
        static_cast<float>(step_safe_), static_cast<float>(step_block_), step);

      map_.at("slope_risk", index) = slope_risk;
      map_.at("roughness_risk", index) = roughness_risk;
      map_.at("step_risk", index) = step_risk;

      const float observed_traversability =
        (1.0f - height_risk) *
        (1.0f - slope_risk) *
        (1.0f - roughness_risk) *
        (1.0f - step_risk);
      const float traversability =
        density_confidence * observed_traversability +
        (1.0f - density_confidence) * static_cast<float>(unknown_traversability_);
      const float terrain_risk = 1.0f - clampFloat(traversability, 0.0f, 1.0f);

      map_.at("traversability", index) = 1.0f - terrain_risk;
      map_.at("terrain_risk", index) = terrain_risk;
      map_.at("static_obstacle", index) =
        terrain_risk >= risk_obstacle_threshold_ ? 1.0f : 0.0f;
    }
  }

  std::uint64_t dynamicCellKey(const double x, const double y) const
  {
    const auto ix = static_cast<std::int32_t>(std::floor(x / resolution_));
    const auto iy = static_cast<std::int32_t>(std::floor(y / resolution_));
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(ix)) << 32) |
           static_cast<std::uint32_t>(iy);
  }

  void applyDynamicTimeDecay(DynamicCell & cell, const double now) const
  {
    if (!std::isfinite(cell.last_update)) {
      cell.last_update = now;
      return;
    }

    const double elapsed = now - cell.last_update;
    if (elapsed <= 0.0) {
      return;
    }

    const double decay_steps = elapsed / dynamic_decay_time_;
    cell.score *= static_cast<float>(std::pow(dynamic_time_decay_, decay_steps));
    cell.score = clampFloat(cell.score, 0.0f, 1.0f);
    cell.last_update = now;
  }

  void decayDynamicCache(const double now)
  {
    for (auto iterator = dynamic_cache_.begin(); iterator != dynamic_cache_.end(); ) {
      applyDynamicTimeDecay(iterator->second, now);
      if (
        now - iterator->second.last_seen > dynamic_keep_time_ &&
        iterator->second.score < 0.01f)
      {
        iterator = dynamic_cache_.erase(iterator);
      } else {
        ++iterator;
      }
    }
  }

  void updateDynamicObstacles(const double now)
  {
    if (!dynamic_enabled_) {
      for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator) {
        const grid_map::Index index(*iterator);
        map_.at("dynamic_risk", index) = 0.0f;
        map_.at("dynamic_obstacle", index) = 0.0f;
      }
      dynamic_cache_.clear();
      return;
    }

    decayDynamicCache(now);

    for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator) {
      const grid_map::Index index(*iterator);
      grid_map::Position position;
      map_.getPosition(index, position);

      const float height_risk = map_.at("height_risk", index);
      const float density_confidence = map_.at("density_confidence", index);
      const auto key = dynamicCellKey(position.x(), position.y());

      if (isFinite(height_risk) && height_risk >= dynamic_min_height_risk_) {
        auto & cell = dynamic_cache_[key];
        applyDynamicTimeDecay(cell, now);
        cell.score = clampFloat(
          cell.score + static_cast<float>(dynamic_hit_gain_) * height_risk,
          0.0f, 1.0f);
        cell.last_seen = now;
        cell.last_update = now;
      } else if (isFinite(density_confidence) && density_confidence > 0.5f) {
        auto cache_iterator = dynamic_cache_.find(key);
        if (cache_iterator != dynamic_cache_.end()) {
          applyDynamicTimeDecay(cache_iterator->second, now);
          cache_iterator->second.score *= static_cast<float>(dynamic_free_decay_);
          cache_iterator->second.last_update = now;
        }
      }
    }

    for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator) {
      const grid_map::Index index(*iterator);
      grid_map::Position position;
      map_.getPosition(index, position);
      const auto cache_iterator = dynamic_cache_.find(dynamicCellKey(position.x(), position.y()));
      const float dynamic_risk =
        cache_iterator != dynamic_cache_.end() ?
        clampFloat(cache_iterator->second.score, 0.0f, 1.0f) :
        0.0f;
      map_.at("dynamic_risk", index) = dynamic_risk;
      map_.at("dynamic_obstacle", index) =
        dynamic_risk >= dynamic_risk_threshold_ ? 1.0f : 0.0f;
    }
  }

  void composeIntensity()
  {
    for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator) {
      const grid_map::Index index(*iterator);
      const float terrain_risk = map_.at("terrain_risk", index);
      if (!isFinite(terrain_risk)) {
        continue;
      }

      const float dynamic_risk = isFinite(map_.at("dynamic_risk", index)) ?
        clampFloat(map_.at("dynamic_risk", index), 0.0f, 1.0f) :
        0.0f;
      float final_risk = 1.0f -
        (1.0f - clampFloat(terrain_risk, 0.0f, 1.0f)) *
        (1.0f - dynamic_risk);
      final_risk = clampFloat(final_risk, 0.0f, 1.0f);
      if (risk_gamma_ != 1.0) {
        final_risk = clampFloat(
          static_cast<float>(std::pow(final_risk, risk_gamma_)), 0.0f, 1.0f);
      }

      map_.at("traversability", index) = 1.0f - final_risk;
      map_.at("final_risk", index) = final_risk;
      map_.at("risk", index) = final_risk;
      map_.at("intensity", index) = final_risk;
    }
  }

  void publishTerrainCloud(
    const pcl::PointCloud<pcl::PointXYZI> & annotation_cloud,
    const builtin_interfaces::msg::Time & stamp)
  {
    pcl::PointCloud<pcl::PointXYZI> terrain_cloud_pcl;
    terrain_cloud_pcl.reserve(annotation_cloud.points.size());
    terrain_cloud_pcl.header.frame_id = map_frame_;

    for (const auto & source_point : annotation_cloud.points) {
      grid_map::Index index;
      if (!map_.getIndex(grid_map::Position(source_point.x, source_point.y), index)) {
        continue;
      }

      const float elevation = map_.at("elevation_inpainted", index);
      if (!isFinite(elevation)) {
        continue;
      }

      const float signed_relative_height = source_point.z - elevation;
      if (
        signed_relative_height < min_ground_relative_height_ ||
        signed_relative_height > max_ground_relative_height_)
      {
        continue;
      }

      const float cell_intensity = map_.at("intensity", index);
      if (!isFinite(cell_intensity)) {
        continue;
      }

      const float intensity = clampFloat(cell_intensity, 0.0f, 1.0f);

      if (publish_only_blocking_points_ && intensity <= blocking_intensity_threshold_) {
        continue;
      }

      pcl::PointXYZI output_point = source_point;
      output_point.intensity = intensity;
      terrain_cloud_pcl.push_back(output_point);
    }

    terrain_cloud_pcl.width = static_cast<uint32_t>(terrain_cloud_pcl.points.size());
    terrain_cloud_pcl.height = 1;
    terrain_cloud_pcl.is_dense = false;

    sensor_msgs::msg::PointCloud2 terrain_cloud_msg;
    pcl::toROSMsg(terrain_cloud_pcl, terrain_cloud_msg);
    terrain_cloud_msg.header.stamp = stamp;
    terrain_cloud_msg.header.frame_id = map_frame_;
    terrain_pub_->publish(terrain_cloud_msg);
  }

  void publishLayerCloud(
    const std::string & z_layer,
    const std::string & intensity_layer,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher,
    const builtin_interfaces::msg::Time & stamp)
  {
    if (!publish_debug_clouds_ || !publisher) {
      return;
    }

    pcl::PointCloud<pcl::PointXYZI> layer_cloud;
    layer_cloud.reserve(static_cast<size_t>(map_.getSize().prod()));
    layer_cloud.header.frame_id = map_frame_;

    for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator) {
      const grid_map::Index index(*iterator);
      const float z = map_.at(z_layer, index);
      if (!isFinite(z)) {
        continue;
      }

      grid_map::Position position;
      map_.getPosition(index, position);

      pcl::PointXYZI point;
      point.x = static_cast<float>(position.x());
      point.y = static_cast<float>(position.y());
      point.z = z;

      const float intensity = map_.at(intensity_layer, index);
      if (!isFinite(intensity)) {
        continue;
      }

      point.intensity = intensity;
      layer_cloud.push_back(point);
    }

    layer_cloud.width = static_cast<uint32_t>(layer_cloud.points.size());
    layer_cloud.height = 1;
    layer_cloud.is_dense = false;

    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(layer_cloud, cloud_msg);
    cloud_msg.header.stamp = stamp;
    cloud_msg.header.frame_id = map_frame_;
    publisher->publish(cloud_msg);
  }

  void publishDebugMaps(const builtin_interfaces::msg::Time & stamp)
  {
    if (publish_debug_maps_) {
      auto elevation_msg = grid_map::GridMapRosConverter::toMessage(
        map_, std::vector<std::string>{"elevation", "max_height", "height_gap", "density"});
      elevation_pub_->publish(std::move(elevation_msg));

      auto traversability_msg = grid_map::GridMapRosConverter::toMessage(
        map_,
        std::vector<std::string>{
        "elevation", "elevation_inpainted", "slope", "roughness", "step_height",
        "height_risk", "slope_risk", "roughness_risk", "step_risk",
        "density_confidence", "terrain_risk", "dynamic_risk", "final_risk",
        "traversability", "risk", "static_obstacle", "dynamic_obstacle", "intensity"});
      traversability_pub_->publish(std::move(traversability_msg));
    }

    publishLayerCloud("elevation", "elevation", elevation_cloud_pub_, stamp);
    publishLayerCloud("elevation_inpainted", "traversability", traversability_cloud_pub_, stamp);
    publishLayerCloud("elevation_inpainted", "terrain_risk", terrain_risk_cloud_pub_, stamp);
    publishLayerCloud("elevation_inpainted", "dynamic_risk", dynamic_obstacle_cloud_pub_, stamp);
    publishLayerCloud("elevation_inpainted", "height_gap", terrain_height_cloud_pub_, stamp);
  }

  std::string input_cloud_topic_;
  std::string odom_topic_;
  std::string terrain_map_topic_;
  std::string elevation_map_topic_;
  std::string traversability_map_topic_;
  std::string elevation_map_cloud_topic_;
  std::string traversability_map_cloud_topic_;
  std::string terrain_risk_cloud_topic_;
  std::string dynamic_obstacle_cloud_topic_;
  std::string terrain_height_cloud_topic_;
  std::string map_frame_;

  double map_length_x_{6.0};
  double map_length_y_{6.0};
  double resolution_{0.05};
  double publish_rate_{10.0};
  double min_rel_z_{-0.6};
  double max_rel_z_{1.0};
  double dis_ratio_z_{0.2};
  double min_ground_relative_height_{-0.05};
  double max_ground_relative_height_{0.6};
  int min_points_per_cell_{3};
  float elevation_quantile_{0.25f};
  int normal_radius_cells_{2};
  int roughness_radius_cells_{2};
  int step_radius_cells_{2};
  double height_safe_{0.03};
  double height_block_{0.18};
  double slope_safe_{0.35};
  double slope_block_{0.70};
  double roughness_safe_{0.02};
  double roughness_block_{0.08};
  double step_safe_{0.04};
  double step_block_{0.18};
  double unknown_traversability_{0.45};
  int min_density_for_confidence_{2};
  double risk_gamma_{1.0};
  double risk_obstacle_threshold_{0.65};
  bool dynamic_enabled_{true};
  double dynamic_hit_gain_{0.35};
  double dynamic_free_decay_{0.55};
  double dynamic_time_decay_{0.85};
  double dynamic_keep_time_{3.0};
  double dynamic_min_height_risk_{0.35};
  double dynamic_risk_threshold_{0.65};
  double dynamic_decay_time_{0.8};
  bool publish_only_blocking_points_{false};
  double blocking_intensity_threshold_{0.15};
  bool publish_debug_maps_{true};
  bool publish_debug_clouds_{true};

  bool have_odom_{false};
  double robot_x_{0.0};
  double robot_y_{0.0};
  double robot_z_{0.0};
  builtin_interfaces::msg::Time latest_odom_stamp_;
  double last_publish_time_{-std::numeric_limits<double>::infinity()};

  grid_map::GridMap map_;
  std::unordered_map<std::uint64_t, DynamicCell> dynamic_cache_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr terrain_pub_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr elevation_pub_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr traversability_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr elevation_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr traversability_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr terrain_risk_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr dynamic_obstacle_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr terrain_height_cloud_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GridMapTerrainNode>());
  rclcpp::shutdown();
  return 0;
}
