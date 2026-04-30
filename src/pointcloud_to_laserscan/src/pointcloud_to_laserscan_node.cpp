/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

/*
 * Author: Paul Bovbel
 */

#include "pointcloud_to_laserscan/pointcloud_to_laserscan_node.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <algorithm>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "tf2_ros/create_timer_ros.h"

namespace pointcloud_to_laserscan
{

PointCloudToLaserScanNode::PointCloudToLaserScanNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("pointcloud_to_laserscan", options)
{
  target_frame_ = this->declare_parameter("target_frame", "");
  tolerance_ = this->declare_parameter("transform_tolerance", 0.01);
  // TODO(hidmic): adjust default input queue size based on actual concurrency levels
  // achievable by the associated executor
  input_queue_size_ = this->declare_parameter(
    "queue_size", static_cast<int>(std::thread::hardware_concurrency()));
  min_height_ = this->declare_parameter("min_height", std::numeric_limits<double>::min());
  max_height_ = this->declare_parameter("max_height", std::numeric_limits<double>::max());
  angle_min_ = this->declare_parameter("angle_min", -M_PI);
  angle_max_ = this->declare_parameter("angle_max", M_PI);
  angle_increment_ = this->declare_parameter("angle_increment", M_PI / 180.0);
  scan_time_ = this->declare_parameter("scan_time", 1.0 / 30.0);
  range_min_ = this->declare_parameter("range_min", 0.0);
  range_max_ = this->declare_parameter("range_max", std::numeric_limits<double>::max());
  adaptive_ground_filter_ = this->declare_parameter("adaptive_ground_filter", false);
  ground_percentile_ = this->declare_parameter("ground_percentile", 45.0);
  ground_margin_ = this->declare_parameter("ground_margin", 0.08);
  ground_estimation_min_range_ = this->declare_parameter("ground_estimation_min_range", 0.3);
  ground_estimation_max_range_ = this->declare_parameter("ground_estimation_max_range", 2.5);
  ground_min_samples_ = this->declare_parameter("ground_min_samples", 100);
  terrain_ground_filter_ = this->declare_parameter("terrain_ground_filter", false);
  terrain_planar_size_ = this->declare_parameter("terrain_planar_size", 0.2);
  terrain_planar_width_ = this->declare_parameter("terrain_planar_width", 51);
  terrain_quantile_z_ = this->declare_parameter("terrain_quantile_z", 0.25);
  terrain_use_sorting_ = this->declare_parameter("terrain_use_sorting", true);
  terrain_limit_ground_lift_ = this->declare_parameter("terrain_limit_ground_lift", true);
  terrain_max_ground_lift_ = this->declare_parameter("terrain_max_ground_lift", 0.18);
  terrain_min_rel_z_ = this->declare_parameter("terrain_min_rel_z", -1.5);
  terrain_max_rel_z_ = this->declare_parameter("terrain_max_rel_z", 0.35);
  terrain_dis_ratio_z_ = this->declare_parameter("terrain_dis_ratio_z", 0.2);
  terrain_vehicle_height_ = this->declare_parameter("terrain_vehicle_height", 1.2);
  terrain_min_ground_points_ = this->declare_parameter("terrain_min_ground_points", 4);
  terrain_min_block_points_ = this->declare_parameter("terrain_min_block_points", 8);
  terrain_min_obstacle_points_ = this->declare_parameter("terrain_min_obstacle_points", 4);
  terrain_neighbor_spread_ = this->declare_parameter("terrain_neighbor_spread", true);
  terrain_min_obstacle_rel_z_ = this->declare_parameter("terrain_min_obstacle_rel_z", 0.08);
  terrain_obstacle_range_slope_ = this->declare_parameter("terrain_obstacle_range_slope", 0.0);
  terrain_consider_drop_ = this->declare_parameter("terrain_consider_drop", false);
  publish_filtered_cloud_ = this->declare_parameter("publish_filtered_cloud", false);
  inf_epsilon_ = this->declare_parameter("inf_epsilon", 1.0);
  use_inf_ = this->declare_parameter("use_inf", true);
  override_stamp_to_now_ = this->declare_parameter("override_stamp_to_now", false);
  auto output_reliability = this->declare_parameter("output_reliability", "reliable");

  if (ground_percentile_ < 0.0) {
    ground_percentile_ = 0.0;
  }
  if (ground_percentile_ > 100.0) {
    ground_percentile_ = 100.0;
  }
  if (ground_min_samples_ < 1) {
    ground_min_samples_ = 1;
  }
  if (terrain_planar_size_ <= 0.0) {
    terrain_planar_size_ = 0.2;
  }
  if (terrain_planar_width_ < 3) {
    terrain_planar_width_ = 3;
  }
  if (terrain_planar_width_ % 2 == 0) {
    terrain_planar_width_ += 1;
  }
  if (terrain_quantile_z_ < 0.0) {
    terrain_quantile_z_ = 0.0;
  }
  if (terrain_quantile_z_ > 1.0) {
    terrain_quantile_z_ = 1.0;
  }
  if (terrain_max_ground_lift_ < 0.0) {
    terrain_max_ground_lift_ = 0.0;
  }
  if (terrain_vehicle_height_ <= 0.0) {
    terrain_vehicle_height_ = 1.2;
  }
  if (terrain_min_ground_points_ < 1) {
    terrain_min_ground_points_ = 1;
  }
  if (terrain_min_block_points_ < 1) {
    terrain_min_block_points_ = 1;
  }
  if (terrain_min_obstacle_points_ < 1) {
    terrain_min_obstacle_points_ = 1;
  }
  if (terrain_min_obstacle_rel_z_ < 0.0) {
    terrain_min_obstacle_rel_z_ = 0.0;
  }
  if (terrain_obstacle_range_slope_ < 0.0) {
    terrain_obstacle_range_slope_ = 0.0;
  }

  rclcpp::QoS scan_qos(rclcpp::KeepLast(10));
  if (output_reliability == "best_effort") {
    scan_qos.best_effort();
  } else {
    scan_qos.reliable();
    output_reliability = "reliable";
  }
  scan_qos.durability_volatile();

  pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", scan_qos);
  if (publish_filtered_cloud_) {
    filtered_cloud_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_cloud", scan_qos);
  }
  RCLCPP_INFO(
    this->get_logger(),
    "Publishing scan with %s QoS reliability",
    output_reliability.c_str());

  using std::placeholders::_1;
  // if pointcloud target frame specified, we need to filter by transform availability
  if (!target_frame_.empty()) {
    tf2_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
    tf2_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_);
    message_filter_ = std::make_unique<MessageFilter>(
      sub_, *tf2_, target_frame_, input_queue_size_,
      this->get_node_logging_interface(),
      this->get_node_clock_interface());
    message_filter_->registerCallback(
      std::bind(&PointCloudToLaserScanNode::cloudCallback, this, _1));
  } else {  // otherwise setup direct subscription
    sub_.registerCallback(std::bind(&PointCloudToLaserScanNode::cloudCallback, this, _1));
  }

  subscription_listener_thread_ = std::thread(
    std::bind(&PointCloudToLaserScanNode::subscriptionListenerThreadLoop, this));
}

PointCloudToLaserScanNode::~PointCloudToLaserScanNode()
{
  alive_.store(false);
  subscription_listener_thread_.join();
}

void PointCloudToLaserScanNode::subscriptionListenerThreadLoop()
{
  rclcpp::Context::SharedPtr context = this->get_node_base_interface()->get_context();

  const std::chrono::milliseconds timeout(100);
  while (rclcpp::ok(context) && alive_.load()) {
    int subscription_count = pub_->get_subscription_count() +
      pub_->get_intra_process_subscription_count();
    if (filtered_cloud_pub_) {
      subscription_count += filtered_cloud_pub_->get_subscription_count() +
        filtered_cloud_pub_->get_intra_process_subscription_count();
    }
    if (subscription_count > 0) {
      if (!sub_.getSubscriber()) {
        RCLCPP_INFO(
          this->get_logger(),
          "Got a subscriber to laserscan, starting pointcloud subscriber");
        auto qos = rmw_qos_profile_sensor_data;
        qos.depth = input_queue_size_ > 0 ? static_cast<size_t>(input_queue_size_) : 10;
        sub_.subscribe(this, "cloud_in", qos);
      }
    } else if (sub_.getSubscriber()) {
      RCLCPP_INFO(
        this->get_logger(),
        "No subscribers to laserscan, shutting down pointcloud subscriber");
      sub_.unsubscribe();
    }
    rclcpp::Event::SharedPtr event = this->get_graph_event();
    this->wait_for_graph_change(event, timeout);
  }
  sub_.unsubscribe();
}

void PointCloudToLaserScanNode::cloudCallback(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg)
{
  // build laserscan output
  auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
  scan_msg->header = cloud_msg->header;
  if (!target_frame_.empty()) {
    scan_msg->header.frame_id = target_frame_;
  }
  if (override_stamp_to_now_) {
    scan_msg->header.stamp = this->now();
  }

  scan_msg->angle_min = angle_min_;
  scan_msg->angle_max = angle_max_;
  scan_msg->angle_increment = angle_increment_;
  scan_msg->time_increment = 0.0;
  scan_msg->scan_time = scan_time_;
  scan_msg->range_min = range_min_;
  scan_msg->range_max = range_max_;

  // determine amount of rays to create
  uint32_t ranges_size = std::ceil(
    (scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);

  // determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
  if (use_inf_) {
    scan_msg->ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
  } else {
    scan_msg->ranges.assign(ranges_size, scan_msg->range_max + inf_epsilon_);
  }

  // Transform cloud if necessary
  if (scan_msg->header.frame_id != cloud_msg->header.frame_id) {
    try {
      auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
      tf2_->transform(*cloud_msg, *cloud, target_frame_, tf2::durationFromSec(tolerance_));
      cloud_msg = cloud;
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Transform failure: " << ex.what());
      return;
    }
  }

  struct AcceptedPoint
  {
    float x;
    float y;
    float z;
  };
  std::vector<AcceptedPoint> accepted_points;
  if (publish_filtered_cloud_) {
    accepted_points.reserve(static_cast<size_t>(cloud_msg->width) * static_cast<size_t>(cloud_msg->height) / 4 + 1);
  }

  if (terrain_ground_filter_) {
    const int width = terrain_planar_width_;
    const int half_width = width / 2;
    const int cell_num = width * width;
    const double half_voxel = terrain_planar_size_ * 0.5;
    const double processing_range = terrain_planar_size_ * static_cast<double>(half_width + 1);

    std::vector<std::vector<float>> planar_point_elev(static_cast<size_t>(cell_num));
    std::vector<int> planar_point_count(static_cast<size_t>(cell_num), 0);
    std::vector<float> planar_ground(static_cast<size_t>(cell_num),
      std::numeric_limits<float>::quiet_NaN());
    std::vector<int> obstacle_point_count(static_cast<size_t>(cell_num), 0);
    std::vector<double> obstacle_best_range(
      static_cast<size_t>(cell_num), std::numeric_limits<double>::infinity());
    std::vector<AcceptedPoint> obstacle_best_point(static_cast<size_t>(cell_num), AcceptedPoint{0.0f, 0.0f, 0.0f});

    auto to_cell_index = [&](double x, double y) -> int {
      int ind_x = static_cast<int>(std::floor((x + half_voxel) / terrain_planar_size_)) + half_width;
      int ind_y = static_cast<int>(std::floor((y + half_voxel) / terrain_planar_size_)) + half_width;
      if (ind_x < 0 || ind_x >= width || ind_y < 0 || ind_y >= width) {
        return -1;
      }
      return width * ind_x + ind_y;
    };

    // Build local planar elevation statistics (quantile-based ground estimation).
    for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x"),
      iter_y(*cloud_msg, "y"), iter_z(*cloud_msg, "z");
      iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
      if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) {
        continue;
      }

      const double range = hypot(*iter_x, *iter_y);
      if (!std::isfinite(range) || range > processing_range) {
        continue;
      }

      const double local_min_rel_z = terrain_min_rel_z_ - terrain_dis_ratio_z_ * range;
      const double local_max_rel_z = terrain_max_rel_z_ + terrain_dis_ratio_z_ * range;
      if (*iter_z < local_min_rel_z || *iter_z > local_max_rel_z) {
        continue;
      }

      const int cell_idx = to_cell_index(*iter_x, *iter_y);
      if (cell_idx < 0) {
        continue;
      }

      planar_point_count[static_cast<size_t>(cell_idx)]++;
      if (terrain_neighbor_spread_) {
        const int ind_x = cell_idx / width;
        const int ind_y = cell_idx % width;
        for (int dx = -1; dx <= 1; ++dx) {
          for (int dy = -1; dy <= 1; ++dy) {
            const int nx = ind_x + dx;
            const int ny = ind_y + dy;
            if (nx >= 0 && nx < width && ny >= 0 && ny < width) {
              planar_point_elev[static_cast<size_t>(nx * width + ny)].push_back(*iter_z);
            }
          }
        }
      } else {
        planar_point_elev[static_cast<size_t>(cell_idx)].push_back(*iter_z);
      }
    }

    for (int i = 0; i < cell_num; ++i) {
      auto & elev = planar_point_elev[static_cast<size_t>(i)];
      if (elev.size() < static_cast<size_t>(terrain_min_ground_points_)) {
        continue;
      }

      float ground_z = 0.0f;
      if (terrain_use_sorting_) {
        std::sort(elev.begin(), elev.end());
        size_t quantile_id = static_cast<size_t>(
          std::round(terrain_quantile_z_ * static_cast<double>(elev.size() - 1)));
        if (quantile_id >= elev.size()) {
          quantile_id = elev.size() - 1;
        }
        ground_z = elev[quantile_id];
        if (terrain_limit_ground_lift_) {
          const float min_z = elev.front();
          if (ground_z > min_z + static_cast<float>(terrain_max_ground_lift_)) {
            ground_z = min_z + static_cast<float>(terrain_max_ground_lift_);
          }
        }
      } else {
        const auto min_it = std::min_element(elev.begin(), elev.end());
        ground_z = (min_it != elev.end()) ? *min_it : 0.0f;
      }

      planar_ground[static_cast<size_t>(i)] = ground_z;
    }

    // Classify obstacle points using relative elevation above local quantile ground.
    for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x"),
      iter_y(*cloud_msg, "y"), iter_z(*cloud_msg, "z");
      iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
      if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) {
        continue;
      }

      const double range = hypot(*iter_x, *iter_y);
      if (!std::isfinite(range) || range < range_min_ || range > range_max_) {
        continue;
      }

      const int cell_idx = to_cell_index(*iter_x, *iter_y);
      if (cell_idx < 0) {
        continue;
      }
      if (planar_point_count[static_cast<size_t>(cell_idx)] < terrain_min_block_points_) {
        continue;
      }

      const float ground_z = planar_ground[static_cast<size_t>(cell_idx)];
      if (!std::isfinite(ground_z)) {
        continue;
      }

      double rel_z = static_cast<double>(*iter_z - ground_z);
      if (terrain_consider_drop_) {
        rel_z = std::fabs(rel_z);
      }
      const double obstacle_rel_z_threshold =
        terrain_min_obstacle_rel_z_ + terrain_obstacle_range_slope_ * range;
      if (rel_z < obstacle_rel_z_threshold || rel_z > terrain_vehicle_height_) {
        continue;
      }

      obstacle_point_count[static_cast<size_t>(cell_idx)]++;
      if (range < obstacle_best_range[static_cast<size_t>(cell_idx)]) {
        obstacle_best_range[static_cast<size_t>(cell_idx)] = range;
        obstacle_best_point[static_cast<size_t>(cell_idx)] = AcceptedPoint{*iter_x, *iter_y, *iter_z};
      }
    }

    // Only keep cells with a stable cluster of obstacle points to suppress sparse floor speckle.
    for (int cell_idx = 0; cell_idx < cell_num; ++cell_idx) {
      if (obstacle_point_count[static_cast<size_t>(cell_idx)] < terrain_min_obstacle_points_) {
        continue;
      }

      const auto & point = obstacle_best_point[static_cast<size_t>(cell_idx)];
      const double range = obstacle_best_range[static_cast<size_t>(cell_idx)];
      if (!std::isfinite(range)) {
        continue;
      }

      const double angle = atan2(point.y, point.x);
      if (angle < scan_msg->angle_min || angle > scan_msg->angle_max) {
        continue;
      }

      const int index = static_cast<int>((angle - scan_msg->angle_min) / scan_msg->angle_increment);
      if (index < 0 || static_cast<size_t>(index) >= scan_msg->ranges.size()) {
        continue;
      }
      if (range < scan_msg->ranges[static_cast<size_t>(index)]) {
        scan_msg->ranges[static_cast<size_t>(index)] = range;
      }
      if (publish_filtered_cloud_) {
        accepted_points.push_back(point);
      }
    }
  } else {
    double effective_min_height = min_height_;
    if (adaptive_ground_filter_) {
      std::vector<float> z_candidates;
      z_candidates.reserve(
        static_cast<size_t>(cloud_msg->width) * static_cast<size_t>(cloud_msg->height) / 4 + 1);

      for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x"),
        iter_y(*cloud_msg, "y"), iter_z(*cloud_msg, "z");
        iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
      {
        if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z)) {
          continue;
        }

        const double range = hypot(*iter_x, *iter_y);
        if (!std::isfinite(range)) {
          continue;
        }
        if (range < ground_estimation_min_range_ || range > ground_estimation_max_range_) {
          continue;
        }

        z_candidates.emplace_back(*iter_z);
      }

      if (z_candidates.size() >= static_cast<size_t>(ground_min_samples_)) {
        const size_t idx = static_cast<size_t>(
          std::round((ground_percentile_ / 100.0) * static_cast<double>(z_candidates.size() - 1)));
        std::nth_element(z_candidates.begin(), z_candidates.begin() + idx, z_candidates.end());
        const double ground_ref = static_cast<double>(z_candidates[idx]);
        const double adaptive_min = ground_ref + ground_margin_;
        if (adaptive_min > effective_min_height) {
          effective_min_height = adaptive_min;
        }
        if (effective_min_height > max_height_) {
          effective_min_height = max_height_;
        }
      } else {
        RCLCPP_DEBUG(
          this->get_logger(),
          "adaptive_ground_filter: insufficient points (%zu < %d), using fixed min_height=%f",
          z_candidates.size(), ground_min_samples_, min_height_);
      }
    }

    for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x"),
      iter_y(*cloud_msg, "y"), iter_z(*cloud_msg, "z");
      iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
      if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) {
        continue;
      }

      if (*iter_z > max_height_ || *iter_z < effective_min_height) {
        continue;
      }

      const double range = hypot(*iter_x, *iter_y);
      if (range < range_min_ || range > range_max_) {
        continue;
      }

      const double angle = atan2(*iter_y, *iter_x);
      if (angle < scan_msg->angle_min || angle > scan_msg->angle_max) {
        continue;
      }

      const int index = static_cast<int>((angle - scan_msg->angle_min) / scan_msg->angle_increment);
      if (index < 0 || static_cast<size_t>(index) >= scan_msg->ranges.size()) {
        continue;
      }
      if (range < scan_msg->ranges[static_cast<size_t>(index)]) {
        scan_msg->ranges[static_cast<size_t>(index)] = range;
      }
      if (publish_filtered_cloud_) {
        accepted_points.push_back(AcceptedPoint{*iter_x, *iter_y, *iter_z});
      }
    }
  }

  if (filtered_cloud_pub_) {
    sensor_msgs::msg::PointCloud2 filtered_cloud_msg;
    filtered_cloud_msg.header = scan_msg->header;

    sensor_msgs::PointCloud2Modifier modifier(filtered_cloud_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(accepted_points.size());

    sensor_msgs::PointCloud2Iterator<float> out_x(filtered_cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(filtered_cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(filtered_cloud_msg, "z");

    for (const auto & point : accepted_points) {
      *out_x = point.x;
      *out_y = point.y;
      *out_z = point.z;
      ++out_x;
      ++out_y;
      ++out_z;
    }

    filtered_cloud_pub_->publish(filtered_cloud_msg);
  }
  pub_->publish(std::move(scan_msg));
}

}  // namespace pointcloud_to_laserscan

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_to_laserscan::PointCloudToLaserScanNode)
