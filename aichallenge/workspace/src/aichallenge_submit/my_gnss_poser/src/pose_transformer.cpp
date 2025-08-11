// Copyright 2024 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <vector>
#include <deque>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nav_msgs/msg/odometry.hpp"

class GnssImuPoser : public rclcpp::Node
{
public:
  GnssImuPoser() : Node("gnss_imu_poser"), is_initialized_(false)
  {
    // Publishers
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/localization/imu_gnss_poser/pose_with_covariance", 10);
    
    // Subscribers
    gnss_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/sensing/gnss/pose", 10, 
      std::bind(&GnssImuPoser::gnss_pose_callback, this, std::placeholders::_1));
    
    gnss_fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/sensing/gnss/nav_sat_fix", 10,
      std::bind(&GnssImuPoser::gnss_fix_callback, this, std::placeholders::_1));
    
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/sensing/imu/imu_data", 10,
      std::bind(&GnssImuPoser::imu_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "GNSS-IMU Poser initialized");
  }

private:
  // Main processing callback
  void gnss_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // Store the latest GNSS data
    latest_gnss_pose_ = *msg;
    
    // Evaluate GNSS quality
    double quality_factor = evaluate_gnss_quality();
    
    // Calculate dynamic covariance based on quality
    auto covariance = calculate_dynamic_covariance(quality_factor);
    
    // Create output message
    auto output_msg = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
    output_msg->header = msg->header;
    output_msg->pose.pose = msg->pose;
    output_msg->pose.covariance = covariance;
    
    // Apply IMU fusion if available
    if (has_valid_imu_data()) {
      apply_imu_fusion(output_msg);
    }
    
    pose_pub_->publish(*output_msg);
    is_initialized_ = true;
  }
  
  void gnss_fix_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    latest_gnss_fix_ = *msg;
  }
  
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    latest_imu_ = *msg;
    imu_history_.push_back(*msg);
    
    // Keep only recent IMU data (last 2 seconds)
    auto current_time = this->now();
    while (!imu_history_.empty()) {
      auto age = current_time - rclcpp::Time(imu_history_.front().header.stamp);
      if (age.seconds() > 2.0) {
        imu_history_.pop_front();
      } else {
        break;
      }
    }
  }
  
  // Quality evaluation function (Phase 1 implementation)
  double evaluate_gnss_quality()
  {
    double quality_factor = 1.0;  // Default quality
    
    switch (latest_gnss_fix_.status.status) {
      case sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX:
        quality_factor = 0.1;  // Very low quality
        break;
      case sensor_msgs::msg::NavSatStatus::STATUS_FIX:
        quality_factor = 0.7;  // Medium quality
        break;
      case sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX:
        quality_factor = 0.9;  // High quality
        break;
      case sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX:
        quality_factor = 1.0;  // Excellent quality
        break;
      default:
        quality_factor = 1.0;  // Default to highest quality for unknown status
        break;
    }
    
    // Consider position covariance if available
    if (latest_gnss_fix_.position_covariance_type == sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN) {
      double pos_std = std::sqrt(latest_gnss_fix_.position_covariance[0] + 
                                latest_gnss_fix_.position_covariance[4]);
      quality_factor *= std::exp(-pos_std * 0.1);  // Exponential decay based on std
    }
    
    return std::max(0.01, std::min(1.0, quality_factor));
  }
  
  // Dynamic covariance calculation
  std::array<double, 36> calculate_dynamic_covariance(double quality_factor)
  {
    std::array<double, 36> covariance;
    std::fill(covariance.begin(), covariance.end(), 0.0);
    
    // Base covariance values inversely related to quality
    double pos_var = (1.0 / quality_factor) * 0.25;  // Position variance
    double rot_var = (1.0 / quality_factor) * 0.1;   // Rotation variance
    
    // Set diagonal elements
    covariance[0] = pos_var;   // x variance
    covariance[7] = pos_var;   // y variance  
    covariance[14] = pos_var * 2.0;  // z variance (usually worse)
    covariance[21] = rot_var;  // roll variance
    covariance[28] = rot_var;  // pitch variance
    covariance[35] = rot_var;  // yaw variance
    
    return covariance;
  }
  
  bool has_valid_imu_data()
  {
    if (imu_history_.empty()) return false;
    
    auto current_time = this->now();
    auto latest_imu_time = rclcpp::Time(latest_imu_.header.stamp);
    auto age = current_time - latest_imu_time;
    
    return age.seconds() < 0.1;  // IMU data should be recent (within 100ms)
  }
  
  void apply_imu_fusion(std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped> pose_msg)
  {
    // Simple IMU fusion: use IMU orientation if GNSS orientation is invalid
    auto& orientation = pose_msg->pose.pose.orientation;
    
    if (std::isnan(orientation.x) || std::isnan(orientation.y) || 
        std::isnan(orientation.z) || std::isnan(orientation.w) ||
        (orientation.x == 0 && orientation.y == 0 && 
         orientation.z == 0 && orientation.w == 0)) {
      
      orientation = latest_imu_.orientation;
      
      // Increase orientation uncertainty when using IMU
      pose_msg->pose.covariance[21] *= 2.0;  // roll
      pose_msg->pose.covariance[28] *= 2.0;  // pitch  
      pose_msg->pose.covariance[35] *= 2.0;  // yaw
    }
  }
  
  // Member variables
  bool is_initialized_;
  
  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  
  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gnss_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_fix_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  
  // Data storage
  geometry_msgs::msg::PoseStamped latest_gnss_pose_;
  sensor_msgs::msg::NavSatFix latest_gnss_fix_;
  sensor_msgs::msg::Imu latest_imu_;
  std::deque<sensor_msgs::msg::Imu> imu_history_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GnssImuPoser>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
