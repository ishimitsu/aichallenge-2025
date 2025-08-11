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
private:
  // Configuration constants
  static constexpr int ROS_QUEUE_SIZE = 10;
  static constexpr double IMU_HISTORY_MAX_AGE_SEC = 2.0;
  static constexpr double IMU_DATA_VALID_AGE_SEC = 0.1;
  
  // GNSS quality factors
  static constexpr double QUALITY_NO_FIX = 0.1;
  static constexpr double QUALITY_BASIC_FIX = 0.7;
  static constexpr double QUALITY_SBAS_FIX = 0.9;
  static constexpr double QUALITY_GBAS_FIX = 1.0;
  static constexpr double QUALITY_DEFAULT = 1.0;
  static constexpr double QUALITY_MIN_LIMIT = 0.01;
  static constexpr double QUALITY_MAX_LIMIT = 1.0;
  
  // Covariance calculation constants  
  static constexpr double BASE_POSITION_VARIANCE = 0.25;
  static constexpr double BASE_ROTATION_VARIANCE = 0.1;
  static constexpr double Z_AXIS_VARIANCE_MULTIPLIER = 2.0;
  static constexpr double IMU_FUSION_UNCERTAINTY_MULTIPLIER = 2.0;
  static constexpr double COVARIANCE_EXPONENTIAL_DECAY_FACTOR = 0.1;
  
  // Covariance matrix indices
  static constexpr int COV_XX = 0;   // x position variance
  static constexpr int COV_YY = 7;   // y position variance  
  static constexpr int COV_ZZ = 14;  // z position variance
  static constexpr int COV_ROLL = 21;  // roll variance
  static constexpr int COV_PITCH = 28; // pitch variance
  static constexpr int COV_YAW = 35;   // yaw variance

public:
  GnssImuPoser() : Node("gnss_imu_poser"), is_initialized_(false)
  {
    // Publishers
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/localization/imu_gnss_poser/pose_with_covariance", ROS_QUEUE_SIZE);
    
    // Subscribers
    gnss_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/sensing/gnss/pose", ROS_QUEUE_SIZE, 
      std::bind(&GnssImuPoser::gnss_pose_callback, this, std::placeholders::_1));
    
    gnss_fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/sensing/gnss/nav_sat_fix", ROS_QUEUE_SIZE,
      std::bind(&GnssImuPoser::gnss_fix_callback, this, std::placeholders::_1));
    
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/sensing/imu/imu_data", ROS_QUEUE_SIZE,
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
    
    // Keep only recent IMU data
    auto current_time = this->now();
    while (!imu_history_.empty()) {
      auto age = current_time - rclcpp::Time(imu_history_.front().header.stamp);
      if (age.seconds() > IMU_HISTORY_MAX_AGE_SEC) {
        imu_history_.pop_front();
      } else {
        break;
      }
    }
  }
  
  // Quality evaluation function (Phase 1 implementation)
  double evaluate_gnss_quality()
  {
    double quality_factor = QUALITY_DEFAULT;  // Default quality
    
    switch (latest_gnss_fix_.status.status) {
      case sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX:
        quality_factor = QUALITY_NO_FIX;  // Very low quality
        break;
      case sensor_msgs::msg::NavSatStatus::STATUS_FIX:
        quality_factor = QUALITY_BASIC_FIX;  // Medium quality
        break;
      case sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX:
        quality_factor = QUALITY_SBAS_FIX;  // High quality
        break;
      case sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX:
        quality_factor = QUALITY_GBAS_FIX;  // Excellent quality
        break;
      default:
        quality_factor = QUALITY_DEFAULT;  // Default to highest quality for unknown status
        break;
    }
    
    // Consider position covariance if available
    if (latest_gnss_fix_.position_covariance_type == sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN) {
      double pos_std = std::sqrt(latest_gnss_fix_.position_covariance[COV_XX] + 
                                latest_gnss_fix_.position_covariance[COV_YY]);
      quality_factor *= std::exp(-pos_std * COVARIANCE_EXPONENTIAL_DECAY_FACTOR);  // Exponential decay based on std
    }
    
    return std::max(QUALITY_MIN_LIMIT, std::min(QUALITY_MAX_LIMIT, quality_factor));
  }
  
  // Dynamic covariance calculation
  std::array<double, 36> calculate_dynamic_covariance(double quality_factor)
  {
    std::array<double, 36> covariance;
    std::fill(covariance.begin(), covariance.end(), 0.0);
    
    // Base covariance values inversely related to quality
    double pos_var = (1.0 / quality_factor) * BASE_POSITION_VARIANCE;  // Position variance
    double rot_var = (1.0 / quality_factor) * BASE_ROTATION_VARIANCE;   // Rotation variance
    
    // Set diagonal elements
    covariance[COV_XX] = pos_var;   // x variance
    covariance[COV_YY] = pos_var;   // y variance  
    covariance[COV_ZZ] = pos_var * Z_AXIS_VARIANCE_MULTIPLIER;  // z variance (usually worse)
    covariance[COV_ROLL] = rot_var;  // roll variance
    covariance[COV_PITCH] = rot_var;  // pitch variance
    covariance[COV_YAW] = rot_var;  // yaw variance
    
    return covariance;
  }
  
  bool has_valid_imu_data()
  {
    if (imu_history_.empty()) return false;
    
    auto current_time = this->now();
    auto latest_imu_time = rclcpp::Time(latest_imu_.header.stamp);
    auto age = current_time - latest_imu_time;
    
    return age.seconds() < IMU_DATA_VALID_AGE_SEC;  // IMU data should be recent (within 100ms)
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
      pose_msg->pose.covariance[COV_ROLL] *= IMU_FUSION_UNCERTAINTY_MULTIPLIER;  // roll
      pose_msg->pose.covariance[COV_PITCH] *= IMU_FUSION_UNCERTAINTY_MULTIPLIER;  // pitch  
      pose_msg->pose.covariance[COV_YAW] *= IMU_FUSION_UNCERTAINTY_MULTIPLIER;  // yaw
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
