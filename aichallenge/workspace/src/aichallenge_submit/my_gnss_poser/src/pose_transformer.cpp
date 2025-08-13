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
  // Configuration parameters (loaded from YAML at startup)
  int ros_queue_size_;
  double imu_history_max_age_sec_;
  double imu_data_valid_age_sec_;
  
  // Outlier detection parameters
  int gnss_history_max_size_;
  int min_samples_for_outlier_detection_;
  double outlier_z_score_threshold_;
  double max_position_jump_m_;
  double max_velocity_mps_;
  double time_consistency_threshold_sec_;
  
  // GNSS quality factors
  double quality_no_fix_;
  double quality_basic_fix_;
  double quality_sbas_fix_;
  double quality_gbas_fix_;
  double quality_default_;
  double quality_min_limit_;
  double quality_max_limit_;
  
  // Covariance calculation parameters
  double base_position_variance_;
  double base_rotation_variance_;
  double z_axis_variance_multiplier_;
  double imu_fusion_uncertainty_multiplier_;
  double covariance_exponential_decay_factor_;
  
  // Debug options
  bool enable_outlier_logging_;
  bool enable_quality_logging_;
  double log_statistics_interval_sec_;
  
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
    // Load parameters from YAML
    load_parameters();
    
    // Publishers
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/localization/imu_gnss_poser/pose_with_covariance", ros_queue_size_);
    
    // Subscribers
    gnss_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/sensing/gnss/pose", ros_queue_size_, 
      std::bind(&GnssImuPoser::gnss_pose_callback, this, std::placeholders::_1));
    
    gnss_fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/sensing/gnss/nav_sat_fix", ros_queue_size_,
      std::bind(&GnssImuPoser::gnss_fix_callback, this, std::placeholders::_1));
    
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/sensing/imu/imu_data", ros_queue_size_,
      std::bind(&GnssImuPoser::imu_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "GNSS-IMU Poser initialized with configurable parameters");
  }

  // Default parameter values (avoid magic numbers)
  static constexpr int DEFAULT_ROS_QUEUE_SIZE = 10;
  static constexpr double DEFAULT_IMU_HISTORY_MAX_AGE_SEC = 2.0;
  static constexpr double DEFAULT_IMU_DATA_VALID_AGE_SEC = 0.1;
  static constexpr int DEFAULT_GNSS_HISTORY_MAX_SIZE = 50;
  static constexpr int DEFAULT_MIN_SAMPLES_FOR_OUTLIER_DETECTION = 20;
  static constexpr double DEFAULT_OUTLIER_Z_SCORE_THRESHOLD = 5.0;
  static constexpr double DEFAULT_MAX_POSITION_JUMP_M = 100.0;
  static constexpr double DEFAULT_MAX_VELOCITY_MPS = 100.0;
  static constexpr double DEFAULT_TIME_CONSISTENCY_THRESHOLD_SEC = 5.0;
  static constexpr double DEFAULT_QUALITY_NO_FIX = 0.1;
  static constexpr double DEFAULT_QUALITY_BASIC_FIX = 0.7;
  static constexpr double DEFAULT_QUALITY_SBAS_FIX = 0.9;
  static constexpr double DEFAULT_QUALITY_GBAS_FIX = 1.0;
  static constexpr double DEFAULT_QUALITY_DEFAULT = 1.0;
  static constexpr double DEFAULT_QUALITY_MIN_LIMIT = 0.01;
  static constexpr double DEFAULT_QUALITY_MAX_LIMIT = 1.0;
  static constexpr double DEFAULT_BASE_POSITION_VARIANCE = 0.25;
  static constexpr double DEFAULT_BASE_ROTATION_VARIANCE = 0.1;
  static constexpr double DEFAULT_Z_AXIS_VARIANCE_MULTIPLIER = 2.0;
  static constexpr double DEFAULT_IMU_FUSION_UNCERTAINTY_MULTIPLIER = 2.0;
  static constexpr double DEFAULT_COVARIANCE_EXPONENTIAL_DECAY_FACTOR = 0.1;
  static constexpr bool DEFAULT_ENABLE_OUTLIER_LOGGING = true;
  static constexpr bool DEFAULT_ENABLE_QUALITY_LOGGING = false;
  static constexpr double DEFAULT_LOG_STATISTICS_INTERVAL_SEC = 10.0;

  // Parameter loading function
  void load_parameters()
  {
    // Basic configuration parameters
    ros_queue_size_ = this->declare_parameter("ros_queue_size", DEFAULT_ROS_QUEUE_SIZE);
    imu_history_max_age_sec_ = this->declare_parameter("imu_history_max_age_sec", DEFAULT_IMU_HISTORY_MAX_AGE_SEC);
    imu_data_valid_age_sec_ = this->declare_parameter("imu_data_valid_age_sec", DEFAULT_IMU_DATA_VALID_AGE_SEC);
    
    // Outlier detection parameters
    gnss_history_max_size_ = this->declare_parameter("gnss_history_max_size", DEFAULT_GNSS_HISTORY_MAX_SIZE);
    min_samples_for_outlier_detection_ = this->declare_parameter("min_samples_for_outlier_detection", DEFAULT_MIN_SAMPLES_FOR_OUTLIER_DETECTION);
    outlier_z_score_threshold_ = this->declare_parameter("outlier_z_score_threshold", DEFAULT_OUTLIER_Z_SCORE_THRESHOLD);
    max_position_jump_m_ = this->declare_parameter("max_position_jump_m", DEFAULT_MAX_POSITION_JUMP_M);
    max_velocity_mps_ = this->declare_parameter("max_velocity_mps", DEFAULT_MAX_VELOCITY_MPS);
    time_consistency_threshold_sec_ = this->declare_parameter("time_consistency_threshold_sec", DEFAULT_TIME_CONSISTENCY_THRESHOLD_SEC);
    
    // GNSS quality factors
    quality_no_fix_ = this->declare_parameter("quality_no_fix", DEFAULT_QUALITY_NO_FIX);
    quality_basic_fix_ = this->declare_parameter("quality_basic_fix", DEFAULT_QUALITY_BASIC_FIX);
    quality_sbas_fix_ = this->declare_parameter("quality_sbas_fix", DEFAULT_QUALITY_SBAS_FIX);
    quality_gbas_fix_ = this->declare_parameter("quality_gbas_fix", DEFAULT_QUALITY_GBAS_FIX);
    quality_default_ = this->declare_parameter("quality_default", DEFAULT_QUALITY_DEFAULT);
    quality_min_limit_ = this->declare_parameter("quality_min_limit", DEFAULT_QUALITY_MIN_LIMIT);
    quality_max_limit_ = this->declare_parameter("quality_max_limit", DEFAULT_QUALITY_MAX_LIMIT);
    
    // Covariance calculation parameters
    base_position_variance_ = this->declare_parameter("base_position_variance", DEFAULT_BASE_POSITION_VARIANCE);
    base_rotation_variance_ = this->declare_parameter("base_rotation_variance", DEFAULT_BASE_ROTATION_VARIANCE);
    z_axis_variance_multiplier_ = this->declare_parameter("z_axis_variance_multiplier", DEFAULT_Z_AXIS_VARIANCE_MULTIPLIER);
    imu_fusion_uncertainty_multiplier_ = this->declare_parameter("imu_fusion_uncertainty_multiplier", DEFAULT_IMU_FUSION_UNCERTAINTY_MULTIPLIER);
    covariance_exponential_decay_factor_ = this->declare_parameter("covariance_exponential_decay_factor", DEFAULT_COVARIANCE_EXPONENTIAL_DECAY_FACTOR);
    
    // Debug options
    enable_outlier_logging_ = this->declare_parameter("enable_outlier_logging", DEFAULT_ENABLE_OUTLIER_LOGGING);
    enable_quality_logging_ = this->declare_parameter("enable_quality_logging", DEFAULT_ENABLE_QUALITY_LOGGING);
    log_statistics_interval_sec_ = this->declare_parameter("log_statistics_interval_sec", DEFAULT_LOG_STATISTICS_INTERVAL_SEC);
    
    RCLCPP_INFO(this->get_logger(), "Parameters loaded from YAML configuration");
  }

private:
  // Main processing callback
  void gnss_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // Apply outlier detection and rejection
    if (!is_valid_gnss_data(*msg)) {
      RCLCPP_WARN(this->get_logger(), "GNSS data rejected as outlier");
      return;
    }
    
    // Store the latest GNSS data
    latest_gnss_pose_ = *msg;
    update_gnss_history(*msg);
    
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
      if (age.seconds() > imu_history_max_age_sec_) {
        imu_history_.pop_front();
      } else {
        break;
      }
    }
  }
  
  // Quality evaluation function (Phase 1 implementation)
  double evaluate_gnss_quality()
  {
    double quality_factor = quality_default_;  // Default quality
    
    switch (latest_gnss_fix_.status.status) {
      case sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX:
        quality_factor = quality_no_fix_;  // Very low quality
        break;
      case sensor_msgs::msg::NavSatStatus::STATUS_FIX:
        quality_factor = quality_basic_fix_;  // Medium quality
        break;
      case sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX:
        quality_factor = quality_sbas_fix_;  // High quality
        break;
      case sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX:
        quality_factor = quality_gbas_fix_;  // Excellent quality
        break;
      default:
        quality_factor = quality_default_;  // Default to highest quality for unknown status
        break;
    }
    
    // Consider position covariance if available
    if (latest_gnss_fix_.position_covariance_type == sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN) {
      double pos_std = std::sqrt(latest_gnss_fix_.position_covariance[COV_XX] + 
                                latest_gnss_fix_.position_covariance[COV_YY]);
      quality_factor *= std::exp(-pos_std * covariance_exponential_decay_factor_);  // Exponential decay based on std
    }
    
    return std::max(quality_min_limit_, std::min(quality_max_limit_, quality_factor));
  }
  
  // Dynamic covariance calculation
  std::array<double, 36> calculate_dynamic_covariance(double quality_factor)
  {
    std::array<double, 36> covariance;
    std::fill(covariance.begin(), covariance.end(), 0.0);
    
    // Base covariance values inversely related to quality
    double pos_var = (1.0 / quality_factor) * base_position_variance_;  // Position variance
    double rot_var = (1.0 / quality_factor) * base_rotation_variance_;   // Rotation variance
    
    // Set diagonal elements
    covariance[COV_XX] = pos_var;   // x variance
    covariance[COV_YY] = pos_var;   // y variance  
    covariance[COV_ZZ] = pos_var * z_axis_variance_multiplier_;  // z variance (usually worse)
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
    
    return age.seconds() < imu_data_valid_age_sec_;  // IMU data should be recent (within 100ms)
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
      pose_msg->pose.covariance[COV_ROLL] *= imu_fusion_uncertainty_multiplier_;  // roll
      pose_msg->pose.covariance[COV_PITCH] *= imu_fusion_uncertainty_multiplier_;  // pitch  
      pose_msg->pose.covariance[COV_YAW] *= imu_fusion_uncertainty_multiplier_;  // yaw
    }
  }
  
  // Outlier detection functions (Phase 1)
  bool is_valid_gnss_data(const geometry_msgs::msg::PoseStamped& pose)
  {
    // Check for position jump detection
    if (!gnss_position_history_.empty() && !check_position_continuity(pose)) {
      return false;
    }
    
    // Check time consistency
    if (!check_time_consistency(pose)) {
      return false;
    }
    
    // Check statistical outlier if enough history exists
    if (gnss_position_history_.size() >= min_samples_for_outlier_detection_) {
      if (!check_statistical_outlier(pose)) {
        return false;
      }
    }
    
    return true;
  }
  
  void update_gnss_history(const geometry_msgs::msg::PoseStamped& pose)
  {
    gnss_position_history_.push_back(pose);
    
    // Keep history size within limits
    while (gnss_position_history_.size() > gnss_history_max_size_) {
      gnss_position_history_.pop_front();
    }
  }
  
  bool check_position_continuity(const geometry_msgs::msg::PoseStamped& pose)
  {
    if (gnss_position_history_.empty()) return true;
    
    const auto& last_pose = gnss_position_history_.back();
    
    // Calculate 3D distance change
    double dx = pose.pose.position.x - last_pose.pose.position.x;
    double dy = pose.pose.position.y - last_pose.pose.position.y;
    double dz = pose.pose.position.z - last_pose.pose.position.z;
    double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
    
    // Check for position jump
    if (distance > max_position_jump_m_) {
      RCLCPP_WARN(this->get_logger(), "Position jump detected: %.2fm", distance);
      return false;
    }
    
    // Check velocity constraint
    double dt = (rclcpp::Time(pose.header.stamp) - rclcpp::Time(last_pose.header.stamp)).seconds();
    if (dt > 0.001) {  // Avoid division by very small numbers
      double velocity = distance / dt;
      if (velocity > max_velocity_mps_) {
        RCLCPP_WARN(this->get_logger(), "Excessive velocity detected: %.2f m/s", velocity);
        return false;
      }
    }
    
    return true;
  }
  
  bool check_time_consistency(const geometry_msgs::msg::PoseStamped& pose)
  {
    if (gnss_position_history_.empty()) return true;
    
    const auto& last_pose = gnss_position_history_.back();
    double dt = (rclcpp::Time(pose.header.stamp) - rclcpp::Time(last_pose.header.stamp)).seconds();
    
    // Check for time gaps that are too large
    if (std::abs(dt) > time_consistency_threshold_sec_) {
      RCLCPP_WARN(this->get_logger(), "Time consistency check failed: %.2fs gap", dt);
      return false;
    }
    
    // Check for time going backwards (except small timing variations)
    if (dt < -0.01) {  // Allow 10ms backward tolerance for timing jitter
      RCLCPP_WARN(this->get_logger(), "Time sequence violation: %.3fs", dt);
      return false;
    }
    
    return true;
  }
  
  bool check_statistical_outlier(const geometry_msgs::msg::PoseStamped& pose)
  {
    if (gnss_position_history_.size() < min_samples_for_outlier_detection_) {
      return true;  // Not enough data for statistical analysis
    }
    
    // Calculate mean position from history
    double mean_x = 0.0, mean_y = 0.0, mean_z = 0.0;
    for (const auto& hist_pose : gnss_position_history_) {
      mean_x += hist_pose.pose.position.x;
      mean_y += hist_pose.pose.position.y;
      mean_z += hist_pose.pose.position.z;
    }
    size_t n = gnss_position_history_.size();
    mean_x /= n;
    mean_y /= n;
    mean_z /= n;
    
    // Calculate standard deviation
    double var_x = 0.0, var_y = 0.0, var_z = 0.0;
    for (const auto& hist_pose : gnss_position_history_) {
      double dx = hist_pose.pose.position.x - mean_x;
      double dy = hist_pose.pose.position.y - mean_y;
      double dz = hist_pose.pose.position.z - mean_z;
      var_x += dx * dx;
      var_y += dy * dy;
      var_z += dz * dz;
    }
    double std_x = std::sqrt(var_x / n);
    double std_y = std::sqrt(var_y / n);
    double std_z = std::sqrt(var_z / n);
    
    // Calculate Z-scores for current pose
    double z_x = std_x > 0.001 ? std::abs(pose.pose.position.x - mean_x) / std_x : 0.0;
    double z_y = std_y > 0.001 ? std::abs(pose.pose.position.y - mean_y) / std_y : 0.0;
    double z_z = std_z > 0.001 ? std::abs(pose.pose.position.z - mean_z) / std_z : 0.0;
    
    // Check if any Z-score exceeds threshold
    if (z_x > outlier_z_score_threshold_ || z_y > outlier_z_score_threshold_ || z_z > outlier_z_score_threshold_) {
      RCLCPP_WARN(this->get_logger(), "Statistical outlier detected: Z-scores(%.2f, %.2f, %.2f)", z_x, z_y, z_z);
      return false;
    }
    
    return true;
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
  std::deque<geometry_msgs::msg::PoseStamped> gnss_position_history_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GnssImuPoser>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
