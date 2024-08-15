#pragma once

// TODO
// - remove m_ before private members
// - add const to member functions
// fix includes in all files
// - should we rclcpp::shutdown in construction instead
//

// std
#include <chrono>      //chrono_literals
#include <functional>  // std::bind , std::placeholders

// ros
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>           // WallTimer
#include <sensor_msgs/msg/image.hpp>  //image msg published
#include <std_srvs/srv/trigger.hpp>   // Trigger
#include <boost/thread.hpp>
#include "boost/multi_array.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
// arena sdk
#include "ArenaApi.h"

class ArenaCameraNode : public rclcpp::Node
{
 public:
  ArenaCameraNode() : Node("arena_camera_node")
  {
    // set stdout buffer size for ROS defined size BUFSIZE
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    log_info(std::string("Creating \"") + this->get_name() + "\" node");
    parse_parameters_();
    initialize_();
    log_info(std::string("Created \"") + this->get_name() + "\" node");
  }

  ~ArenaCameraNode()
  {
    log_info(std::string("Destroying \"") + this->get_name() + "\" node");
  }

  void log_debug(std::string msg) { RCLCPP_DEBUG(this->get_logger(), msg.c_str()); };
  void log_info(std::string msg) { RCLCPP_INFO(this->get_logger(), msg.c_str()); };
  void log_warn(std::string msg) { RCLCPP_WARN(this->get_logger(), msg.c_str()); };
  void log_err(std::string msg) { RCLCPP_ERROR(this->get_logger(), msg.c_str()); };

 private:
  std::shared_ptr<Arena::ISystem> m_pSystem;
  std::shared_ptr<Arena::IDevice> m_pDevice;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_pub_;
  rclcpp::TimerBase::SharedPtr m_wait_for_device_timer_callback_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_trigger_an_image_srv_;

  std::string serial_;
  bool is_passed_serial_;
  std::string device_user_id_; //added
  std::string topic_;
  std::string topic_point_cloud_; 
  size_t width_;
  bool is_passed_width;

  size_t height_;
  bool is_passed_height;

  double gain_;
  bool is_passed_gain_;

  double exposure_time_;
  bool is_passed_exposure_time_;

  std::string pixelformat_pfnc_;
  std::string pixelformat_ros_;
  bool is_passed_pixelformat_ros_;

  bool trigger_mode_activated_;

  std::string pub_qos_history_;
  bool is_passed_pub_qos_history_;

  size_t pub_qos_history_depth_;
  bool is_passed_pub_qos_history_depth_;

  std::string pub_qos_reliability_;
  bool is_passed_pub_qos_reliability_;



  // Added 
  /** Controls if spatial filtering is applied. */
  // Added 
  boost::recursive_mutex grab_mutex_;
 /** Specifies the acquisition mode of the current device. It helps determine the number of frames to acquire during each acquisition sequence. */
  std::string acquisition_mode_;
  /** Flag to indicates if we want to publish the PointCloud or not */
  bool publish_point_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
  // Scales and Offsets for x, y and z
  float scale_x, scale_y, scale_z, offset_x, offset_y;
  
  /** Controls if spatial filtering is applied. */
  bool scan_3d_spatial_filter_enable_;

  /** Controls if the filter for removing flying pixels is applied. */
  bool scan_3d_flying_pixels_removal_enable_;

  /** Controls the distance threshold in the filter for removing flying pixels. */
  int scan_3d_flying_pixels_distance_threshold_;

  /** Selects the exposure time. */
  std::string exposure_time_selector_;

  /** Selects the time of flight operating mode. */
  std::string scan_3d_operating_mode_;

  /** Controls if the confidence threshold is enabled and applied to the output. */
  bool scan_3d_confidence_threshold_enable_;

  /** Controls the minimum threshold value that determines whether or not the depth of a pixel is reliable. */
  int scan_3d_confidence_threshold_min_;

  /** Specifies high dynamic range (HDR) mode. */
  std::string scan_3d_hdr_mode_;

  /** Selects which Scan 3D Mode is enabled. */
  std::string scan_3d_mode_selector_;

  /** This node selects the specific trigger type to configure. */
  std::string trigger_selector_;

  /** Controls the On/Off status of the current trigger. */
  std::string trigger_mode_;

  /** This node specifies the source of the trigger. It can be a software internal signal of a physical input hardware signal. */
  std::string trigger_source_;

  /** This node specifies the state in which trigger is activated. */
  std::string trigger_activation_;

  /** Specifies the delay in microseconds (us) to apply after the trigger reception before activating it. */
  float trigger_delay_;

  /** Info from parameters whose names contain 'get_camera_parameter_info_' value will be displayed. */
  std::string get_camera_parameter_info_;

  /** Controls the echo of the elapsed time to grab and publish an image. */
  bool echo_elapsed_time_;

  double frame_rate_;
  // End Added 



  void parse_parameters_();
  void initialize_();
  void parameters_declarations();
  void wait_for_device_timer_callback_();

  void run_();
  // TODO :
  // - handle misconfigured device
  Arena::IDevice* create_device_ros_();
  void set_nodes_();
  void set_nodes_load_default_profile_();
  void set_nodes_roi_();
  void set_nodes_gain_();
  void set_nodes_pixelformat_();
  void set_nodes_exposure_();
  void set_nodes_trigger_mode_();
  void set_nodes_test_pattern_image_();
  void publish_images_();
  //added 
  void publish_an_image_on_trigger_(
      std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void msg_form_image_(Arena::IImage* pImage,
                       sensor_msgs::msg::Image& image_msg);


  // Added

  /**
   * Update the AcquisitionFrameRateEnable value
   * @param target_acquisition_frame_rate_enable the target value
   * @param reached_acquisition_frame_rate_enable the value that could be reached
   * @return true if the targeted value could be reached
   */
  bool setAcquisitionFrameRateEnable(const bool& target_acquisition_frame_rate_enable, bool& reached_acquisition_frame_rate_enable);

  /**
   *  Update the AcquisitionFrameRate value
   * @param target_acquisition_frame_rate the target value
   * @param reached_acquisition_frame_rate the value that could be reached
   * @return true if the targeted value could be reached
   */
  bool setAcquisitionFrameRate(const float& target_acquisition_frame_rate, float& reached_acquisition_frame_rate);

  /**
   * Update the AcquisitionMode value
   * @param target_acquisition_mode the target value
   * @param reached_acquisition_mode the value that could be reached
   * @return true if the targeted value could be reached
   */
  bool setAcquisitionMode(const std::string& target_acquisition_mode, std::string& reached_acquisition_mode);

  /**
  * Update the Scan3dSpatialFilterEnable value
  * @param target_scan_3d_spatial_filter_enable the target value
  * @param reached_scan_3d_spatial_filter_enable_ the value that could be reached
  * @return true if the targeted value could be reached
  */
  bool setScan3dSpatialFilterEnable(const bool& target_scan_3d_spatial_filter_enable, bool& reached_scan_3d_spatial_filter_enable);

  /**
  * Update the Scan3dFlyingPixelsRemovalEnable value
  * @param target_scan_3d_flying_pixels_removal_enable the target value
  * @param reached_scan_3d_flying_pixels_removal_enable the value that could be reached
  * @return true if the targeted value could be reached
  */
  bool setScan3dFlyingPixelsRemovalEnable(const bool& target_scan_3d_flying_pixels_removal_enable, bool& reached_scan_3d_flying_pixels_removal_enable);

  /**
  * Update the Scan3dFlyingPixelsDistanceThreshold value
  * @param target_scan_3d_flying_pixels_distance_threshold the target value
  * @param reached_scan_3d_flying_pixels_distance_threshold the value that could be reached
  * @return true if the targeted value could be reached
  */
  bool setScan3dFlyingPixelsDistanceThreshold(const int& target_scan_3d_flying_pixels_distance_threshold, int& reached_scan_3d_flying_pixels_distance_threshold);

  /**
   * Update the ExposureTimeSelector value
   * @param target_exposure_time_selector the target value
   * @param reached_exposure_time_selector the value that could be reached
   * @return true if the targeted value could be reached
   */
  bool setExposureTimeSelector(const std::string& target_exposure_time_selector, std::string& reached_exposure_time_selector);

  /**
   * Update the Scan3dOperatingMode value
   * @param target_scan_3d_operating_mode the target value
   * @param reached_scan_3d_operating_mode the value that could be reached
   * @return true if the targeted value could be reached
   */
  bool setScan3dOperatingMode(const std::string& target_scan_3d_operating_mode, std::string& reached_scan_3d_operating_mode);

  /**
   * Update the Scan3dConfidenceThresholdEnable value
   * @param target_scan_3d_confidence_threshold_enable the target value
   * @param reached_scan_3d_confidence_threshold_enable the value that could be reached
   * @return true if the targeted value could be reached
   */
  bool setScan3dConfidenceThresholdEnable(const bool& target_scan_3d_confidence_threshold_enable, bool& reached_scan_3d_confidence_threshold_enable);

  /**
   * Update the Scan3dConfidenceThresholdMin value
   * @param target_scan_3d_confidence_threshold_min the target value
   * @param reached_scan_3d_confidence_threshold_min the value that could be reached
   * @return true if the targeted value could be reached
   */
  bool setScan3dConfidenceThresholdMin(const int& target_scan_3d_confidence_threshold_min, int& reached_scan_3d_confidence_threshold_min);

  /**
   * Update the Scan3dHDRMode value
   * @param target_scan_3d_hdr_mode the target value
   * @param reached_scan_3d_hdr_mode the value that could be reached
   * @return true if the targeted value could be reached
   */
  bool setScan3dHDRMode(const std::string& target_scan_3d_hdr_mode, std::string& reached_scan_3d_hdr_mode);

  /**
   * Update the Scan3dModeSelector value
   * @param target_scan_3d_mode_selector the target value
   * @param reached_scan_3d_mode_selector the value that could be reached
   * @return true if the targeted value could be reached
   */
  bool setScan3dModeSelector(const std::string& target_scan_3d_mode_selector, std::string& reached_scan_3d_mode_selector);

  /**
   * Update the TriggerSelector value
   * @param target_trigger_selector the target value
   * @param reached_trigger_selector the value that could be reached
   * @return true if the targeted value could be reached
   */
  bool setTriggerSelector(const std::string& target_trigger_selector, std::string& reached_trigger_selector);

  /**
   * Update the TriggerMode value
   * @param target_trigger_mode the target value
   * @param reached_trigger_mode the value that could be reached
   * @return true if the targeted value could be reached
   */
  bool setTriggerMode(const std::string& target_trigger_mode, std::string& reached_trigger_mode);

  /**
   * Update the TriggerSource value
   * @param target_trigger_source the target value
   * @param reached_trigger_source the value that could be reached
   * @return true if the targeted value could be reached
   */
  bool setTriggerSource(const std::string& target_trigger_source, std::string& reached_trigger_source);

  /**
   * Update the TriggerActivation value
   * @param target_trigger_activation the target value
   * @param reached_trigger_activation the value that could be reached
   * @return true if the targeted value could be reached
   */
  bool setTriggerActivation(const std::string& target_trigger_activation, std::string& reached_trigger_activation);

  /**
   * Update the TriggerDelay value
   * @param target_trigger_delay the target value
   * @param reached_trigger_delay the value that could be reached
   * @return true if the targeted value could be reached
   */
  bool setTriggerDelay(const float& target_trigger_delay, float& reached_trigger_delay);
  bool setTriggerDelayValue(const float& target_trigger_delay, float& reached_trigger_delay); 
  bool setTriggerActivationValue(const std::string& target_trigger_activation, std::string& reached_trigger_activation);
  bool setTriggerSourceValue(const std::string& target_trigger_source, std::string& reached_trigger_source);
  bool setTriggerSelectorValue(const std::string& target_trigger_selector, std::string& reached_trigger_selector);
  bool setScan3dModeSelectorValue(const std::string& target_scan_3d_mode_selector, std::string& reached_scan_3d_mode_selector);
  bool setScan3dHDRModeValue(const std::string& target_scan_3d_hdr_mode, std::string& reached_scan_3d_hdr_mode);
  bool setScan3dConfidenceThresholdMinValue(const int& target_scan_3d_confidence_threshold_min, int& reached_scan_3d_confidence_threshold_min); 
  bool setScan3dConfidenceThresholdEnableValue(const bool& target_scan_3d_confidence_threshold_enable, bool& reached_scan_3d_confidence_threshold_enable); 
  bool setScan3dOperatingModeValue(const std::string& target_scan_3d_operating_mode, std::string& reached_scan_3d_operating_mode);
  bool setScan3dFlyingPixelsDistanceThresholdValue(const int& target_scan_3d_flying_pixels_distance_threshold, int& reached_scan_3d_flying_pixels_distance_threshold);
  bool setScan3dFlyingPixelsRemovalEnableValue(const bool& target_scan_3d_flying_pixels_removal_enable, bool& reached_scan_3d_flying_pixels_removal_enable); 
  bool setScan3dSpatialFilterEnableValue(const bool& target_scan_3d_spatial_filter_enable, bool& reached_scan_3d_spatial_filter_enable);
  bool setAcquisitionModeValue(const std::string& target_acquisition_mode, std::string& reached_acquisition_mode); 
  bool setAcquisitionFrameRateEnableValue(const bool& target_acquisition_frame_rate_enable, bool& reached_acquisition_frame_rate_enable);
  bool setTriggerModeValue(const std::string& target_trigger_mode, std::string& reached_trigger_mode);
  bool setExposureTimeSelectorValue(const std::string& target_exposure_time_selector, std::string& reached_exposure_time_selector);
  bool setAcquisitionFrameRateValue(const float& target_acquisition_frame_rate, float& reached_acquisition_frame_rate); 


  // End Added
};
