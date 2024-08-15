#include <cstring>    // memcopy
#include <stdexcept>  // std::runtime_err
#include <string>

// ROS
#include "rmw/types.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
// ArenaSDK
#include "ArenaCameraNode.h"
#include "light_arena/deviceinfo_helper.h"
#include "rclcpp_adapter/pixelformat_translation.h"
#include "rclcpp_adapter/quilty_of_service_translation.cpp"

void ArenaCameraNode::parse_parameters_()
{
  std::string nextParameterToDeclare = "";
  try {
    nextParameterToDeclare = "serial";
    if (this->has_parameter("serial")) {
        int serial_integer;
        this->get_parameter<int>("serial", serial_integer);
        serial_ = std::to_string(serial_integer);
        is_passed_serial_ = true;
} else {
    serial_ = ""; // Set it to an empty string to indicate it's not passed.
    is_passed_serial_ = false;
}
    
    nextParameterToDeclare = "pixelformat";
    pixelformat_ros_ = this->declare_parameter("pixelformat", "");
    is_passed_pixelformat_ros_ = pixelformat_ros_ != "";

    nextParameterToDeclare = "width";
    width_ = this->declare_parameter("width", 0);
    is_passed_width = width_ > 0;

    nextParameterToDeclare = "height";
    height_ = this->declare_parameter("height", 0);
    is_passed_height = height_ > 0;

    nextParameterToDeclare = "gain";
    gain_ = this->declare_parameter("gain", -1.0);
    is_passed_gain_ = gain_ >= 0;

    nextParameterToDeclare = "exposure_time";
    exposure_time_ = this->declare_parameter("exposure_time", -1.0);
    is_passed_exposure_time_ = exposure_time_ >= 0;

    nextParameterToDeclare = "topic";
    topic_ = this->declare_parameter(
        "topic", std::string("/") + this->get_name() + "/images");

    // Added 
    topic_point_cloud_ = this->declare_parameter(
        "topic_point_cloud", std::string("/") + this->get_name() + "/point_cloud");
    // no need to is_passed_topic_

    nextParameterToDeclare = "qos_history";
    pub_qos_history_ = this->declare_parameter("qos_history", "");
    is_passed_pub_qos_history_ = pub_qos_history_ != "";

    nextParameterToDeclare = "qos_history_depth";
    pub_qos_history_depth_ = this->declare_parameter("qos_history_depth", 0);
    is_passed_pub_qos_history_depth_ = pub_qos_history_depth_ > 0;

    nextParameterToDeclare = "qos_reliability";
    pub_qos_reliability_ = this->declare_parameter("qos_reliability", "");
    is_passed_pub_qos_reliability_ = pub_qos_reliability_ != "";

  } catch (rclcpp::ParameterTypeException& e) {
    log_err(nextParameterToDeclare + " argument");
    throw;
  }
}

void ArenaCameraNode::initialize_()
{
  using namespace std::chrono_literals;
  // ARENASDK ---------------------------------------------------------------
  // Custom deleter for system
  m_pSystem =
      std::shared_ptr<Arena::ISystem>(nullptr, [=](Arena::ISystem* pSystem) {
        if (pSystem) {  // this is an issue for multi devices
          Arena::CloseSystem(pSystem);
          log_info("System is destroyed");
        }
      });
  m_pSystem.reset(Arena::OpenSystem());

  // Custom deleter for device
  m_pDevice =
      std::shared_ptr<Arena::IDevice>(nullptr, [=](Arena::IDevice* m_pDevice) {
        if (m_pSystem && m_pDevice) {
          m_pSystem->DestroyDevice(m_pDevice);
          log_info("Device is destroyed");
        }
      });

  //
  // CHECK DEVICE CONNECTION ( timer ) --------------------------------------
  //
  // TODO
  // - Think of design that allow the node to start stream as soon as
  // it is initialized without waiting for spin to be called
  // - maybe change 1s to a smaller value
  m_wait_for_device_timer_callback_ = this->create_wall_timer(
      1s, std::bind(&ArenaCameraNode::wait_for_device_timer_callback_, this));

  //
  // TRIGGER (service) ------------------------------------------------------
  //
  using namespace std::placeholders;
  m_trigger_an_image_srv_ = this->create_service<std_srvs::srv::Trigger>(
      std::string(this->get_name()) + "/trigger_image",
      std::bind(&ArenaCameraNode::publish_an_image_on_trigger_, this, _1, _2));

  //
  // Publisher --------------------------------------------------------------
  //
  // m_pub_qos is rclcpp::SensorDataQoS has these defaults
  // https://github.com/ros2/rmw/blob/fb06b57975373b5a23691bb00eb39c07f1660ed7/rmw/include/rmw/qos_profiles.h#L25

  /*
  static const rmw_qos_profile_t rmw_qos_profile_sensor_data =
  {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    5, // history depth
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false // avoid ros namespace conventions
  };
  */
  rclcpp::SensorDataQoS pub_qos_;
  // QoS history
  if (is_passed_pub_qos_history_) {
    if (is_supported_qos_histroy_policy(pub_qos_history_)) {
      pub_qos_.history(
          K_CMDLN_PARAMETER_TO_QOS_HISTORY_POLICY[pub_qos_history_]);
    } else {
      log_err(pub_qos_history_ + " is not supported for this node");
      // TODO
      // should thorow instead??
      // should this keeps shutting down if for some reasons this node is kept
      // alive
      throw;
    }
  }
  // QoS depth
  if (is_passed_pub_qos_history_depth_ &&
      K_CMDLN_PARAMETER_TO_QOS_HISTORY_POLICY[pub_qos_history_] ==
          RMW_QOS_POLICY_HISTORY_KEEP_LAST) {
    // TODO
    // test err msg withwhen -1
    pub_qos_.keep_last(pub_qos_history_depth_);
  }

  // Qos reliability
  if (is_passed_pub_qos_reliability_) {
    if (is_supported_qos_reliability_policy(pub_qos_reliability_)) {
      pub_qos_.reliability(
          K_CMDLN_PARAMETER_TO_QOS_RELIABILITY_POLICY[pub_qos_reliability_]);
    } else {
      log_err(pub_qos_reliability_ + " is not supported for this node");
      throw;
    }
  }

  // rmw_qos_history_policy_t history_policy_ = RMW_QOS_
  // rmw_qos_history_policy_t;
  // auto pub_qos_init = rclcpp::QoSInitialization(history_policy_, );

  m_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      this->get_parameter("topic").as_string(), pub_qos_);

  std::stringstream pub_qos_info;
  auto pub_qos_profile = pub_qos_.get_rmw_qos_profile();
  pub_qos_info
      << '\t' << "QoS history     = "
      << K_QOS_HISTORY_POLICY_TO_CMDLN_PARAMETER[pub_qos_profile.history]
      << '\n';
  pub_qos_info << "\t\t\t\t"
               << "QoS depth       = " << pub_qos_profile.depth << '\n';
  pub_qos_info << "\t\t\t\t"
               << "QoS reliability = "
               << K_QOS_RELIABILITY_POLICY_TO_CMDLN_PARAMETER[pub_qos_profile
                                                                  .reliability]
               << '\n';

  log_info(pub_qos_info.str());
}

void ArenaCameraNode::wait_for_device_timer_callback_()
{
  // something happend while checking for cameras
  if (!rclcpp::ok()) {
    log_err("Interrupted while waiting for arena camera. Exiting.");
    rclcpp::shutdown();
  }

  // camera discovery
  m_pSystem->UpdateDevices(100);  // in millisec
  auto device_infos = m_pSystem->GetDevices();

  // no camera is connected
  if (!device_infos.size()) {
    log_info("No arena camera is connected. Waiting for device(s)...");
  }
  // at least on is found
  else {
    m_wait_for_device_timer_callback_->cancel();
    log_info(std::to_string(device_infos.size()) +
             " arena device(s) has been discoved.");
    run_();
  }
}


void ArenaCameraNode::parameters_declarations(){

  // Added 
    acquisition_mode_ = this->declare_parameter("acquisition_mode", "Continuous");
    RCLCPP_DEBUG_STREAM(this->get_logger(), "acquisition_mode will be: " << acquisition_mode_);

    // Get publish_point_cloud
    publish_point_cloud_ = this->declare_parameter("publish_point_cloud", true);
    RCLCPP_DEBUG_STREAM(this->get_logger(), "publish_point_cloud will be: " << publish_point_cloud_);

    // Get scan_3d_spatial_filter_enable_
    scan_3d_spatial_filter_enable_ = this->declare_parameter("scan_3d_spatial_filter_enable", false);
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Scan3dSpatialFilterEnable will be: " << scan_3d_spatial_filter_enable_);

    // Get scan_3d_flying_pixels_removal_enable_m
    scan_3d_flying_pixels_removal_enable_ = this->declare_parameter("scan_3d_flying_pixels_removal_enable", false);
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Scan3dFlyingPixelsRemovalEnable will be: " << scan_3d_flying_pixels_removal_enable_);

    // Get scan_3d_flying_pixels_distance_threshold_
    //nextParameterToDeclare = "scan_3d_flying_pixels_distance_threshold";
    //scan_3d_flying_pixels_distance_threshold_ = this->declare_parameter("scan_3d_flying_pixels_distance_threshold", 0);
    //RCLCPP_DEBUG_STREAM(this->get_logger(), "Scan3dFlyingPixelsDistanceThreshold will be: " << scan_3d_flying_pixels_distance_threshold_);

    // Get exposure_time_selector_
    exposure_time_selector_ = this->declare_parameter("exposure_time_selector", "Exp1000Us");
    RCLCPP_DEBUG_STREAM(this->get_logger(), "ExposureTimeSelector will be: " << exposure_time_selector_);

    //Device user id
    device_user_id_ = this->declare_parameter("device_user_id", "");
    //device_user_id_ = "camera_ToF_1";
    // Get scan_3d_operating_mode_
    scan_3d_operating_mode_ = this->declare_parameter("scan_3d_operating_mode", "Distance1250mmSingleFreq");
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Scan3dOperatingMode will be: " << scan_3d_operating_mode_);

    // Get scan_3d_confidence_threshold_enable_
    scan_3d_confidence_threshold_enable_ = this->declare_parameter("scan_3d_confidence_threshold_enable", false);
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Scan3dConfidenceThresholdEnable will be: " << scan_3d_confidence_threshold_enable_);

    // Get scan_3d_confidence_threshold_min_
    scan_3d_confidence_threshold_min_ = this->declare_parameter("scan_3d_confidence_threshold_min", 0);
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Scan3dConfidenceThresholdEnable will be: " << scan_3d_confidence_threshold_min_);

    // Get scan_3d_hdr_mode_
    scan_3d_hdr_mode_ = this->declare_parameter("scan_3d_hdr_mode", "Off");
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Scan3HDRMode will be: " << scan_3d_hdr_mode_);

    // Get scan_3d_mode_selector_
    scan_3d_mode_selector_ = this->declare_parameter("scan_3d_mode_selector", "Processed");
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Scan3dModeSelector will be: " << scan_3d_mode_selector_);

    // Get trigger_selector_
    trigger_selector_ = this->declare_parameter("trigger_selector", "FrameStart");
    RCLCPP_DEBUG_STREAM(this->get_logger(), "TriggerSelector will be: " << trigger_selector_);

    // Get trigger_mode_
    trigger_mode_ = this->declare_parameter("trigger_mode", "Off");
    RCLCPP_DEBUG_STREAM(this->get_logger(), "TriggerMode will be: " << trigger_mode_);

    // Get trigger_source_
    trigger_source_ = this->declare_parameter("trigger_source", "Software");
    RCLCPP_DEBUG_STREAM(this->get_logger(), "TriggerSource will be: " << trigger_source_);

    // Get trigger_activation_
    trigger_activation_ = this->declare_parameter("trigger_activation", "RisingEdge");
    RCLCPP_DEBUG_STREAM(this->get_logger(), "TriggerActivation will be: " << trigger_activation_);

    // Get trigger_delay_
    trigger_delay_ = this->declare_parameter("trigger_delay", 0.0);
    RCLCPP_DEBUG_STREAM(this->get_logger(), "TriggerDelay will be: " << trigger_delay_);

    // Get frame_rate_
    frame_rate_ = this->declare_parameter("frame_rate", 10.0);
    RCLCPP_DEBUG_STREAM(this->get_logger(), "frame_rate will be: " << frame_rate_);


    // Get get_camera_parameter_info_
    get_camera_parameter_info_ = this->declare_parameter("get_camera_parameter_info", "none");
    if (get_camera_parameter_info_ == "none")
        RCLCPP_DEBUG_STREAM(this->get_logger(), "No parameters will have their info displayed");
    else if (get_camera_parameter_info_ == "all")
        RCLCPP_DEBUG_STREAM(this->get_logger(), "All parameters will have their info displayed");
    else
        RCLCPP_DEBUG_STREAM(this->get_logger(), "Parameters whose names contain '" << get_camera_parameter_info_ << "' will have their info displayed");

    // Get echo_elapsed_time
    echo_elapsed_time_ = this->declare_parameter("echo_elapsed_time", false);
    RCLCPP_DEBUG_STREAM(this->get_logger(), "echo_elapsed_time will be: " << echo_elapsed_time_);

}
void ArenaCameraNode::run_()
{
  parameters_declarations(); 
  auto device = create_device_ros_();
  m_pDevice.reset(device);
  set_nodes_();

   //Added
   // Code get from ArenaSDK_Linux_x64/Examples/Arena/Cpp_Helios_MinMaxD epth/Cpp_Helios_MinMaxDepth.cpp
      // Code to get the scales and offsets for x, y and z
  auto nodemap = m_pDevice->GetNodeMap();
  Arena::SetNodeValue<GenICam::gcstring>(nodemap, "Scan3dCoordinateSelector", "CoordinateA");
  scale_x = static_cast<float>(Arena::GetNodeValue<double>(nodemap, "Scan3dCoordinateScale"));
  offset_x = static_cast<float>(Arena::GetNodeValue<double>(nodemap, "Scan3dCoordinateOffset"));
  Arena::SetNodeValue<GenICam::gcstring>(nodemap, "Scan3dCoordinateSelector", "CoordinateB");
  scale_y = static_cast<float>(Arena::GetNodeValue<double>(nodemap, "Scan3dCoordinateScale"));
  offset_y = static_cast<float>(Arena::GetNodeValue<double>(nodemap, "Scan3dCoordinateOffset"));
  Arena::SetNodeValue<GenICam::gcstring>(nodemap, "Scan3dCoordinateSelector", "CoordinateC");
  scale_z = static_cast<float>(Arena::GetNodeValue<double>(nodemap, "Scan3dCoordinateScale"));

  RCLCPP_INFO_STREAM(this->get_logger(), "#################### SCALES INFO ####################");
  RCLCPP_INFO_STREAM(this->get_logger(), "scale_x = " << scale_x << " ; offset_x = " << offset_x);
  RCLCPP_INFO_STREAM(this->get_logger(), "scale_y = " << scale_y << " ; offset_y = " << offset_y);
  RCLCPP_INFO_STREAM(this->get_logger(), "scale_z = " << scale_z);

   GenApi::node_vector nodeList;
   nodemap->GetNodes(nodeList);
   if(get_camera_parameter_info_ != "none")
   {
     RCLCPP_INFO(this->get_logger(), "#################### PARAMETERS INFO ####################");
     RCLCPP_INFO(this->get_logger(), "get_camera_parameter_info_: %s", get_camera_parameter_info_.c_str());
     RCLCPP_INFO(this->get_logger(), "---------------------------------------------------------");
     for(size_t i = 0 ; i < nodeList.size() ; i++)
     {
       if(get_camera_parameter_info_ == "all" || nodeList[i]->GetName().find(get_camera_parameter_info_.c_str()) != std::string::npos)
       {
         RCLCPP_INFO(this->get_logger(), "Node %zu: %s", i, nodeList[i]->GetName().c_str());
         RCLCPP_INFO(this->get_logger(), "\t - Description: %s", nodeList[i]->GetDescription().c_str());
         RCLCPP_INFO(this->get_logger(), "\t - Interface: %s", GetInterfaceName(nodeList[i]).c_str());
         RCLCPP_INFO(this->get_logger(), "\t - IsAvailable = %d", IsAvailable(nodeList[i]));
         RCLCPP_INFO(this->get_logger(), "\t - IsReadable = %d", IsReadable(nodeList[i]));
         RCLCPP_INFO(this->get_logger(), "\t - IsWritable = %d", IsWritable(nodeList[i]));
         RCLCPP_INFO(this->get_logger(), "---------------------------------------------------------");
       }
     }
   }
 // End added

  RCLCPP_INFO(this->get_logger(), "#################### SET PARAMETERS ####################");
  m_pDevice->StartStream();
  if (trigger_mode_=="Off") {
    publish_images_();
  } else {
    // else ros::spin will
  }
}

void ArenaCameraNode::publish_images_()
{
  Arena::IImage* pImage = nullptr;
  while (rclcpp::ok()) {
    try {
      auto p_image_msg = std::make_unique<sensor_msgs::msg::Image>();
      pImage = m_pDevice->GetImage(1000);
      msg_form_image_(pImage, *p_image_msg);
      // Added 
      
      point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_point_cloud_, 10);
      

      // End Added
      


      // Added
    if (point_cloud_pub_->get_subscription_count() > 0 && p_image_msg->encoding == "Coord3D_ABCY16" && publish_point_cloud_)
    {
        pcl::PointCloud<pcl::PointXYZI> point_cloud;
        point_cloud.height = p_image_msg->height;
        point_cloud.width = p_image_msg->width;
        point_cloud.is_dense = true;
        point_cloud.points.resize(point_cloud.height * point_cloud.width);
        int size_img = point_cloud.points.size();
        int data;
        for(int i = 0; i < size_img; i++)
        {
            // x
            data = (p_image_msg->data[i*8+1] << 8) + p_image_msg->data[i*8];
            point_cloud.points[i].x = (data * scale_x + offset_x) * 0.001;
            
            // y
            data = (p_image_msg->data[i*8+3] << 8) + p_image_msg->data[i*8+2];
            point_cloud.points[i].y = (data * scale_y + offset_y) * 0.001;
            
            // z
            data = (p_image_msg->data[i*8+5] << 8) + p_image_msg->data[i*8+4];
            point_cloud.points[i].z = (data * scale_z) * 0.001;
            
            // intensity
            data = (p_image_msg->data[i*8+7] << 8) + p_image_msg->data[i*8+6];
            point_cloud.points[i].intensity = static_cast<float>(data);
        }
        
        sensor_msgs::msg::PointCloud2 point_cloud_msg;
        pcl::toROSMsg(point_cloud, point_cloud_msg); // Convert PCL PointCloud to ROS2 PointCloud2
        point_cloud_msg.header = p_image_msg->header;
        
        point_cloud_pub_->publish(point_cloud_msg);
    }

    m_pub_->publish(std::move(p_image_msg));

      // End added 
      log_info(std::string("image ") + std::to_string(pImage->GetFrameId()) +
               " published to " + topic_);
      this->m_pDevice->RequeueBuffer(pImage);

    } catch (std::exception& e) {
      if (pImage) {
        this->m_pDevice->RequeueBuffer(pImage);
        pImage = nullptr;
        log_warn(std::string("Exception occurred while publishing an image\n") +
                 e.what());
      }
    }
  };
}

void ArenaCameraNode::msg_form_image_(Arena::IImage* pImage,
                                      sensor_msgs::msg::Image& image_msg)
{
  try {
    // 1 ) Header
    //      - stamp.sec
    //      - stamp.nanosec
    //      - Frame ID
    image_msg.header.stamp.sec =
        static_cast<uint32_t>(pImage->GetTimestampNs() / 1000000000);
    image_msg.header.stamp.nanosec =
        static_cast<uint32_t>(pImage->GetTimestampNs() % 1000000000);
    image_msg.header.frame_id = std::to_string(pImage->GetFrameId());

    //
    // 2 ) Height
    //
    image_msg.height = height_;

    //
    // 3 ) Width
    //
    image_msg.width = width_;

    //
    // 4 ) encoding
    //
    image_msg.encoding = pixelformat_ros_;

    //
    // 5 ) is_big_endian
    //
    // TODO what to do if unknown
    image_msg.is_bigendian = pImage->GetPixelEndianness() ==
                             Arena::EPixelEndianness::PixelEndiannessBig;
    //
    // 6 ) step
    //
    // TODO could be optimized by moving it out
    auto pixel_length_in_bytes = pImage->GetBitsPerPixel() / 8;
    auto width_length_in_bytes = pImage->GetWidth() * pixel_length_in_bytes;
    image_msg.step =
        static_cast<sensor_msgs::msg::Image::_step_type>(width_length_in_bytes);

    //
    // 7) data
    //
    auto image_data_length_in_bytes = width_length_in_bytes * height_;
    image_msg.data.resize(image_data_length_in_bytes);
    auto x = pImage->GetData();
    std::memcpy(&image_msg.data[0], pImage->GetData(),
                image_data_length_in_bytes);

  } catch (...) {
    log_warn(
        "Failed to create Image ROS MSG. Published Image Msg might be "
        "corrupted");
  }
}

void ArenaCameraNode::publish_an_image_on_trigger_(
    std::shared_ptr<std_srvs::srv::Trigger::Request> request /*unused*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (trigger_mode_=="Off") {
    std::string msg =
        "Failed to trigger image because the device is not in trigger mode."
        "run `ros2 run arena_camera_node run --ros-args -p trigger_mode:=true`";
    log_warn(msg);
    response->message = msg;
    response->success = false;
  }

  log_info("A client triggered an image request");

  Arena::IImage* pImage = nullptr;
  try {
    // trigger
    bool triggerArmed = false;
    auto waitForTriggerCount = 10;
    
    if(trigger_mode_ == "On" && trigger_source_ == "Software") // added
    {
    do {
      // infinite loop when I step in (sometimes)
      triggerArmed =
          Arena::GetNodeValue<bool>(m_pDevice->GetNodeMap(), "TriggerArmed");

      if (triggerArmed == false && (waitForTriggerCount % 10) == 0) {
        log_info("waiting for trigger to be armed");
      }

    } while (triggerArmed == false);
    }
    log_debug("trigger is armed; triggering an image");
    Arena::ExecuteNode(m_pDevice->GetNodeMap(), "TriggerSoftware");

    // get image
    auto p_image_msg = std::make_unique<sensor_msgs::msg::Image>();

    log_debug("getting an image");
    pImage = m_pDevice->GetImage(1000);
    auto msg = std::string("image ") + std::to_string(pImage->GetFrameId()) +
               " published to " + topic_;
    msg_form_image_(pImage, *p_image_msg);
    m_pub_->publish(std::move(p_image_msg));
    response->message = msg;
    response->success = true;

    log_info(msg);
    this->m_pDevice->RequeueBuffer(pImage);

  }

  catch (std::exception& e) {
    if (pImage) {
      this->m_pDevice->RequeueBuffer(pImage);
      pImage = nullptr;
    }
    auto msg =
        std::string("Exception occurred while grabbing an image\n") + e.what();
    log_warn(msg);
    response->message = msg;
    response->success = false;

  }

  catch (GenICam::GenericException& e) {
    if (pImage) {
      this->m_pDevice->RequeueBuffer(pImage);
      pImage = nullptr;
    }
    auto msg =
        std::string("GenICam Exception occurred while grabbing an image\n") +
        e.what();
    log_warn(msg);
    response->message = msg;
    response->success = false;
  }
}

Arena::IDevice* ArenaCameraNode::create_device_ros_()
{
  m_pSystem->UpdateDevices(100);  // in millisec
  auto device_infos = m_pSystem->GetDevices();
  if (!device_infos.size()) {
    // TODO: handel disconnection
    throw std::runtime_error(
        "camera(s) were disconnected after they were discovered");
  }

  auto index = 0;
  if (is_passed_serial_) {
    index = DeviceInfoHelper::get_index_of_serial(device_infos, serial_);
  }

  // Added 
  if (!device_user_id_.empty())
  {
    std::vector<Arena::DeviceInfo>::iterator it;
    bool found_desired_device = false;
    for (it = device_infos.begin(); it != device_infos.end(); ++it)
      {
        std::string device_user_id_found(it->UserDefinedName());
        if ((0 == device_user_id_.compare(device_user_id_found)) ||
            (device_user_id_.length() < device_user_id_found.length() &&
             (0 ==
              device_user_id_found.compare(device_user_id_found.length() - device_user_id_.length(),
                                           device_user_id_.length(), device_user_id_))))
        {
          found_desired_device = true;
          break;
        }
      }
      if (found_desired_device)
      {
        RCLCPP_INFO_STREAM(this->get_logger(), "Found the desired camera with DeviceUserID " << device_user_id_ << ": ");

        auto pDevice_ = m_pSystem->CreateDevice(*it);
        return pDevice_;
      }
      else
      {
        RCLCPP_ERROR_STREAM(this->get_logger(),"Couldn't find the camera that matches the "
                         << "given DeviceUserID: " << device_user_id_<< "! "
                         << "Either the ID is wrong or the cam is not yet connected");
      }
  }
  
  auto pDevice = m_pSystem->CreateDevice(device_infos.at(index));
  log_info(std::string("device created ") +
           DeviceInfoHelper::info(device_infos.at(index)));
  return pDevice;
}

void ArenaCameraNode::set_nodes_()
{
  set_nodes_load_default_profile_();
  set_nodes_roi_();
  set_nodes_gain_();
  set_nodes_pixelformat_();
  set_nodes_exposure_();
  set_nodes_trigger_mode_();
  // Added
  
  // AcquisitionFrameRateEnable
  bool target_acquisition_frame_rate_enable, reached_acquisition_frame_rate_enable;
  if (trigger_mode_ == "On")
    target_acquisition_frame_rate_enable = false;
  else
    target_acquisition_frame_rate_enable = true;

  if (setAcquisitionFrameRateEnable(target_acquisition_frame_rate_enable, reached_acquisition_frame_rate_enable))
    RCLCPP_INFO(this->get_logger(), "AcquisitionFrameRateEnable set to: %s", reached_acquisition_frame_rate_enable ? "true" : "false");
  else
    RCLCPP_ERROR(this->get_logger(), "Error while setting AcquisitionFrameRateEnable. Current AcquisitionFrameRateEnable value is: %s", reached_acquisition_frame_rate_enable ? "true" : "false");
  
  // AcquisitionFrameRate
  float reached_acquisition_frame_rate;
  if (setAcquisitionFrameRate(frame_rate_, reached_acquisition_frame_rate))
    RCLCPP_INFO(this->get_logger(), "AcquisitionFrameRate set to: %f", reached_acquisition_frame_rate);
  else
    RCLCPP_ERROR(this->get_logger(), "Error while setting AcquisitionFrameRate. Current AcquisitionFrameRate value is: %f", reached_acquisition_frame_rate);
  /*
  // AcquisitionMode
  std::string reached_acquisition_mode;
  if (setAcquisitionMode(acquisition_mode_, reached_acquisition_mode))
    RCLCPP_INFO(this->get_logger(), "AcquisitionMode set to: %s", reached_acquisition_mode.c_str());
  else
    RCLCPP_ERROR(this->get_logger(), "Error while setting AcquisitionMode. Current AcquisitionMode value is: %s", reached_acquisition_mode.c_str());

  // Scan3dSpatialFilterEnable
  bool reached_scan_3d_spatial_filter_enable;
  if (setScan3dSpatialFilterEnable(scan_3d_spatial_filter_enable_, reached_scan_3d_spatial_filter_enable))
    RCLCPP_INFO(this->get_logger(), "Scan3dSpatialFilterEnable set to: %s", reached_scan_3d_spatial_filter_enable ? "true" : "false");
  else
    RCLCPP_ERROR(this->get_logger(), "Error while setting Scan3dSpatialFilterEnable. Current Scan3dSpatialFilterEnable value is: %s", reached_scan_3d_spatial_filter_enable ? "true" : "false");

  // Scan3dFlyingPixelsRemovalEnable
  bool reached_scan_3d_flying_pixels_removal_enable;
  if (setScan3dFlyingPixelsRemovalEnable(scan_3d_flying_pixels_removal_enable_, reached_scan_3d_flying_pixels_removal_enable))
    RCLCPP_INFO(this->get_logger(), "Scan3dFlyingPixelsRemovalEnable set to: %s", reached_scan_3d_flying_pixels_removal_enable ? "true" : "false");
  else
    RCLCPP_ERROR(this->get_logger(), "Error while setting Scan3dFlyingPixelsRemovalEnable. Current Scan3dFlyingPixelsRemovalEnable value is: %s", reached_scan_3d_flying_pixels_removal_enable ? "true" : "false");

  // Scan3dFlyingPixelsDistanceThreshold
  //int reached_scan_3d_flying_pixels_distance_threshold;
  //if (setScan3dFlyingPixelsDistanceThreshold(scan_3d_flying_pixels_distance_threshold_, reached_scan_3d_flying_pixels_distance_threshold))
  //  RCLCPP_INFO(this->get_logger(), "Scan3dFlyingPixelsDistanceThreshold set to: %d", reached_scan_3d_flying_pixels_distance_threshold);
  //else
  //  RCLCPP_ERROR(this->get_logger(), "Error while setting Scan3dFlyingPixelsDistanceThreshold. Current Scan3dFlyingPixelsDistanceThreshold value is: %d", reached_scan_3d_flying_pixels_distance_threshold);

  // ExposureTimeSelector
  std::string reached_exposure_time_selector;
  if (setExposureTimeSelector(exposure_time_selector_, reached_exposure_time_selector))
    RCLCPP_INFO(this->get_logger(), "ExposureTimeSelector set to: %s", reached_exposure_time_selector.c_str());
  else
    RCLCPP_ERROR(this->get_logger(), "Error while setting ExposureTimeSelector. Current ExposureTimeSelector value is: %s", reached_exposure_time_selector.c_str());

  // Scan3dOperatingMode
  std::string reached_scan_3d_operating_mode;
  if (setScan3dOperatingMode(scan_3d_operating_mode_, reached_scan_3d_operating_mode))
    RCLCPP_INFO(this->get_logger(), "Scan3dOperatingMode set to: %s", reached_scan_3d_operating_mode.c_str());
  else
    RCLCPP_ERROR(this->get_logger(), "Error while setting Scan3dOperatingMode. Current Scan3dOperatingMode value is: %s", reached_scan_3d_operating_mode.c_str());

  // Scan3dConfidenceThresholdEnable
  bool reached_scan_3d_confidence_threshold_enable;
  if (setScan3dConfidenceThresholdEnable(scan_3d_confidence_threshold_enable_, reached_scan_3d_confidence_threshold_enable))
    RCLCPP_INFO(this->get_logger(), "Scan3dConfidenceThresholdEnable set to: %s", reached_scan_3d_confidence_threshold_enable ? "true" : "false");
  else
    RCLCPP_ERROR(this->get_logger(), "Error while setting Scan3dConfidenceThresholdEnable. Current Scan3dConfidenceThresholdEnable value is: %s", reached_scan_3d_confidence_threshold_enable ? "true" : "false");

  // Scan3dConfidenceThresholdMin
  int reached_scan_3d_confidence_threshold_min;
  if (setScan3dConfidenceThresholdMin(scan_3d_confidence_threshold_min_, reached_scan_3d_confidence_threshold_min))
    RCLCPP_INFO(this->get_logger(), "Scan3dConfidenceThresholdMin set to: %d", reached_scan_3d_confidence_threshold_min);
  else
    RCLCPP_ERROR(this->get_logger(), "Error while setting Scan3dConfidenceThresholdMin. Current Scan3dConfidenceThresholdMin value is: %d", reached_scan_3d_confidence_threshold_min);

  // Scan3dHDRMode
  //std::string reached_scan_3d_hdr_mode;
  //if (setScan3dHDRMode(scan_3d_hdr_mode_, reached_scan_3d_hdr_mode))
  //  RCLCPP_INFO(this->get_logger(), "Scan3dHDRMode set to: %s", reached_scan_3d_hdr_mode.c_str());
  //else
  //  RCLCPP_ERROR(this->get_logger(), "Error while setting Scan3dHDRMode. Current Scan3dHDRMode value is: %s", reached_scan_3d_hdr_mode.c_str());

  // Scan3dModeSelector
  std::string reached_scan_3d_mode_selector;
  if (setScan3dModeSelector(scan_3d_mode_selector_, reached_scan_3d_mode_selector))
    RCLCPP_INFO(this->get_logger(), "Scan3dModeSelector set to: %s", reached_scan_3d_mode_selector.c_str());
  else
    RCLCPP_ERROR(this->get_logger(), "Error while setting Scan3dModeSelector. Current Scan3dModeSelector value is: %s", reached_scan_3d_mode_selector.c_str());

  // TriggerSelector
  std::string reached_trigger_selector;
  if (setTriggerSelector(trigger_selector_, reached_trigger_selector))
    RCLCPP_INFO(this->get_logger(), "TriggerSelector set to: %s", reached_trigger_selector.c_str());
  else
    RCLCPP_ERROR(this->get_logger(), "Error while setting TriggerSelector. Current TriggerSelector value is: %s", reached_trigger_selector.c_str());

  // TriggerMode
  std::string reached_trigger_mode;
  if (setTriggerMode(trigger_mode_, reached_trigger_mode))
    RCLCPP_INFO(this->get_logger(), "TriggerMode set to: %s", reached_trigger_mode.c_str());
  else
    RCLCPP_ERROR(this->get_logger(), "Error while setting TriggerMode. Current TriggerMode value is: %s", reached_trigger_mode.c_str());

  // TriggerSource
  std::string reached_trigger_source;
  if (setTriggerSource(trigger_source_, reached_trigger_source))
    RCLCPP_INFO(this->get_logger(), "TriggerSource set to: %s", reached_trigger_source.c_str());
  else
    RCLCPP_ERROR(this->get_logger(), "Error while setting TriggerSource. Current TriggerSource value is: %s", reached_trigger_source.c_str());

  // TriggerActivation
  std::string reached_trigger_activation;
  if (setTriggerActivation(trigger_activation_, reached_trigger_activation))
    RCLCPP_INFO(this->get_logger(), "TriggerActivation set to: %s", reached_trigger_activation.c_str());
  else
    RCLCPP_ERROR(this->get_logger(), "Error while setting TriggerActivation. Current TriggerActivation value is: %s", reached_trigger_activation.c_str());

  // TriggerDelay
  float reached_trigger_delay;
  if (setTriggerDelay(trigger_delay_, reached_trigger_delay))
    RCLCPP_INFO(this->get_logger(), "TriggerDelay set to: %f", reached_trigger_delay);
  else
    RCLCPP_ERROR(this->get_logger(), "Error while setting TriggerDelay. Current TriggerDelay value is: %f", reached_trigger_delay);

  // End Added
*/
    






  // configure Auto Negotiate Packet Size and Packet Resend
  Arena::SetNodeValue<bool>(m_pDevice->GetTLStreamNodeMap(), "StreamAutoNegotiatePacketSize", true);  // Modified, it was True instead of true
  Arena::SetNodeValue<bool>(m_pDevice->GetTLStreamNodeMap(), "StreamPacketResendEnable", true);

  //set_nodes_test_pattern_image_();
}

void ArenaCameraNode::set_nodes_load_default_profile_()
{
  auto nodemap = m_pDevice->GetNodeMap();
  // device run on default profile all the time if no args are passed
  // otherwise, overwise only these params
  Arena::SetNodeValue<GenICam::gcstring>(nodemap, "UserSetSelector", "Default");
  // execute the profile
  Arena::ExecuteNode(nodemap, "UserSetLoad");
  log_info("\tdefault profile is loaded");
}

void ArenaCameraNode::set_nodes_roi_()
{
  auto nodemap = m_pDevice->GetNodeMap();

  // Width -------------------------------------------------
  if (is_passed_width) {
    Arena::SetNodeValue<int64_t>(nodemap, "Width", width_);
  } else {
    width_ = Arena::GetNodeValue<int64_t>(nodemap, "Width");
  }

  // Height ------------------------------------------------
  if (is_passed_height) {
    Arena::SetNodeValue<int64_t>(nodemap, "Height", height_);
  } else {
    height_ = Arena::GetNodeValue<int64_t>(nodemap, "Height");
  }

  // TODO only if it was passed by ros arg
  log_info(std::string("\tROI set to ") + std::to_string(width_) + "X" +
           std::to_string(height_));
}

void ArenaCameraNode::set_nodes_gain_()
{
  if (is_passed_gain_) {  // not default
    auto nodemap = m_pDevice->GetNodeMap();
    Arena::SetNodeValue<double>(nodemap, "Gain", gain_);
    log_info(std::string("\tGain set to ") + std::to_string(gain_));
  }
}

void ArenaCameraNode::set_nodes_pixelformat_()
{
  auto nodemap = m_pDevice->GetNodeMap();
  // TODO ---------------------------------------------------------------------
  // PIXEL FORMAT HANDLEING

  if (is_passed_pixelformat_ros_) {
    pixelformat_pfnc_ = K_ROS2_PIXELFORMAT_TO_PFNC[pixelformat_ros_];
    if (pixelformat_pfnc_.empty()) {
      throw std::invalid_argument("pixelformat is not supported!");
    }

    try {
      Arena::SetNodeValue<GenICam::gcstring>(nodemap, "PixelFormat",
                                             pixelformat_pfnc_.c_str());
      log_info(std::string("\tPixelFormat set to ") + pixelformat_pfnc_);

    } catch (GenICam::GenericException& e) {
      // TODO
      // an rcl expectation might be expected
      auto x = std::string("pixelformat is not supported by this camera");
      x.append(e.what());
      throw std::invalid_argument(x);
    }
  } else {
    pixelformat_pfnc_ =
        Arena::GetNodeValue<GenICam::gcstring>(nodemap, "PixelFormat");
    pixelformat_ros_ = K_PFNC_TO_ROS2_PIXELFORMAT[pixelformat_pfnc_];

    if (pixelformat_ros_.empty()) {
      log_warn(
          "the device current pixelfromat value is not supported by ROS2. "
          "please use --ros-args -p pixelformat:=\"<supported pixelformat>\".");
      // TODO
      // print list of supported pixelformats
    }
  }
}

void ArenaCameraNode::set_nodes_exposure_()
{
  if (is_passed_exposure_time_) {
    auto nodemap = m_pDevice->GetNodeMap();
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "ExposureAuto", "Off");
    Arena::SetNodeValue<double>(nodemap, "ExposureTime", exposure_time_);
  }
}

void ArenaCameraNode::set_nodes_trigger_mode_()
{
  auto nodemap = m_pDevice->GetNodeMap();
  if (trigger_mode_=="On") {
    if (exposure_time_ < 0) {
      log_warn(
          "\tavoid long waits wating for triggered images by providing proper "
          "exposure_time.");
    }
    // Enable trigger mode before setting the source and selector
    // and before starting the stream. Trigger mode cannot be turned
    // on and off while the device is streaming.

    // Make sure Trigger Mode set to 'Off' after finishing this example
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerMode", "On");

    // Set the trigger source to software in order to trigger buffers
    // without the use of any additional hardware.
    // Lines of the GPIO can also be used to trigger.
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerSource",
                                           "Software");
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerSelector",
                                           "FrameStart");
    auto msg =
        std::string(
            "\ttrigger_mode is activated. To trigger an image run `ros2 run ") +
        this->get_name() + " trigger_image`";
    log_warn(msg);
  }
  // unset device from being in trigger mode if user did not pass trigger
  // mode parameter because the trigger nodes are not rest when loading
  // the user default profile
  else {
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerMode", "Off");
  }
}

// just for debugging
void ArenaCameraNode::set_nodes_test_pattern_image_()
{
  auto nodemap = m_pDevice->GetNodeMap();
  Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TestPattern", "Pattern3");
}

// Added

bool ArenaCameraNode::setAcquisitionFrameRateEnableValue(const bool& target_acquisition_frame_rate_enable, bool& reached_acquisition_frame_rate_enable)
{
  try
  {
    GenApi::CBooleanPtr pAcquisitionFrameRateEnable = m_pDevice->GetNodeMap()->GetNode("AcquisitionFrameRateEnable");
    if (GenApi::IsWritable(pAcquisitionFrameRateEnable))
    {
      bool acquisition_frame_rate_enable_to_set = target_acquisition_frame_rate_enable;
      pAcquisitionFrameRateEnable->SetValue(acquisition_frame_rate_enable_to_set);
      reached_acquisition_frame_rate_enable = pAcquisitionFrameRateEnable->GetValue();
    }
    else
    {
      RCLCPP_WARN_STREAM(this->get_logger(), "Camera does not support AcquisitionFrameRateEnable. Will keep the current settings");
      reached_acquisition_frame_rate_enable = pAcquisitionFrameRateEnable->GetValue();
    }
  }

  catch (const GenICam::GenericException& e)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "An exception while setting target AcquisitionFrameRateEnable to " << std::to_string(target_acquisition_frame_rate_enable) << " occurred: " << e.GetDescription());
    return false;
  }
  return true;
}

bool ArenaCameraNode::setAcquisitionFrameRateEnable(const bool& target_acquisition_frame_rate_enable, bool& reached_acquisition_frame_rate_enable)
{
  boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);

  if (!setAcquisitionFrameRateEnableValue(target_acquisition_frame_rate_enable, reached_acquisition_frame_rate_enable))
  {
    // retry till timeout
    rclcpp::Rate r(10.0);
    auto timeout = this->now() + rclcpp::Duration::from_seconds(2.0);
    while (rclcpp::ok())
    {
      if (setAcquisitionFrameRateEnableValue(target_acquisition_frame_rate_enable, reached_acquisition_frame_rate_enable))
      {
        break;
      }
      if (this->now() > timeout)
      {
        RCLCPP_ERROR(this->get_logger(), "Error in setAcquisitionFrameRateEnable(): Unable to set target AcquisitionFrameRateEnable before timeout");
        return false;
      }
      r.sleep();
    }
  }
  return true;
}

bool ArenaCameraNode::setAcquisitionFrameRateValue(const float& target_acquisition_frame_rate, float& reached_acquisition_frame_rate)
{
  try
  {
    GenApi::CFloatPtr pAcquisitionFrameRate = m_pDevice->GetNodeMap()->GetNode("AcquisitionFrameRate");
    if (GenApi::IsWritable(pAcquisitionFrameRate))
    {
      float acquisition_frame_rate_to_set = target_acquisition_frame_rate;
      if (acquisition_frame_rate_to_set < pAcquisitionFrameRate->GetMin())
      {
        RCLCPP_WARN(this->get_logger(), "Desired AcquisitionFrameRate '%.2f' unreachable! Setting to lower limit: %.2f", 
                    acquisition_frame_rate_to_set, pAcquisitionFrameRate->GetMin());
        acquisition_frame_rate_to_set = pAcquisitionFrameRate->GetMin();
      }
      else if (acquisition_frame_rate_to_set > pAcquisitionFrameRate->GetMax())
      {
        RCLCPP_WARN(this->get_logger(), "Desired AcquisitionFrameRate '%.2f' unreachable! Setting to upper limit: %.2f", 
                    acquisition_frame_rate_to_set, pAcquisitionFrameRate->GetMax());
        acquisition_frame_rate_to_set = pAcquisitionFrameRate->GetMax();
      }
      pAcquisitionFrameRate->SetValue(acquisition_frame_rate_to_set);
      reached_acquisition_frame_rate = pAcquisitionFrameRate->GetValue();
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Camera does not support AcquisitionFrameRate. Will keep the current settings");
      reached_acquisition_frame_rate = pAcquisitionFrameRate->GetValue();
    }
  }
  catch (const GenICam::GenericException& e)
  {
    RCLCPP_ERROR(this->get_logger(), "An exception while setting target AcquisitionFrameRate to %.2f occurred: %s", 
                 target_acquisition_frame_rate, e.GetDescription());
    return false;
  }
  return true;
}


bool ArenaCameraNode::setAcquisitionFrameRate(const float& target_acquisition_frame_rate, float& reached_acquisition_frame_rate)
{
  boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);

  if (!setAcquisitionFrameRateValue(target_acquisition_frame_rate, reached_acquisition_frame_rate))
  {
    // retry till timeout
    rclcpp::Rate r(10.0);
    auto timeout = this->now() + rclcpp::Duration::from_seconds(2.0);
    while (rclcpp::ok())
    {
      if (setAcquisitionFrameRateValue(target_acquisition_frame_rate, reached_acquisition_frame_rate))
      {
        break;
      }
      if (this->now() > timeout)
      {
        RCLCPP_ERROR(this->get_logger(), "Error in setAcquisitionFrameRate(): Unable to set target AcquisitionFrameRate before timeout");
        return false;
      }
      r.sleep();
    }
  }
  return true;
}

bool ArenaCameraNode::setAcquisitionModeValue(const std::string& target_acquisition_mode, std::string& reached_acquisition_mode)
{
  try
  {
    GenApi::CEnumerationPtr pAcquisitionMode = m_pDevice->GetNodeMap()->GetNode("AcquisitionMode");
    if (GenApi::IsWritable(pAcquisitionMode))
    {
      if (target_acquisition_mode == "SingleFrame" || target_acquisition_mode == "MultiFrame" ||
          target_acquisition_mode == "Continuous")
      {
        GenICam::gcstring acquisition_mode_to_set = target_acquisition_mode.c_str();
        Arena::SetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(), "AcquisitionMode", acquisition_mode_to_set);
        reached_acquisition_mode = Arena::GetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(), "AcquisitionMode").c_str();
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Camera does not support the value '%s' for AcquisitionMode. Will keep the current settings",
                    target_acquisition_mode.c_str());
        reached_acquisition_mode = Arena::GetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(), "AcquisitionMode").c_str();
      }
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Camera does not support AcquisitionMode. Will keep the current settings");
      reached_acquisition_mode = Arena::GetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(), "AcquisitionMode").c_str();
    }
  }
  catch (const GenICam::GenericException& e)
  {
    RCLCPP_ERROR(this->get_logger(), "An exception while setting target AcquisitionMode to %s occurred: %s",
                 target_acquisition_mode.c_str(), e.GetDescription());
    return false;
  }
  return true;
}

bool ArenaCameraNode::setAcquisitionMode(const std::string& target_acquisition_mode, std::string& reached_acquisition_mode)
{
  boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);

  if (!setAcquisitionModeValue(target_acquisition_mode, reached_acquisition_mode))
  {
    // retry till timeout
    rclcpp::Rate r(10.0);
    auto timeout = this->now() + rclcpp::Duration::from_seconds(2.0);
    while (rclcpp::ok())
    {
      if (setAcquisitionModeValue(target_acquisition_mode, reached_acquisition_mode))
      {
        break;
      }
      if (this->now() > timeout)
      {
        RCLCPP_ERROR(this->get_logger(), "Error in setAcquisitionMode(): Unable to set target AcquisitionMode before timeout");
        return false;
      }
      r.sleep();
    }
  }
  return true;
}


bool ArenaCameraNode::setScan3dSpatialFilterEnableValue(const bool& target_scan_3d_spatial_filter_enable, bool& reached_scan_3d_spatial_filter_enable)
{
  try
  {
    GenApi::CBooleanPtr pScan3dSpatialFilterEnable = m_pDevice->GetNodeMap()->GetNode("Scan3dSpatialFilterEnable");
    if (GenApi::IsWritable(pScan3dSpatialFilterEnable))
    {
      bool scan_3d_spatial_filter_enable_to_set = target_scan_3d_spatial_filter_enable;
      pScan3dSpatialFilterEnable->SetValue(scan_3d_spatial_filter_enable_to_set);
      reached_scan_3d_spatial_filter_enable = pScan3dSpatialFilterEnable->GetValue();
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Camera does not support Scan3dSpatialFilterEnable. Will keep the current settings");
      reached_scan_3d_spatial_filter_enable = pScan3dSpatialFilterEnable->GetValue();
    }
  }
  catch (const GenICam::GenericException& e)
  {
    RCLCPP_ERROR(this->get_logger(), "An exception while setting target Scan3dSpatialFilterEnable to %s occurred: %s",
                 std::to_string(target_scan_3d_spatial_filter_enable).c_str(), e.GetDescription());
    return false;
  }
  return true;
}

bool ArenaCameraNode::setScan3dSpatialFilterEnable(const bool& target_scan_3d_spatial_filter_enable, bool& reached_scan_3d_spatial_filter_enable)
{
  boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);

  if (!setScan3dSpatialFilterEnableValue(target_scan_3d_spatial_filter_enable, reached_scan_3d_spatial_filter_enable))
  {
    // retry till timeout
    rclcpp::Rate r(10.0);
    auto timeout = this->now() + rclcpp::Duration::from_seconds(2.0);
    while (rclcpp::ok())
    {
      if (setScan3dSpatialFilterEnableValue(target_scan_3d_spatial_filter_enable, reached_scan_3d_spatial_filter_enable))
      {
        break;
      }
      if (this->now() > timeout)
      {
        RCLCPP_ERROR(this->get_logger(), "Error in setScan3dSpatialFilterEnable(): Unable to set target Scan3dSpatialFilterEnable before timeout");
        return false;
      }
      r.sleep();
    }
  }
  return true;
}

bool ArenaCameraNode::setScan3dFlyingPixelsRemovalEnableValue(const bool& target_scan_3d_flying_pixels_removal_enable, bool& reached_scan_3d_flying_pixels_removal_enable)
{
  try
  {
    GenApi::CBooleanPtr pScan3dFlyingPixelsRemovalEnable = m_pDevice->GetNodeMap()->GetNode("Scan3dFlyingPixelsRemovalEnable");
    if (GenApi::IsWritable(pScan3dFlyingPixelsRemovalEnable))
    {
      bool scan_3d_flying_pixels_removal_enable_to_set = target_scan_3d_flying_pixels_removal_enable;
      pScan3dFlyingPixelsRemovalEnable->SetValue(scan_3d_flying_pixels_removal_enable_to_set);
      reached_scan_3d_flying_pixels_removal_enable = pScan3dFlyingPixelsRemovalEnable->GetValue();
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Camera does not support Scan3dFlyingPixelsRemovalEnable. Will keep the current settings");
      reached_scan_3d_flying_pixels_removal_enable = pScan3dFlyingPixelsRemovalEnable->GetValue();
    }
  }
  catch (const GenICam::GenericException& e)
  {
    RCLCPP_ERROR(this->get_logger(), "An exception while setting target Scan3dFlyingPixelsRemovalEnable to %s occurred: %s",
                 std::to_string(target_scan_3d_flying_pixels_removal_enable).c_str(), e.GetDescription());
    return false;
  }
  return true;
}

bool ArenaCameraNode::setScan3dFlyingPixelsRemovalEnable(const bool& target_scan_3d_flying_pixels_removal_enable, bool& reached_scan_3d_flying_pixels_removal_enable)
{
  boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);

  if (!setScan3dFlyingPixelsRemovalEnableValue(target_scan_3d_flying_pixels_removal_enable, reached_scan_3d_flying_pixels_removal_enable))
  {
    // retry till timeout
    rclcpp::Rate r(10.0);
    auto timeout = this->now() + rclcpp::Duration::from_seconds(2.0);
    while (rclcpp::ok())
    {
      if (setScan3dFlyingPixelsRemovalEnableValue(target_scan_3d_flying_pixels_removal_enable, reached_scan_3d_flying_pixels_removal_enable))
      {
        break;
      }
      if (this->now() > timeout)
      {
        RCLCPP_ERROR(this->get_logger(), "Error in setScan3dFlyingPixelsRemovalEnable(): Unable to set target Scan3dFlyingPixelsRemovalEnable before timeout");
        return false;
      }
      r.sleep();
    }
  }
  return true;
}
/*
bool ArenaCameraNode::setScan3dFlyingPixelsDistanceThresholdValue(const int& target_scan_3d_flying_pixels_distance_threshold, int& reached_scan_3d_flying_pixels_distance_threshold)
{
  try
  {
    GenApi::CIntegerPtr pScan3dFlyingPixelsDistanceThreshold = m_pDevice->GetNodeMap()->GetNode("Scan3dFlyingPixelsDistanceThreshold");
    if (GenApi::IsWritable(pScan3dFlyingPixelsDistanceThreshold))
    {
      int scan_3d_flying_pixels_distance_to_set = target_scan_3d_flying_pixels_distance_threshold;
      if (scan_3d_flying_pixels_distance_to_set < pScan3dFlyingPixelsDistanceThreshold->GetMin())
      {
        RCLCPP_WARN(this->get_logger(), "Desired Scan3dFlyingPixelsDistanceThreshold '%d' unreachable! Setting to lower limit: %d",
                    scan_3d_flying_pixels_distance_to_set, pScan3dFlyingPixelsDistanceThreshold->GetMin());
        scan_3d_flying_pixels_distance_to_set = pScan3dFlyingPixelsDistanceThreshold->GetMin();
      }
      else if (scan_3d_flying_pixels_distance_to_set > pScan3dFlyingPixelsDistanceThreshold->GetMax())
      {
        RCLCPP_WARN(this->get_logger(), "Desired Scan3dFlyingPixelsDistanceThreshold '%d' unreachable! Setting to upper limit: %d",
                    scan_3d_flying_pixels_distance_to_set, pScan3dFlyingPixelsDistanceThreshold->GetMax());
        scan_3d_flying_pixels_distance_to_set = pScan3dFlyingPixelsDistanceThreshold->GetMax();
      }
      pScan3dFlyingPixelsDistanceThreshold->SetValue(scan_3d_flying_pixels_distance_to_set);
      reached_scan_3d_flying_pixels_distance_threshold = pScan3dFlyingPixelsDistanceThreshold->GetValue();
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Camera does not support Scan3dFlyingPixelsDistanceThreshold. Will keep the current settings");
      reached_scan_3d_flying_pixels_distance_threshold = pScan3dFlyingPixelsDistanceThreshold->GetValue();
    }
  }
  catch (const GenICam::GenericException& e)
  {
    RCLCPP_ERROR(this->get_logger(), "An exception while setting target Scan3dFlyingPixelsDistanceThreshold to %d occurred: %s",
                 target_scan_3d_flying_pixels_distance_threshold, e.GetDescription());
    return false;
  }
  return true;
}


bool ArenaCameraNode::setScan3dFlyingPixelsDistanceThreshold(const int& target_scan_3d_flying_pixels_distance_threshold, int& reached_scan_3d_flying_pixels_distance_threshold)
{
  boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);

  if (!setScan3dFlyingPixelsDistanceThresholdValue(target_scan_3d_flying_pixels_distance_threshold, reached_scan_3d_flying_pixels_distance_threshold))
  {
    // retry till timeout
    rclcpp::Rate r(10.0);
    auto timeout = this->now() + rclcpp::Duration::from_seconds(2.0);
    while (rclcpp::ok())
    {
      if (setScan3dFlyingPixelsDistanceThresholdValue(target_scan_3d_flying_pixels_distance_threshold, reached_scan_3d_flying_pixels_distance_threshold))
      {
        break;
      }
      if (this->now() > timeout)
      {
        RCLCPP_ERROR(this->get_logger(), "Error in setScan3dFlyingPixelsDistanceThreshold(): Unable to set target Scan3dFlyingPixelsDistanceThreshold before timeout");
        return false;
      }
      r.sleep();
    }
  }
  return true;
}
*/
bool ArenaCameraNode::setExposureTimeSelectorValue(const std::string& target_exposure_time_selector, std::string& reached_exposure_time_selector)
{
  try
  {
    GenApi::CEnumerationPtr pExposureTimeSelector = m_pDevice->GetNodeMap()->GetNode("ExposureTimeSelector");
    if (GenApi::IsWritable(pExposureTimeSelector))
    {
      if(target_exposure_time_selector == "Exp1000Us" || target_exposure_time_selector == "Exp250Us" || target_exposure_time_selector == "Exp62_5Us")
      {
        GenICam::gcstring exposure_time_selector_to_set = target_exposure_time_selector.c_str();
        Arena::SetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(), "ExposureTimeSelector", exposure_time_selector_to_set);
        reached_exposure_time_selector = Arena::GetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(), "ExposureTimeSelector").c_str();
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Camera does not support the value '%s' for ExposureTimeSelector. Will keep the current settings",
                    target_exposure_time_selector.c_str());
        reached_exposure_time_selector = Arena::GetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(), "ExposureTimeSelector").c_str();
      }
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Camera does not support ExposureTimeSelector. Will keep the current settings");
      reached_exposure_time_selector = Arena::GetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(), "ExposureTimeSelector").c_str();
    }
  }
  catch (const GenICam::GenericException& e)
  {
    RCLCPP_ERROR(this->get_logger(), "An exception while setting target ExposureTimeSelector to %s occurred: %s",
                 target_exposure_time_selector.c_str(), e.GetDescription());
    return false;
  }
  return true;
}

bool ArenaCameraNode::setExposureTimeSelector(const std::string& target_exposure_time_selector, std::string& reached_exposure_time_selector)
{
  boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);

  if (!setExposureTimeSelectorValue(target_exposure_time_selector, reached_exposure_time_selector))
  {
    // retry till timeout
    rclcpp::Rate r(10.0);
    auto timeout = this->now() + rclcpp::Duration::from_seconds(2.0);
    while (rclcpp::ok())
    {
      if (setExposureTimeSelectorValue(target_exposure_time_selector, reached_exposure_time_selector))
      {
        break;
      }
      if (this->now() > timeout)
      {
        RCLCPP_ERROR(this->get_logger(), "Error in setExposureTimeSelector(): Unable to set target ExposureTimeSelector before timeout");
        return false;
      }
      r.sleep();
    }
  }
  return true;
}

bool ArenaCameraNode::setScan3dOperatingModeValue(const std::string& target_scan_3d_operating_mode, std::string& reached_scan_3d_operating_mode)
{
  try
  {
    GenApi::CEnumerationPtr pScan3dOperatingMode = m_pDevice->GetNodeMap()->GetNode("Scan3dOperatingMode");
    if (GenApi::IsWritable(pScan3dOperatingMode))
    {
      if(target_scan_3d_operating_mode == "Distance1250mmSingleFreq" || target_scan_3d_operating_mode == "Distance3000mmSingleFreq" ||
      target_scan_3d_operating_mode == "Distance4000mmSingleFreq" || target_scan_3d_operating_mode == "Distance5000mmMultiFreq" ||
      target_scan_3d_operating_mode == "Distance6000mmSingleFreq" || target_scan_3d_operating_mode == "Distance8300mmMultiFreq" ||
      target_scan_3d_operating_mode == "HighSpeedDistance625mmSingleFreq" || target_scan_3d_operating_mode == "HighSpeedDistance1250mmSingleFreq" ||
      target_scan_3d_operating_mode == "HighSpeedDistance2500mmSingleFreq")
      {
        GenICam::gcstring scan_3d_operating_mode_to_set = target_scan_3d_operating_mode.c_str();
        Arena::SetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(), "Scan3dOperatingMode", scan_3d_operating_mode_to_set);
        reached_scan_3d_operating_mode = Arena::GetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(), "Scan3dOperatingMode").c_str();
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Camera does not support the value '%s' for Scan3dOperatingMode. Will keep the current settings",
                    target_scan_3d_operating_mode.c_str());
        reached_scan_3d_operating_mode = Arena::GetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(), "Scan3dOperatingMode").c_str();
      }
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Camera does not support Scan3dOperatingMode. Will keep the current settings");
      reached_scan_3d_operating_mode = Arena::GetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(), "Scan3dOperatingMode").c_str();
    }
  }
  catch (const GenICam::GenericException& e)
  {
    RCLCPP_ERROR(this->get_logger(), "An exception while setting target Scan3dOperatingMode to %s occurred: %s",
                 target_scan_3d_operating_mode.c_str(), e.GetDescription());
    return false;
  }
  return true;
}

bool ArenaCameraNode::setScan3dOperatingMode(const std::string& target_scan_3d_operating_mode, std::string& reached_scan_3d_operating_mode)
{
  boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);

  if (!setScan3dOperatingModeValue(target_scan_3d_operating_mode, reached_scan_3d_operating_mode))
  {
    // retry till timeout
    rclcpp::Rate r(10.0);
    auto timeout = this->now() + rclcpp::Duration::from_seconds(2.0);
    while (rclcpp::ok())
    {
      if (setScan3dOperatingModeValue(target_scan_3d_operating_mode, reached_scan_3d_operating_mode))
      {
        break;
      }
      if (this->now() > timeout)
      {
        RCLCPP_ERROR(this->get_logger(), "Error in setScan3dOperatingMode(): Unable to set target Scan3dOperatingMode before timeout");
        return false;
      }
      r.sleep();
    }
  }
  return true;
}

bool ArenaCameraNode::setScan3dConfidenceThresholdEnableValue(const bool& target_scan_3d_confidence_threshold_enable, bool& reached_scan_3d_confidence_threshold_enable)
{
  try
  {
    GenApi::CBooleanPtr pScan3dConfidenceThresholdEnable = m_pDevice->GetNodeMap()->GetNode("Scan3dConfidenceThresholdEnable");
    if (GenApi::IsWritable(pScan3dConfidenceThresholdEnable))
    {
      bool scan_3d_confidence_threshold_enable_to_set = target_scan_3d_confidence_threshold_enable;
      pScan3dConfidenceThresholdEnable->SetValue(scan_3d_confidence_threshold_enable_to_set);
      reached_scan_3d_confidence_threshold_enable = pScan3dConfidenceThresholdEnable->GetValue();
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Camera does not support Scan3dConfidenceThresholdEnable. Will keep the current settings");
      reached_scan_3d_confidence_threshold_enable = pScan3dConfidenceThresholdEnable->GetValue();
    }
  }
  catch (const GenICam::GenericException& e)
  {
    RCLCPP_ERROR(this->get_logger(), "An exception while setting target Scan3dConfidenceThresholdEnable to %s occurred: %s",
                 std::to_string(target_scan_3d_confidence_threshold_enable).c_str(), e.GetDescription());
    return false;
  }
  return true;
}

bool ArenaCameraNode::setScan3dConfidenceThresholdEnable(const bool& target_scan_3d_confidence_threshold_enable, bool& reached_scan_3d_confidence_threshold_enable)
{
  boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);

  if (!setScan3dConfidenceThresholdEnableValue(target_scan_3d_confidence_threshold_enable, reached_scan_3d_confidence_threshold_enable))
  {
    // retry till timeout
    rclcpp::Rate r(10.0);
    auto timeout = this->now() + rclcpp::Duration::from_seconds(2.0);
    while (rclcpp::ok())
    {
      if (setScan3dConfidenceThresholdEnableValue(target_scan_3d_confidence_threshold_enable, reached_scan_3d_confidence_threshold_enable))
      {
        break;
      }
      if (this->now() > timeout)
      {
        RCLCPP_ERROR(this->get_logger(), "Error in setScan3dConfidenceThresholdEnable(): Unable to set target Scan3dConfidenceThresholdEnable before timeout");
        return false;
      }
      r.sleep();
    }
  }
  return true;
}

bool ArenaCameraNode::setScan3dConfidenceThresholdMinValue(const int& target_scan_3d_confidence_threshold_min, int& reached_scan_3d_confidence_threshold_min)
{
  try
  {
    GenApi::CIntegerPtr pScan3dConfidenceThresholdMin = m_pDevice->GetNodeMap()->GetNode("Scan3dConfidenceThresholdMin");
    if (GenApi::IsWritable(pScan3dConfidenceThresholdMin))
    {
      int scan_3d_confidence_threshold_min_to_set = target_scan_3d_confidence_threshold_min;
      if (scan_3d_confidence_threshold_min_to_set < pScan3dConfidenceThresholdMin->GetMin())
      {
        RCLCPP_WARN(this->get_logger(), "Desired Scan3dConfidenceThresholdMin '%d' unreachable! Setting to lower limit: %d",
                    scan_3d_confidence_threshold_min_to_set, pScan3dConfidenceThresholdMin->GetMin());
        scan_3d_confidence_threshold_min_to_set = pScan3dConfidenceThresholdMin->GetMin();
      }
      else if (scan_3d_confidence_threshold_min_to_set > pScan3dConfidenceThresholdMin->GetMax())
      {
        RCLCPP_WARN(this->get_logger(), "Desired Scan3dConfidenceThresholdMin '%d' unreachable! Setting to upper limit: %d",
                    scan_3d_confidence_threshold_min_to_set, pScan3dConfidenceThresholdMin->GetMax());
        scan_3d_confidence_threshold_min_to_set = pScan3dConfidenceThresholdMin->GetMax();
      }
      pScan3dConfidenceThresholdMin->SetValue(scan_3d_confidence_threshold_min_to_set);
      reached_scan_3d_confidence_threshold_min = pScan3dConfidenceThresholdMin->GetValue();
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Camera does not support Scan3dConfidenceThresholdMin. Will keep the current settings");
      reached_scan_3d_confidence_threshold_min = pScan3dConfidenceThresholdMin->GetValue();
    }
  }
  catch (const GenICam::GenericException& e)
  {
    RCLCPP_ERROR(this->get_logger(), "An exception while setting target Scan3dConfidenceThresholdMin to %d occurred: %s",
                 target_scan_3d_confidence_threshold_min, e.GetDescription());
    return false;
  }
  return true;
}

bool ArenaCameraNode::setScan3dConfidenceThresholdMin(const int& target_scan_3d_confidence_threshold_min, int& reached_scan_3d_confidence_threshold_min)
{
  boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);

  if (!setScan3dConfidenceThresholdMinValue(target_scan_3d_confidence_threshold_min, reached_scan_3d_confidence_threshold_min))
  {
    // retry till timeout
    rclcpp::Rate r(10.0);
    auto timeout = this->now() + rclcpp::Duration::from_seconds(2.0);
    while (rclcpp::ok())
    {
      if (setScan3dConfidenceThresholdMinValue(target_scan_3d_confidence_threshold_min, reached_scan_3d_confidence_threshold_min))
      {
        break;
      }
      if (this->now() > timeout)
      {
        RCLCPP_ERROR(this->get_logger(), "Error in setScan3dConfidenceThresholdMin(): Unable to set target Scan3dConfidenceThresholdMin before timeout");
        return false;
      }
      r.sleep();
    }
  }
  return true;
}
/*
  bool ArenaCameraNode::setScan3dHDRModeValue(const std::string& target_scan_3d_hdr_mode, std::string& reached_scan_3d_hdr_mode)
  {
    try
    {
      GenApi::CEnumerationPtr pScan3dHDRMode = m_pDevice->GetNodeMap()->GetNode("Scan3dHDRMode");
      if (GenApi::IsWritable(pScan3dHDRMode))
      {
        if(target_scan_3d_hdr_mode == "Off" || target_scan_3d_hdr_mode == "StandardHDR" ||
          target_scan_3d_hdr_mode == "LowNoiseHDRX4" || target_scan_3d_hdr_mode == "LowNoiseHDRX8" ||
          target_scan_3d_hdr_mode == "LowNoiseHDRX16" || target_scan_3d_hdr_mode == "LowNoiseHDRX32")
        {
          GenICam::gcstring scan_3d_hdr_mode_to_set = target_scan_3d_hdr_mode.c_str();
          Arena::SetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(), "Scan3dHDRMode", scan_3d_hdr_mode_to_set);
          reached_scan_3d_hdr_mode = Arena::GetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(), "Scan3dHDRMode").c_str();
        }
        else
        {
          RCLCPP_WARN(this->get_logger(), "Camera does not support the value '%s' for Scan3dHDRMode. Will keep the current settings",
                      target_scan_3d_hdr_mode.c_str());
          reached_scan_3d_hdr_mode = Arena::GetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(), "Scan3dHDRMode").c_str();
        }
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Camera does not support Scan3dHDRMode. Will keep the current settings");
        reached_scan_3d_hdr_mode = Arena::GetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(), "Scan3dHDRMode").c_str();
      }
    }
    catch (const GenICam::GenericException& e)
    {
      RCLCPP_ERROR(this->get_logger(), "An exception while setting target Scan3dHDRMode to %s occurred: %s",
                  target_scan_3d_hdr_mode.c_str(), e.GetDescription());
      return false;
    }
    return true;
  }

bool ArenaCameraNode::setScan3dHDRMode(const std::string& target_scan_3d_hdr_mode, std::string& reached_scan_3d_hdr_mode)
{
  boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);

  if (!setScan3dHDRModeValue(target_scan_3d_hdr_mode, reached_scan_3d_hdr_mode))
  {
    // retry till timeout
    rclcpp::Rate r(10.0);
    auto timeout = this->now() + rclcpp::Duration::from_seconds(2.0);
    while (rclcpp::ok())
    {
      if (setScan3dHDRModeValue(target_scan_3d_hdr_mode, reached_scan_3d_hdr_mode))
      {
        break;
      }
      if (this->now() > timeout)
      {
        RCLCPP_ERROR(this->get_logger(), "Error in setScan3dHDRMode(): Unable to set target Scan3dHDRMode before timeout");
        return false;
      }
      r.sleep();
    }
  }
  return true;
}
*/
bool ArenaCameraNode::setScan3dModeSelectorValue(const std::string& target_scan_3d_mode_selector, std::string& reached_scan_3d_mode_selector)
{
  try
  {
    GenApi::CEnumerationPtr pScan3dModeSelector = m_pDevice->GetNodeMap()->GetNode("Scan3dModeSelector");
    if (GenApi::IsWritable(pScan3dModeSelector))
    {
      if(target_scan_3d_mode_selector == "Processed" || target_scan_3d_mode_selector == "Raw")
      {
        GenICam::gcstring scan_3d_mode_selector_to_set = target_scan_3d_mode_selector.c_str();
        Arena::SetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(), "Scan3dModeSelector", scan_3d_mode_selector_to_set);
        reached_scan_3d_mode_selector = Arena::GetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(), "Scan3dModeSelector").c_str();
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Camera does not support the value '%s' for Scan3dModeSelector. Will keep the current settings",
                    target_scan_3d_mode_selector.c_str());
        reached_scan_3d_mode_selector = Arena::GetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(), "Scan3dModeSelector").c_str();
      }
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Camera does not support Scan3dModeSelector. Will keep the current settings");
      reached_scan_3d_mode_selector = Arena::GetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(), "Scan3dModeSelector").c_str();
    }
  }
  catch (const GenICam::GenericException& e)
  {
    RCLCPP_ERROR(this->get_logger(), "An exception while setting target Scan3dModeSelector to %s occurred: %s",
                 target_scan_3d_mode_selector.c_str(), e.GetDescription());
    return false;
  }
  return true;
}

bool ArenaCameraNode::setScan3dModeSelector(const std::string& target_scan_3d_mode_selector, std::string& reached_scan_3d_mode_selector)
{
  boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);

  if (!setScan3dModeSelectorValue(target_scan_3d_mode_selector, reached_scan_3d_mode_selector))
  {
    // retry till timeout
    rclcpp::Rate r(10.0);
    auto timeout = this->now() + rclcpp::Duration::from_seconds(2.0);
    while (rclcpp::ok())
    {
      if (setScan3dModeSelectorValue(target_scan_3d_mode_selector, reached_scan_3d_mode_selector))
      {
        break;
      }
      if (this->now() > timeout)
      {
        RCLCPP_ERROR(this->get_logger(), "Error in setScan3dModeSelector(): Unable to set target Scan3dModeSelector before timeout");
        return false;
      }
      r.sleep();
    }
  }
  return true;
}

bool ArenaCameraNode::setTriggerSelectorValue(const std::string& target_trigger_selector, std::string& reached_trigger_selector)
{
  try
  {
    GenApi::CEnumerationPtr pTriggerSelector = m_pDevice->GetNodeMap()->GetNode("TriggerSelector");
    if (GenApi::IsWritable(pTriggerSelector))
    {
      if(target_trigger_selector == "AcquisitionStart" || target_trigger_selector == "FrameStart" ||
         target_trigger_selector == "FrameBurstStart" || target_trigger_selector == "ExposureActive" ||
         target_trigger_selector == "LineStart" || target_trigger_selector == "FrameBurstActive")
      {
        GenICam::gcstring trigger_selector_to_set = target_trigger_selector.c_str();
        Arena::SetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(), "TriggerSelector", trigger_selector_to_set);
        reached_trigger_selector = Arena::GetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(), "TriggerSelector").c_str();
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Camera does not support the value '%s' for TriggerSelector. Will keep the current settings",
                    target_trigger_selector.c_str());
        reached_trigger_selector = Arena::GetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(), "TriggerSelector").c_str();
      }
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Camera does not support TriggerSelector. Will keep the current settings");
      reached_trigger_selector = Arena::GetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(), "TriggerSelector").c_str();
    }
  }
  catch (const GenICam::GenericException& e)
  {
    RCLCPP_ERROR(this->get_logger(), "An exception while setting target TriggerSelector to %s occurred: %s",
                 target_trigger_selector.c_str(), e.GetDescription());
    return false;
  }
  return true;
}

bool ArenaCameraNode::setTriggerSelector(const std::string& target_trigger_selector, std::string& reached_trigger_selector)
{
  boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);

  if (!setTriggerSelectorValue(target_trigger_selector, reached_trigger_selector))
  {
    // retry till timeout
    rclcpp::Rate r(10.0);
    auto timeout = this->now() + rclcpp::Duration::from_seconds(2.0);
    while (rclcpp::ok())
    {
      if (setTriggerSelectorValue(target_trigger_selector, reached_trigger_selector))
      {
        break;
      }
      if (this->now() > timeout)
      {
        RCLCPP_ERROR(this->get_logger(), "Error in setTriggerSelector(): Unable to set target TriggerSelector before timeout");
        return false;
      }
      r.sleep();
    }
  }
  return true;
}

bool ArenaCameraNode::setTriggerModeValue(const std::string& target_trigger_mode, std::string& reached_trigger_mode)
{
  try
  {
    GenApi::CEnumerationPtr pTriggerMode = m_pDevice->GetNodeMap()->GetNode("TriggerMode");
    if (GenApi::IsWritable(pTriggerMode))
    {
      if(target_trigger_mode == "Off" || target_trigger_mode == "On")
      {
        GenICam::gcstring trigger_mode_to_set = target_trigger_mode.c_str();
        Arena::SetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(), "TriggerMode", trigger_mode_to_set);
        reached_trigger_mode = Arena::GetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(), "TriggerMode").c_str();
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Camera does not support the value '%s' for TriggerMode. Will keep the current settings",
                    target_trigger_mode.c_str());
        reached_trigger_mode = Arena::GetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(), "TriggerMode").c_str();
      }
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Camera does not support TriggerMode. Will keep the current settings");
      reached_trigger_mode = Arena::GetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(), "TriggerMode").c_str();
    }
  }
  catch (const GenICam::GenericException& e)
  {
    RCLCPP_ERROR(this->get_logger(), "An exception while setting target TriggerMode to %s occurred: %s",
                 target_trigger_mode.c_str(), e.GetDescription());
    return false;
  }
  return true;
}

bool ArenaCameraNode::setTriggerMode(const std::string& target_trigger_mode, std::string& reached_trigger_mode)
{
  boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);

  if (!setTriggerModeValue(target_trigger_mode, reached_trigger_mode))
  {
    // retry till timeout
    rclcpp::Rate r(10.0);
    auto timeout = this->now() + rclcpp::Duration::from_seconds(2.0);
    while (rclcpp::ok())
    {
      if (setTriggerModeValue(target_trigger_mode, reached_trigger_mode))
      {
        break;
      }
      if (this->now() > timeout)
      {
        RCLCPP_ERROR(this->get_logger(), "Error in setTriggerMode(): Unable to set target TriggerMode before timeout");
        return false;
      }
      r.sleep();
    }
  }
  return true;
}

bool ArenaCameraNode::setTriggerSourceValue(const std::string& target_trigger_source, std::string& reached_trigger_source)
{
  try
  {
    GenApi::CEnumerationPtr pTriggerSource = m_pDevice->GetNodeMap()->GetNode("TriggerSource");
    if (GenApi::IsWritable(pTriggerSource))
    {
      if(target_trigger_source == "Software" || target_trigger_source == "Line0" || target_trigger_source == "Line1" ||
         target_trigger_source == "Line2" || target_trigger_source == "Line3" || target_trigger_source == "Action0")
      {
        GenICam::gcstring trigger_source_to_set = target_trigger_source.c_str();
        Arena::SetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(), "TriggerSource", trigger_source_to_set);
        reached_trigger_source = Arena::GetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(), "TriggerSource").c_str();
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Camera does not support the value '%s' for TriggerSource. Will keep the current settings",
                    target_trigger_source.c_str());
        reached_trigger_source = Arena::GetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(), "TriggerSource").c_str();
      }
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Camera does not support TriggerSource. Will keep the current settings");
      reached_trigger_source = Arena::GetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(), "TriggerSource").c_str();
    }
  }
  catch (const GenICam::GenericException& e)
  {
    RCLCPP_ERROR(this->get_logger(), "An exception while setting target TriggerSource to %s occurred: %s",
                 target_trigger_source.c_str(), e.GetDescription());
    return false;
  }
  return true;
}

bool ArenaCameraNode::setTriggerSource(const std::string& target_trigger_source, std::string& reached_trigger_source)
{
  boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);

  if (!setTriggerSourceValue(target_trigger_source, reached_trigger_source))
  {
    // retry till timeout
    rclcpp::Rate r(10.0);
    auto timeout = this->now() + rclcpp::Duration::from_seconds(2.0);
    while (rclcpp::ok())
    {
      if (setTriggerSourceValue(target_trigger_source, reached_trigger_source))
      {
        break;
      }
      if (this->now() > timeout)
      {
        RCLCPP_ERROR(this->get_logger(), "Error in setTriggerSource(): Unable to set target TriggerSource before timeout");
        return false;
      }
      r.sleep();
    }
  }
  return true;
}

bool ArenaCameraNode::setTriggerActivationValue(const std::string& target_trigger_activation, std::string& reached_trigger_activation)
{
  try
  {
    GenApi::CEnumerationPtr pTriggerActivation = m_pDevice->GetNodeMap()->GetNode("TriggerActivation");
    if (GenApi::IsWritable(pTriggerActivation))
    {
      if(target_trigger_activation == "RisingEdge")
      {
        GenICam::gcstring trigger_activation_to_set = target_trigger_activation.c_str();
        Arena::SetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(), "TriggerActivation", trigger_activation_to_set);
        reached_trigger_activation = Arena::GetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(), "TriggerActivation").c_str();
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Camera does not support the value '%s' for TriggerActivation. Will keep the current settings",
                    target_trigger_activation.c_str());
        reached_trigger_activation = Arena::GetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(), "TriggerActivation").c_str();
      }
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Camera does not support TriggerActivation. Will keep the current settings");
      reached_trigger_activation = Arena::GetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(), "TriggerActivation").c_str();
    }
  }
  catch (const GenICam::GenericException& e)
  {
    RCLCPP_ERROR(this->get_logger(), "An exception while setting target TriggerActivation to %s occurred: %s",
                 target_trigger_activation.c_str(), e.GetDescription());
    return false;
  }
  return true;
}

bool ArenaCameraNode::setTriggerActivation(const std::string& target_trigger_activation, std::string& reached_trigger_activation)
{
  boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);

  if (!setTriggerActivationValue(target_trigger_activation, reached_trigger_activation))
  {
    // retry till timeout
    rclcpp::Rate r(10.0);
    auto timeout = this->now() + rclcpp::Duration::from_seconds(2.0);
    while (rclcpp::ok())
    {
      if (setTriggerActivationValue(target_trigger_activation, reached_trigger_activation))
      {
        break;
      }
      if (this->now() > timeout)
      {
        RCLCPP_ERROR(this->get_logger(), "Error in setTriggerActivation(): Unable to set target TriggerActivation before timeout");
        return false;
      }
      r.sleep();
    }
  }
  return true;
}

bool ArenaCameraNode::setTriggerDelayValue(const float& target_trigger_delay, float& reached_trigger_delay)
{
  try
  {
    GenApi::CFloatPtr pTriggerDelay = m_pDevice->GetNodeMap()->GetNode("TriggerDelay");
    if (GenApi::IsWritable(pTriggerDelay))
    {
      float trigger_delay_to_set = target_trigger_delay;
      if (trigger_delay_to_set < pTriggerDelay->GetMin())
      {
        RCLCPP_WARN(this->get_logger(), "Desired TriggerDelay '%f' unreachable! Setting to lower limit: %f",
                    trigger_delay_to_set, pTriggerDelay->GetMin());
        trigger_delay_to_set = pTriggerDelay->GetMin();
      }
      else if (trigger_delay_to_set > pTriggerDelay->GetMax())
      {
        RCLCPP_WARN(this->get_logger(), "Desired TriggerDelay '%f' unreachable! Setting to upper limit: %f",
                    trigger_delay_to_set, pTriggerDelay->GetMax());
        trigger_delay_to_set = pTriggerDelay->GetMax();
      }
      pTriggerDelay->SetValue(trigger_delay_to_set);
      reached_trigger_delay = pTriggerDelay->GetValue();
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Camera does not support TriggerDelay. Will keep the current settings");
      reached_trigger_delay = pTriggerDelay->GetValue();
    }
  }
  catch (const GenICam::GenericException& e)
  {
    RCLCPP_ERROR(this->get_logger(), "An exception while setting target TriggerDelay to %f occurred: %s",
                 target_trigger_delay, e.GetDescription());
    return false;
  }
  return true;
}

bool ArenaCameraNode::setTriggerDelay(const float& target_trigger_delay, float& reached_trigger_delay)
{
  boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);

  if (!setTriggerDelayValue(target_trigger_delay, reached_trigger_delay))
  {
    // retry till timeout
    rclcpp::Rate r(10.0);
    auto timeout = this->now() + rclcpp::Duration::from_seconds(2.0);
    while (rclcpp::ok())
    {
      if (setTriggerDelayValue(target_trigger_delay, reached_trigger_delay))
      {
        break;
      }
      if (this->now() > timeout)
      {
        RCLCPP_ERROR(this->get_logger(), "Error in setTriggerDelay(): Unable to set target TriggerDelay before timeout");
        return false;
      }
      r.sleep();
    }
  }
  return true;
}

// End added