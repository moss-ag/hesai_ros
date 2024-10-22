#include "source_driver.h"

#include <regex>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace hesai_ros
{

SourceDriver::SourceDriver(): 
node_(std::make_shared<rclcpp::Node>("hesai_node")),
lidar_driver_(std::make_shared<HesaiLidarSdk<LidarPointXYZIRT>>()),
diagnostic_updater_(node_, 5.0)
{
  param_listener_ = std::make_shared<hesai_ros::ParamListener>(
  node_->get_node_parameters_interface());
  params_ = param_listener_->get_params();

  const std::string hardware_id = "Hesai LiDAR" + params_.lidar_type;
  diagnostic_updater_.setHardwareID(hardware_id);

  setup_set_state_service();
  setup_packet_publisher();
  setup_pointcloud_publisher();
  setup_status_publisher();

  DriverParam driver_param = set_params();
  if (!lidar_driver_->Init(driver_param)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to initialize driver with params");
    exit(-1);
  }

  lidar_driver_->Start();
}

SourceDriver::~SourceDriver()
{
  lidar_driver_->Stop();
}

std::string SourceDriver::get_full_path(std::string filename)
{
  std::regex package_regex(R"(package:\/\/([A-Za-z0-9]+(_[A-Za-z0-9]+)+)\/(.*))");
  std::smatch sm;

  std::string filepath = filename;
  if (std::regex_search(filepath, sm, package_regex)) {
    filepath = ament_index_cpp::get_package_share_directory(sm.str(1)) + "/" + sm.str(3);
  }
  return filepath;
}

DriverParam SourceDriver::set_params()
{
  DriverParam driver_param;

  driver_param.frame_id = params_.frame_id;
  driver_param.lidar_type = params_.lidar_type;
  driver_param.log_path = "./hesai_lidar.log";

  driver_param.input_param.udp_port = params_.udp_port;
  driver_param.input_param.ptc_port = params_.ptc_port;
  driver_param.input_param.host_ip_address = params_.host_ip_address;
  driver_param.input_param.device_ip_address = params_.device_ip_address;
  driver_param.input_param.multicast_ip_address = "";
  driver_param.input_param.pcap_path = get_full_path(params_.pcap_path);
  driver_param.input_param.correction_file_path = get_full_path(params_.correction_path);
  driver_param.input_param.firetimes_path = get_full_path(params_.firetimes_path);
  driver_param.input_param.source_type = SourceType(params_.source_type);
  driver_param.input_param.standby_mode = params_.standby_mode;
  driver_param.input_param.speed = params_.speed;

  driver_param.decoder_param.enable_parser_thread = true;
  driver_param.decoder_param.pcap_play_synchronization = params_.pcap_play_synchronization;
  driver_param.decoder_param.frame_start_azimuth = params_.frame_start_azimuth;
  driver_param.decoder_param.enable_packet_loss_tool = params_.enable_packet_loss_tool;
  driver_param.decoder_param.fov_start = params_.fov_start;
  driver_param.decoder_param.fov_end = params_.fov_end;
  driver_param.decoder_param.distance_correction_lidar_type = params_.distance_correction_lidar_type;

  driver_param.decoder_param.transform_param.x = params_.transform.x;
  driver_param.decoder_param.transform_param.y = params_.transform.y;
  driver_param.decoder_param.transform_param.z = params_.transform.z;
  driver_param.decoder_param.transform_param.roll = params_.transform.roll;
  driver_param.decoder_param.transform_param.pitch = params_.transform.pitch;
  driver_param.decoder_param.transform_param.yaw = params_.transform.yaw;

  if (params_.source_type == DATA_FROM_ROS_PACKET)
  {
    driver_param.decoder_param.enable_udp_thread = false;
  }

  return driver_param;
}

void SourceDriver::setup_set_state_service() {
  set_state_service_ = node_->create_service<std_srvs::srv::SetBool>(
    "~/activate",
    std::bind(&SourceDriver::set_state_callback, this, std::placeholders::_1, std::placeholders::_2)
  );
}

void SourceDriver::setup_pointcloud_publisher() {
  if (!params_.publish_pointcloud)
  {
    return;
  }

  cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(params_.topics.pointcloud, 100);

  diagnosed_cloud_pub_ = std::make_shared<diagnostic_updater::DiagnosedPublisher<sensor_msgs::msg::PointCloud2>>(
    cloud_pub_,
    diagnostic_updater_,
    diagnostic_updater::FrequencyStatusParam(&params_.diagnostics.cloud_min_freq, &params_.diagnostics.cloud_max_freq, 0, 10),
    diagnostic_updater::TimeStampStatusParam(-1, 1));

  // Reserve 50K points
  pcl_ros_msg_.data.reserve(sizeof(LidarPointXYZIRT) * 5e4);

  // Add fields
  pcl_ros_msg_.fields.reserve(6);
  pcl_ros_msg_.fields.clear();
  int offset = 0;
  offset = addPointField(pcl_ros_msg_, "x", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
  offset = addPointField(pcl_ros_msg_, "y", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
  offset = addPointField(pcl_ros_msg_, "z", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
  offset = addPointField(
    pcl_ros_msg_, "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
    offset);
  offset = addPointField(pcl_ros_msg_, "ring", 1, sensor_msgs::msg::PointField::UINT16, offset);
  offset = addPointField(
    pcl_ros_msg_, "timestamp", 1, sensor_msgs::msg::PointField::FLOAT64,
    offset);

  // Set header and assign other static values
  pcl_ros_msg_.header.frame_id = params_.frame_id;
  pcl_ros_msg_.height = 1;
  pcl_ros_msg_.point_step = offset;
  pcl_ros_msg_.is_dense = false;

  lidar_driver_->RegRecvCallback(
    std::bind(
      &SourceDriver::publish_pointcloud, this,
      std::placeholders::_1));
}

void SourceDriver::setup_packet_publisher()
{
    if (params_.source_type == DATA_FROM_ROS_PACKET) {
    params_.publish_packet = false;
    RCLCPP_WARN(node_->get_logger(), "Cannot publish packet when source is ROS packet");
  
    pkt_sub_ = node_->create_subscription<hesai_ros_driver::msg::UdpFrame>(
      params_.topics.packet, 10,
      std::bind(&SourceDriver::packet_callback, this, std::placeholders::_1));
  }

  else if (params_.publish_packet) {
    pkt_pub_ = node_->create_publisher<hesai_ros_driver::msg::UdpFrame>(
      params_.topics.packet,
      10);

    diagnosed_pkt_pub_ = std::make_shared<diagnostic_updater::DiagnosedPublisher<hesai_ros_driver::msg::UdpFrame>>(
      pkt_pub_,
      diagnostic_updater_,
      diagnostic_updater::FrequencyStatusParam(&params_.diagnostics.pkt_min_freq, &params_.diagnostics.pkt_max_freq, 0, 10),
      diagnostic_updater::TimeStampStatusParam(-1, 1));

    udp_frame_ros_msg_.packets.reserve(1000);
    udp_frame_ros_msg_.header.frame_id = params_.frame_id;

    lidar_driver_->RegRecvCallback(
      std::bind(
        &SourceDriver::publish_packet, this, std::placeholders::_1,
        std::placeholders::_2));
  }
}

void SourceDriver::publish_pointcloud(const LidarDecodedFrame<LidarPointXYZIRT> & frame)
{
  pcl_ros_msg_.width = frame.points_num;
  pcl_ros_msg_.row_step = pcl_ros_msg_.width * pcl_ros_msg_.point_step;

  pcl_ros_msg_.data.clear();
  pcl_ros_msg_.data.resize(frame.points_num * pcl_ros_msg_.point_step);

  sensor_msgs::PointCloud2Iterator<float> iter_x_(pcl_ros_msg_, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y_(pcl_ros_msg_, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z_(pcl_ros_msg_, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_intensity_(pcl_ros_msg_, "intensity");
  sensor_msgs::PointCloud2Iterator<uint16_t> iter_ring_(pcl_ros_msg_, "ring");
  sensor_msgs::PointCloud2Iterator<double> iter_timestamp_(pcl_ros_msg_, "timestamp");

  for (size_t i = 0; i < frame.points_num; i++) {
    LidarPointXYZIRT point = frame.points[i];
    *iter_x_ = point.x;
    *iter_y_ = point.y;
    *iter_z_ = point.z;
    *iter_intensity_ = point.intensity;
    *iter_ring_ = point.ring;
    *iter_timestamp_ = point.timestamp;
    ++iter_x_;
    ++iter_y_;
    ++iter_z_;
    ++iter_intensity_;
    ++iter_ring_;
    ++iter_timestamp_;
  }

  pcl_ros_msg_.header.stamp.sec = (uint32_t) floor(frame.points[0].timestamp);
  pcl_ros_msg_.header.stamp.nanosec =
    (uint32_t) round((frame.points[0].timestamp - pcl_ros_msg_.header.stamp.sec) * 1e9);

  diagnosed_cloud_pub_->publish(pcl_ros_msg_);
}

void SourceDriver::publish_packet(const UdpFrame_t & msg, double timestamp)
{
  udp_frame_ros_msg_.packets.clear();

  for (size_t i = 0; i < msg.size(); i++) {
    hesai_ros_driver::msg::UdpPacket rawpacket;
    rawpacket.size = msg[i].packet_len;
    rawpacket.data.resize(msg[i].packet_len);
    memcpy(&rawpacket.data[0], &msg[i].buffer[0], msg[i].packet_len);
    udp_frame_ros_msg_.packets.push_back(rawpacket);
  }

  udp_frame_ros_msg_.header.stamp.sec = (uint32_t)floor(timestamp);
  udp_frame_ros_msg_.header.stamp.nanosec = (uint32_t)round(
    (timestamp - udp_frame_ros_msg_.header.stamp.sec) * 1e9);

  diagnosed_pkt_pub_->publish(udp_frame_ros_msg_);
}

void SourceDriver::packet_callback(const hesai_ros_driver::msg::UdpFrame::SharedPtr msg)
{
  for (size_t i = 0; i < msg->packets.size(); i++) {
    lidar_driver_->lidar_ptr_->origin_packets_buffer_.emplace_back(
      &msg->packets[i].data[0],
      msg->packets[i].size);
  }
}

void SourceDriver::set_state_callback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response
)
{
  if (request->data) {
    lidar_driver_->lidar_ptr_->ptc_client_->SetStandbyMode(0);

    RCLCPP_INFO(node_->get_logger(), "Lidar driver activated");
  } else {
    lidar_driver_->lidar_ptr_->ptc_client_->SetStandbyMode(1);

    RCLCPP_INFO(node_->get_logger(), "Lidar driver deactivated");
  }

  response->success = true;
}


void SourceDriver::setup_status_publisher()
{
  if (params_.publish_status) {
    status_task_ = std::make_shared<diagnostic_updater::FunctionDiagnosticTask>(
      "lidar status", [&] (diagnostic_updater::DiagnosticStatusWrapper & stat)
      {
        if (lidar_driver_->lidar_ptr_ == nullptr || lidar_driver_->lidar_ptr_->ptc_client_ == nullptr) {
          stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "LiDAR is not running");
          return;
        }

        hesai::lidar::LidarStatus lidar_status;
        int ret = lidar_driver_->lidar_ptr_->ptc_client_->GetLidarStatus(lidar_status);
        if (ret == 0) {
          stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "LiDAR is running");
          stat.add("System uptime (s)", lidar_status.system_uptime);
          stat.add("Motor speed (RPM)", lidar_status.motor_speed);
          stat.add("Bottom circuit board T1 (°C)", lidar_status.temperature[0] / 100.0);
          stat.add("Bottom circuit board T2 (°C)", lidar_status.temperature[1] / 100.0);
          stat.add("Laser emitting board RT_L1 (°C)", lidar_status.temperature[2] / 100.0);
          stat.add("Laser emitting board RT_L2 (°C)", lidar_status.temperature[3] / 100.0);
          stat.add("Laser Receiving board RT_R (°C)", lidar_status.temperature[4] / 100.0);
          stat.add("Laser Receiving board RT2 (°C)", lidar_status.temperature[5] / 100.0);
          stat.add("Top circuit RT3 (°C)", lidar_status.temperature[6] / 100.0);
          stat.add("Top circuit RT4 (°C)", lidar_status.temperature[7] / 100.0);
          stat.add("GPS PPS status", lidar_status.gps_pps_lock ? "Locked" : "Unlocked");
          stat.add("GPS NMEA status", lidar_status.gps_gprmc_status ? "Locked" : "Unlocked");
          stat.add("System start-up times", lidar_status.startup_times);
          stat.add("Total time in operation", lidar_status.total_operation_time);

          static const std::unordered_map<uint8_t, std::string> ptp_status_map = {
            {0, "Free run"},
            {1, "Tracking"},
            {2, "Locked"},
            {3, "Frozen"},
          };
          std::string ptp_status_str = "Unknown";
          if (ptp_status_map.find(lidar_status.ptp_status) != ptp_status_map.end()) {
            ptp_status_str = ptp_status_map.at(lidar_status.ptp_status);
          }
          stat.add("PTP status", ptp_status_str);
  
          stat.add("PTP offset (ns)", lidar_status.ptp_offset);
          
          static const std::unordered_map<uint8_t, std::string> ptp_state_map = {
            {0, "None"},
            {1, "Intializing"},
            {2, "Faulty"},
            {3, "Disabled"},
            {4, "Listening"},
            {5, "Premaster"},
            {6, "Master"},
            {7, "Passive"},
            {8, "Uncalibrated"},
            {9, "Slave"},
            {10, "Grandmaster"},
          };
          std::string ptp_state_str = "Unknown";
          if (ptp_state_map.find(lidar_status.ptp_state) != ptp_state_map.end()) {
            ptp_state_str = ptp_state_map.at(lidar_status.ptp_state);
          }
          stat.add("PTP state", ptp_state_str);

          stat.add("PTP handshake elapsed time (ms)", lidar_status.ptp_handshake_elapsed_time);

        } else {
          stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Failed to get lidar status");
        }

      });
    diagnostic_updater_.add(*status_task_);
  }
}

} // namespace hesai_ros
