/************************************************************************************************
  Copyright(C)2023 Hesai Technology Co., Ltd.
  All code in this repository is released under the terms of the following [Modified BSD License.]
  Modified BSD License:
  Redistribution and use in source and binary forms,with or without modification,are permitted
  provided that the following conditions are met:
  *Redistributions of source code must retain the above copyright notice,this list of conditions
   and the following disclaimer.
  *Redistributions in binary form must reproduce the above copyright notice,this list of conditions and
   the following disclaimer in the documentation and/or other materials provided with the distribution.
  *Neither the names of the University of Texas at Austin,nor Austin Robot Technology,nor the names of
   other contributors maybe used to endorse or promote products derived from this software without
   specific prior written permission.
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGH THOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
  WARRANTIES,INCLUDING,BUT NOT LIMITED TO,THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
  PARTICULAR PURPOSE ARE DISCLAIMED.IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
  ANY DIRECT,INDIRECT,INCIDENTAL,SPECIAL,EXEMPLARY,OR CONSEQUENTIAL DAMAGES(INCLUDING,BUT NOT LIMITED TO,
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;LOSS OF USE,DATA,OR PROFITS;OR BUSINESS INTERRUPTION)HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY,WHETHER IN CONTRACT,STRICT LIABILITY,OR TORT(INCLUDING NEGLIGENCE
  OR OTHERWISE)ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,EVEN IF ADVISED OF THE POSSIBILITY OF
  SUCHDAMAGE.
************************************************************************************************/

#pragma once

#include <chrono>
#include <fstream>
#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <hesai_lidar_sdk.hpp>

#include "hesai_ros_driver/msg/udp_frame.hpp"
#include "hesai_ros_driver/msg/udp_packet.hpp"
#include "params.hpp"


namespace hesai_ros
{

class SourceDriver
{
public:
  SourceDriver();

  ~SourceDriver();

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
  {
    return node_->get_node_base_interface();
  }

private:
  std::string get_full_path(std::string filename);

  DriverParam set_params();

  void setup_set_state_service();

  void setup_pointcloud_publisher();
  
  void setup_packet_publisher();

  void setup_status_publisher();

  void packet_callback(const hesai_ros_driver::msg::UdpFrame::SharedPtr msg);
  
  void publish_pointcloud(const LidarDecodedFrame<LidarPointXYZIRT> & frame);

  void publish_packet(const UdpFrame_t & ros_msg, double timestamp);

  void set_state_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response
  );

  std::shared_ptr<rclcpp::Node> node_;
  
  std::shared_ptr<HesaiLidarSdk<LidarPointXYZIRT>> lidar_driver_;

  diagnostic_updater::Updater diagnostic_updater_;
  std::shared_ptr<diagnostic_updater::FunctionDiagnosticTask> status_task_;

  std::shared_ptr<hesai_ros::ParamListener> param_listener_;
  hesai_ros::Params params_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_state_service_;

  rclcpp::Subscription<hesai_ros_driver::msg::UdpFrame>::SharedPtr pkt_sub_;
  rclcpp::Publisher<hesai_ros_driver::msg::UdpFrame>::SharedPtr pkt_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

  std::shared_ptr<diagnostic_updater::DiagnosedPublisher<hesai_ros_driver::msg::UdpFrame>> diagnosed_pkt_pub_;
  std::shared_ptr<diagnostic_updater::DiagnosedPublisher<sensor_msgs::msg::PointCloud2>> diagnosed_cloud_pub_;

  sensor_msgs::msg::PointCloud2 pcl_ros_msg_;
  hesai_ros_driver::msg::UdpFrame udp_frame_ros_msg_;
};

}  // namespace hesai_ros