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

#include <rclcpp/rclcpp.hpp>

#include "source_driver.h"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  sched_param sch;
  sch.sched_priority = 50;
  if (sched_setscheduler(0, SCHED_FIFO, &sch) == -1) {
    std::string errmsg(
      "Cannot set as real-time thread. Users must set:\n"
      "* hard rtprio 99\n"
      "* soft rtprio 99\n"
      "in /etc/security/limits.conf to enable realtime prioritization! \nError: ");
    throw std::runtime_error(errmsg + std::strerror(errno));
  }

  auto driver = std::make_shared<hesai_ros::SourceDriver>();
  
  rclcpp::spin(driver->get_node_base_interface());

  return 0;
}
