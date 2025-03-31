/* SPDX-License-Identifier: Apache-2.0 */

/* Copyright 2025 Pepperl+Fuchs SE
 *
 * Authors:
 *   Adrian Krzizok <git@breeze-innovations.com>
 *   Jan Hegner <opensource-jhe@de.pepperl-fuchs.com>
 *   Markus Moll <opensource-mmo@de.pepperl-fuchs.com>
 */

#include "SmartRunnerNodeBase.hpp"

#include <cassert>
#include <limits>
#include <memory>
#include <string>
#include <utility>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "Utilities.hpp"

namespace pepperl_fuchs
{
SmartRunnerNodeBase::SmartRunnerNodeBase(const char *name)
: rclcpp::Node(name)
{
  using namespace parameters;
  declparam(this, "frame_id", frame_id_, ReadOnly{});
  declparam(this, "device_ip", device_ip_, ReadOnly{});

  // Reading and checking parameters from launch-file
  get_parameter("frame_id", frame_id_);
  get_parameter("device_ip", device_ip_);

  RCLCPP_INFO(get_logger(), "ip: %s", device_ip_.c_str());
  RCLCPP_INFO(get_logger(), "id: %s", frame_id_.c_str());

  if(device_ip_ == "") {
    throw std::invalid_argument("No IP address given");
  }
}

SmartRunnerNodeBase::~SmartRunnerNodeBase()
{
  assert(!grab_thread_.joinable());  // derived class must have called stop_grabber()!
  disconnect_vsx(vsx_);
}

void SmartRunnerNodeBase::stop_grabber()
{
  // Stop grab thread if it was started
  running_ = false;
  if (grab_thread_.joinable()) {
    grab_thread_.join();
  }
}

void SmartRunnerNodeBase::publish(std::unique_ptr<sensor_msgs::msg::PointCloud2> msg)
{
  if (msg) {
    msg->header.frame_id = frame_id_;
    msg->header.stamp = get_clock()->now();
    scan_publisher_->publish(std::move(msg));
  } else {
    RCLCPP_DEBUG(get_logger(), "publish() called with null PointCloud2 message");
  }
}

void SmartRunnerNodeBase::init(std::string device_type)
{
  auto vsx = connect_vsx(std::move(device_type));
  RCLCPP_INFO(get_logger(), "VSX connection established");

  setup_vsx(vsx);
  RCLCPP_INFO(get_logger(), "VSX setup complete");

  start_grabber(std::move(vsx));
  RCLCPP_INFO(get_logger(), "VSX frame grabbing started");
}

VsxSystemHandleManaged SmartRunnerNodeBase::connect_vsx(std::string device_type) const
{
  const char * version = nullptr;
  const char * plugin = "";

  auto fail = [&](const char *msg)
    {
      RCLCPP_FATAL(get_logger(), msg);
      throw std::runtime_error("Could not connect to device");
    };

  VsxStatusCode status = vsx_GetLibraryVersion(&version);
  if (status != VSX_STATUS_SUCCESS) {
    fail("Failed to retrieve VSX library version");
  } else {
    RCLCPP_INFO(get_logger(), "VSX library version: %s", version);
    status = vsx_ReleaseString(&version);
  }

  VsxSystemHandleManaged vsx;
  status = vsx_InitTcpSensor(vsx.GetPointerForInit(), device_ip_.c_str(), plugin);
  if (status != VSX_STATUS_SUCCESS) {
    fail("Failed to initialize VSX");
  }

  status = vsx_Connect(vsx);
  if (status != VSX_STATUS_SUCCESS) {
    fail("Failed to connect to device");
  }

  VsxDeviceManaged deviceData;
  status = vsx_GetDeviceInformation(vsx, deviceData.GetPointerForInit());

  if (status != VSX_STATUS_SUCCESS) {
    fail("Failed to identify device");
  }

  if (deviceData->sensorType != device_type) {
    fail("Unsupported device");
  }

  RCLCPP_INFO(get_logger(), "Sensor: %s", deviceData->sensorType);
  RCLCPP_INFO(get_logger(), "Firmware: %s", deviceData->firmwareVersion);
  RCLCPP_INFO(get_logger(), "MAC: %s", deviceData->macAddress);
  RCLCPP_INFO(get_logger(), "IP: %s", deviceData->ipAddress);

  return vsx;
}

void SmartRunnerNodeBase::setup_vsx(VsxSystemHandle *vsx)
{
  return;
}

void SmartRunnerNodeBase::start_grabber(VsxSystemHandleManaged vsx)
{
  VsxStatusCode status = vsx_ResetDynamicContainerGrabber(vsx, 2, VSX_STRATEGY_DROP_OLDEST);
  if (status != VSX_STATUS_SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Failed to set up frame grabber");
    throw std::runtime_error("Failed to start frame grabber");
  }

  // Declare publisher
  scan_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("scan", 5);

  vsx_ = std::move(vsx);

  // set up grabbing loop
  grab_thread_ = std::thread(&SmartRunnerNodeBase::grab_loop, this);
}

void SmartRunnerNodeBase::disconnect_vsx(VsxSystemHandle *vsx)
{
  auto status = vsx_Disconnect(vsx);
  if (status != VSX_STATUS_SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Failed to disconnect from sensor.");
  }
}

void SmartRunnerNodeBase::grab_loop()
{
  while (running_) {
    VsxDataContainerManaged dc;
    VsxStatusCode status = vsx_GetDataContainer(vsx_, dc.GetPointerForInit(), 500);
    if (status == VSX_STATUS_SUCCESS) {
      on_data_received(dc);
    } else if (status != VSX_STATUS_ERROR_DRIVER_TIMEOUT) {
      RCLCPP_FATAL(get_logger(), "Failed to capture image, terminating");
      break;
    }
  }
}
}  // namespace pepperl_fuchs
