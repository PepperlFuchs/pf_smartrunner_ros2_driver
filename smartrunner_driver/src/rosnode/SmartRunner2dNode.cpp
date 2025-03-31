/* SPDX-License-Identifier: Apache-2.0 */

/* Copyright 2025 Pepperl+Fuchs SE
 *
 * Authors:
 *   Adrian Krzizok <git@breeze-innovations.com>
 *   Jan Hegner <opensource-jhe@de.pepperl-fuchs.com>
 *   Markus Moll <opensource-mmo@de.pepperl-fuchs.com>
 */

#include "SmartRunner2dNode.hpp"

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
SmartRunner2dNode::SmartRunner2dNode()
: SmartRunnerNodeBase("smartrunner_2d_node")
{
  using namespace parameters;
  declparam(this, "autotrigger_enable", autotrigger_enable_);
  declparam(this, "exposure_time", exposure_time_);
  declparam(this, "use_manual_exposure_time", use_manual_exposure_time_);
  declparam(this, "flash_time", flash_time_);
  declparam(this, "object_contrast", object_contrast_);
  declparam(this, "roi_min_x", roi_min_x_);
  declparam(this, "roi_max_x", roi_max_x_);
  declparam(this, "roi_min_z", roi_min_z_);
  declparam(this, "roi_max_z", roi_max_z_);
  declparam(this, "image_transfer_active", image_transfer_active_);

  get_parameter("autotrigger_enable", autotrigger_enable_);
  get_parameter("exposure_time", exposure_time_);
  get_parameter("use_manual_exposure_time", use_manual_exposure_time_);
  get_parameter("flash_time", flash_time_);
  get_parameter("object_contrast", object_contrast_);
  get_parameter("roi_min_x", roi_min_x_);
  get_parameter("roi_max_x", roi_max_x_);
  get_parameter("roi_min_z", roi_min_z_);
  get_parameter("roi_max_z", roi_max_z_);
  get_parameter("image_transfer_active", image_transfer_active_);

  RCLCPP_INFO(get_logger(), "trigger_enable: %d", autotrigger_enable_);
  RCLCPP_INFO(get_logger(), "exposure_time_: %d", exposure_time_);
  RCLCPP_INFO(get_logger(), "use_manual_exposure_time_: %d", use_manual_exposure_time_);
  RCLCPP_INFO(get_logger(), "flash_time_: %d", flash_time_);
  RCLCPP_INFO(get_logger(), "object_contrast_: %d", object_contrast_);
  RCLCPP_INFO(get_logger(), "roi_min_x_: %d", roi_min_x_);
  RCLCPP_INFO(get_logger(), "roi_max_x_: %d", roi_max_x_);
  RCLCPP_INFO(get_logger(), "roi_min_z_: %d", roi_min_z_);
  RCLCPP_INFO(get_logger(), "roi_max_z_: %d", roi_max_z_);
  RCLCPP_INFO(get_logger(), "image_transfer_active_: %d", image_transfer_active_);

  init("SMARTRUNNER");
}

SmartRunner2dNode::~SmartRunner2dNode()
{
  stop_grabber();
}

void SmartRunner2dNode::setup_vsx(VsxSystemHandle *vsx)
{
  auto test = [&](VsxStatusCode s) {
      if (s != VSX_STATUS_SUCCESS) {
        RCLCPP_FATAL(get_logger(), "Failed to configure device");
        throw std::runtime_error("Failed to configure device");
      }
    };

  test(vsx_SetSingleParameterValueInt32(vsx, 1, "General", 1, "0x510001", autotrigger_enable_));
  test(vsx_SetSingleParameterValueInt32(vsx, 1, "General", 1, "0x680001", exposure_time_));
  test(vsx_SetSingleParameterValueInt32(vsx, 1, "General", 1, "0xBE0001",
      use_manual_exposure_time_));
  test(vsx_SetSingleParameterValueInt32(vsx, 1, "General", 1, "0x100001", flash_time_));
  test(vsx_SetSingleParameterValueInt32(vsx, 1, "General", 1, "0x9F0001", object_contrast_));
  test(vsx_SetSingleParameterValueInt32(vsx, 1, "General", 1, "0xC90001", roi_min_x_));
  test(vsx_SetSingleParameterValueInt32(vsx, 1, "General", 1, "0xC90002", roi_max_x_));
  test(vsx_SetSingleParameterValueInt32(vsx, 1, "General", 1, "0xC90003", roi_min_z_));
  test(vsx_SetSingleParameterValueInt32(vsx, 1, "General", 1, "0xC90004", roi_max_z_));
  test(vsx_SetSingleParameterValueInt32(vsx, 1, "General", 1, "ImageTransferActive",
      image_transfer_active_));
}

void SmartRunner2dNode::on_data_received(VsxDataContainerHandle *dc)
{
  publish(makePointCloud2(dc, "SMARTRUNNER"));
}
}  // namespace pepperl_fuchs

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pepperl_fuchs::SmartRunner2dNode>());
  rclcpp::shutdown();
}
