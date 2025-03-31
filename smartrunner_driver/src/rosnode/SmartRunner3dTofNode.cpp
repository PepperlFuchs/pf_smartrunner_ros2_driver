/* SPDX-License-Identifier: Apache-2.0 */

/* Copyright 2025 Pepperl+Fuchs SE
 *
 * Authors:
 *   Adrian Krzizok <git@breeze-innovations.com>
 *   Jan Hegner <opensource-jhe@de.pepperl-fuchs.com>
 *   Markus Moll <opensource-mmo@de.pepperl-fuchs.com>
 */

#include "SmartRunner3dTofNode.hpp"

#include <memory>
#include <string>
#include <utility>

#include "Utilities.hpp"

namespace pepperl_fuchs
{
SmartRunner3dTofNode::SmartRunner3dTofNode()
: SmartRunnerNodeBase("smartrunner_3d_tof_node")
{
  using namespace parameters;
  declparam(this, "trigger_source", trigger_source_);
  declparam(this, "output_mode", output_mode_);
  declparam(this, "auto_trigger_rate", auto_trigger_rate_,
      Description{"The frequency with which the sensor is triggered in Hz"},
      FRange{1.0, 30.0, 0.1});
  declparam(this, "trigger_enable", trigger_enable_);
  declparam(this, "exposure_time", exposure_time_,
      Description{"The exposure time in micro-seconds"}, IRange{150, 1000, 1});
  declparam(this, "range_mode", range_mode_, Description{"Extent of the working range in mm"},
      AdditionalConstraints{"Must be one of 1500, 3750, or 7500"});

  get_parameter("trigger_source", trigger_source_);
  get_parameter("output_mode", output_mode_);
  get_parameter("auto_trigger_rate", auto_trigger_rate_);
  get_parameter("trigger_enable", trigger_enable_);
  get_parameter("exposure_time", exposure_time_);
  get_parameter("range_mode", range_mode_);

  RCLCPP_INFO(get_logger(), "trigger_source: %s", trigger_source_.c_str());
  RCLCPP_INFO(get_logger(), "auto_trigger_rate: %f", auto_trigger_rate_);
  RCLCPP_INFO(get_logger(), "trigger_enable: %d", trigger_enable_);
  RCLCPP_INFO(get_logger(), "exposure_time: %d", exposure_time_);
  RCLCPP_INFO(get_logger(), "range_mode: %d", range_mode_);
  RCLCPP_INFO(get_logger(), "output_mode: %s", output_mode_.c_str());

  param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

  exposure_time_cb_ = param_subscriber_->add_parameter_callback("exposure_time",
      [this](const rclcpp::Parameter & p) {
        const int value = p.as_int();
        VsxStatusCode status = vsx_SetSingleParameterValueInt32(get_vsx(), 1, "Base", 1,
        "ExposureTime",
        value);
        if (status != VSX_STATUS_SUCCESS) {
          RCLCPP_ERROR(get_logger(), "Could not set exposure time to %d", value);
        } else {exposure_time_ = value;}
  });

  range_mode_cb_ = param_subscriber_->add_parameter_callback("range_mode",
      [this](const rclcpp::Parameter & p) {
        const int value = p.as_int();
        if (value != 1500 && value != 3750 && value != 7500)
        {
          RCLCPP_ERROR(get_logger(), "Invalid range mode value: %d", value);
          return;
        }
        VsxStatusCode status = vsx_SetSingleParameterValueInt32(get_vsx(), 1, "Base", 1, "RangeMode",
        value);
        if (status != VSX_STATUS_SUCCESS) {
          RCLCPP_ERROR(get_logger(), "Could not set range mode to %d", value);
        } else {range_mode_ = value;}
  });

  auto_trigger_rate_cb_ =
    param_subscriber_->add_parameter_callback("auto_trigger_rate",
      [this](const rclcpp::Parameter & p) {
        const double value = p.as_double();
        VsxStatusCode status = vsx_SetSingleParameterValueDouble(get_vsx(), 1, "Base", 1,
        "AutoTriggerFrameRate", value);
        if (status != VSX_STATUS_SUCCESS) {
          RCLCPP_ERROR(get_logger(), "Could not set auto trigger frame rate to %f", value);
        } else {auto_trigger_rate_ = value;}
  });

  init("SR3D_TOF");
}

SmartRunner3dTofNode::~SmartRunner3dTofNode()
{
  stop_grabber();
}


void SmartRunner3dTofNode::setup_vsx(VsxSystemHandle *vsx)
{
  auto test = [&](VsxStatusCode s) {
      if (s != VSX_STATUS_SUCCESS) {
        RCLCPP_FATAL(get_logger(), "Failed to configure device");
        throw std::runtime_error("Failed to configure device");
      }
    };

  test(vsx_SetSingleParameterValue(vsx, 1, "Base", 1, "TriggerSource",
        trigger_source_.c_str()));
  test(vsx_SetSingleParameterValueDouble(vsx, 1, "Base", 1, "AutoTriggerFrameRate",
        auto_trigger_rate_));
  test(vsx_SetSingleParameterValue(vsx, 1, "Base", 1, "TriggerEnabled", "0"));
  test(vsx_SetSingleParameterValueInt32(vsx, 1, "Base", 1, "TriggerEnabled",
        trigger_enable_));
  test(vsx_SetSingleParameterValueInt32(vsx, 1, "Base", 1, "RangeMode", range_mode_));
  test(vsx_SetSingleParameterValueInt32(vsx, 1, "Base", 1, "ExposureTime",
        exposure_time_));
  test(vsx_SetSingleParameterValue(vsx, 1, "Base", 1, "OutputMode",
        output_mode_.c_str()));
}

void SmartRunner3dTofNode::on_data_received(VsxDataContainerHandle *dc)
{
  publish(makePointCloud2(dc, "SR3D_TOF"));
}
}  // namespace pepperl_fuchs

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pepperl_fuchs::SmartRunner3dTofNode>());
  rclcpp::shutdown();
}
