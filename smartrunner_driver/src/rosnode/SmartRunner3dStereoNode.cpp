/* SPDX-License-Identifier: Apache-2.0 */

/* Copyright 2025 Pepperl+Fuchs SE
 *
 * Authors:
 *   Adrian Krzizok <git@breeze-innovations.com>
 *   Jan Hegner <opensource-jhe@de.pepperl-fuchs.com>
 *   Markus Moll <opensource-mmo@de.pepperl-fuchs.com>
 */

#include "SmartRunner3dStereoNode.hpp"

#include <limits>
#include <memory>
#include <string>
#include <utility>

#include "Utilities.hpp"

namespace pepperl_fuchs
{
SmartRunner3dStereoNode::SmartRunner3dStereoNode()
: SmartRunnerNodeBase("smartrunner_3d_stereo_node")
{
  using namespace parameters;
  declparam(this, "trigger_source", trigger_source_);
  declparam(this, "output_mode", output_mode_);
  declparam(this, "auto_trigger_rate", auto_trigger_rate_,
      Description{"The frequency with which the sensor is triggered in Hz"},
      FRange{1.0, 10.0, 0.1});
  declparam(this, "trigger_enable", trigger_enable_);
  declparam(this, "exposure_time", exposure_time_,
      Description{"The exposure time in micro-seconds"}, IRange{10, 10'000, 1});
  declparam(this, "gain", gain_, Description{"Imager gain in 1/10th dB"}, IRange{0, 480, 1});
  declparam(this, "uniqueness", uniqueness_,
      Description{
      "Confidence threshold for 3d estimation. Higher values lead to more discarded points."},
      IRange{0, 100, 1});
  declparam(this, "median", median_, Description{"Activates noise filtering on 3d points"});

  get_parameter("trigger_source", trigger_source_);
  get_parameter("output_mode", output_mode_);
  get_parameter("auto_trigger_rate", auto_trigger_rate_);
  get_parameter("trigger_enable", trigger_enable_);
  get_parameter("exposure_time", exposure_time_);
  get_parameter("gain", gain_);
  get_parameter("uniqueness", uniqueness_);
  get_parameter("median", median_);

  RCLCPP_INFO(get_logger(), "trigger_source: %s", trigger_source_.c_str());
  RCLCPP_INFO(get_logger(), "auto_trigger_rate: %f", auto_trigger_rate_);
  RCLCPP_INFO(get_logger(), "trigger_enable: %d", trigger_enable_);
  RCLCPP_INFO(get_logger(), "exposure_time: %d", exposure_time_);
  RCLCPP_INFO(get_logger(), "gain: %d", gain_);
  RCLCPP_INFO(get_logger(), "uniqueness: %d", uniqueness_);
  RCLCPP_INFO(get_logger(), "output_mode: %s", output_mode_.c_str());
  RCLCPP_INFO(get_logger(), "median: %d", median_);

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

  gain_cb_ = param_subscriber_->add_parameter_callback("gain",
      [this](const rclcpp::Parameter & p) {
        const int value = p.as_int();
        VsxStatusCode status = vsx_SetSingleParameterValueInt32(get_vsx(), 1, "Base", 1, "Gain",
        value);
        if (status != VSX_STATUS_SUCCESS) {
          RCLCPP_ERROR(get_logger(), "Could not set gain to %d", value);
        } else {gain_ = value;}
  });

  uniqueness_cb_ = param_subscriber_->add_parameter_callback("uniqueness",
      [this](const rclcpp::Parameter & p) {
        const int value = p.as_int();
        VsxStatusCode status = vsx_SetSingleParameterValueInt32(get_vsx(), 1, "Base", 1,
        "SGBMUniqueness", value);
        if (status != VSX_STATUS_SUCCESS) {
          RCLCPP_ERROR(get_logger(), "Could not set uniqueness threshold to %d", value);
        } else {uniqueness_ = value;}
  });

  median_cb_ = param_subscriber_->add_parameter_callback("median",
      [this](const rclcpp::Parameter & p) {
        const bool value = p.as_bool();
        VsxStatusCode status = vsx_SetSingleParameterValueInt32(get_vsx(), 1, "Base", 1,
        "SGBMEnableMedian", value);
        if (status != VSX_STATUS_SUCCESS) {
          RCLCPP_ERROR(get_logger(), "Could not set median filter flag to %s",
          value ? "true" : "false");
        } else {median_ = value;}
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

  init("SR3D_STEREO");
}

SmartRunner3dStereoNode::~SmartRunner3dStereoNode()
{
  stop_grabber();
}

void SmartRunner3dStereoNode::setup_vsx(VsxSystemHandle *vsx)
{
  auto test = [&](VsxStatusCode s) {
      if (s != VSX_STATUS_SUCCESS) {
        RCLCPP_FATAL(get_logger(), "Failed to configure device");
        throw std::runtime_error("Failed to configure device");
      }
    };

  test(vsx_SetSingleParameterValue(vsx, 1, "Base", 1, "TriggerSource", trigger_source_.c_str()));
  test(vsx_SetSingleParameterValueDouble(vsx, 1, "Base", 1, "AutoTriggerFrameRate",
      auto_trigger_rate_));
  test(vsx_SetSingleParameterValue(vsx, 1, "Base", 1, "TriggerEnabled", "0"));
  test(vsx_SetSingleParameterValueInt32(vsx, 1, "Base", 1, "TriggerEnabled", trigger_enable_));
  test(vsx_SetSingleParameterValueInt32(vsx, 1, "Base", 1, "ExposureTime", exposure_time_));
  test(vsx_SetSingleParameterValueInt32(vsx, 1, "Base", 1, "Gain", gain_));
  test(vsx_SetSingleParameterValueInt32(vsx, 1, "Base", 1, "SGBMUniqueness", uniqueness_));
  test(vsx_SetSingleParameterValueInt32(vsx, 1, "Base", 1, "SGBMEnableMedian", median_));
  test(vsx_SetSingleParameterValue(vsx, 1, "Base", 1, "OutputMode", output_mode_.c_str()));
}

void SmartRunner3dStereoNode::on_data_received(VsxDataContainerHandle *dc)
{
  publish(makePointCloud2(dc, "SR3D_STEREO"));
}
}  // namespace pepperl_fuchs

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pepperl_fuchs::SmartRunner3dStereoNode>());
  rclcpp::shutdown();
}
