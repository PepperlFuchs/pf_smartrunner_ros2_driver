/* SPDX-License-Identifier: Apache-2.0 */

/* Copyright 2025 Pepperl+Fuchs SE
 *
 * Authors:
 *   Adrian Krzizok <git@breeze-innovations.com>
 *   Jan Hegner <opensource-jhe@de.pepperl-fuchs.com>
 *   Markus Moll <opensource-mmo@de.pepperl-fuchs.com>
 */

#ifndef ROSNODE__SMARTRUNNER3DSTEREONODE_HPP_
#define ROSNODE__SMARTRUNNER3DSTEREONODE_HPP_

#include "SmartRunnerNodeBase.hpp"

namespace pepperl_fuchs
{
class SmartRunner3dStereoNode : public SmartRunnerNodeBase
{
public:
  SmartRunner3dStereoNode();
  ~SmartRunner3dStereoNode();

private:
  void setup_vsx(VsxSystemHandle *vsx) override;
  void on_data_received(VsxDataContainerHandle *dc) override;

  std::string trigger_source_ = "";
  double auto_trigger_rate_ = 0.0;
  bool trigger_enable_ = false;
  int exposure_time_ = 0;
  int gain_ = 0;
  int uniqueness_ = 0;
  bool median_ = false;
  std::string output_mode_ = "";

  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
  rclcpp::ParameterCallbackHandle::SharedPtr exposure_time_cb_;
  rclcpp::ParameterCallbackHandle::SharedPtr gain_cb_;
  rclcpp::ParameterCallbackHandle::SharedPtr uniqueness_cb_;
  rclcpp::ParameterCallbackHandle::SharedPtr median_cb_;
  rclcpp::ParameterCallbackHandle::SharedPtr auto_trigger_rate_cb_;
};
}  // namespace pepperl_fuchs

#endif  // ROSNODE__SMARTRUNNER3DSTEREONODE_HPP_
