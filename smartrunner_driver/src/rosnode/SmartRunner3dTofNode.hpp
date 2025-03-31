/* SPDX-License-Identifier: Apache-2.0 */

/* Copyright 2025 Pepperl+Fuchs SE
 *
 * Authors:
 *   Adrian Krzizok <git@breeze-innovations.com>
 *   Jan Hegner <opensource-jhe@de.pepperl-fuchs.com>
 *   Markus Moll <opensource-mmo@de.pepperl-fuchs.com>
 */

#ifndef ROSNODE__SMARTRUNNER3DTOFNODE_HPP_
#define ROSNODE__SMARTRUNNER3DTOFNODE_HPP_

#include "SmartRunnerNodeBase.hpp"

namespace pepperl_fuchs
{
class SmartRunner3dTofNode : public SmartRunnerNodeBase
{
public:
  SmartRunner3dTofNode();
  ~SmartRunner3dTofNode();

private:
  void setup_vsx(VsxSystemHandle *vsx) override;
  void on_data_received(VsxDataContainerHandle *dc) override;

  std::string trigger_source_ = "";
  double auto_trigger_rate_ = 0.0;
  bool trigger_enable_ = false;
  int exposure_time_ = 0;
  int range_mode_ = 0;
  std::string output_mode_ = "";

  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
  rclcpp::ParameterCallbackHandle::SharedPtr exposure_time_cb_;
  rclcpp::ParameterCallbackHandle::SharedPtr range_mode_cb_;
  rclcpp::ParameterCallbackHandle::SharedPtr auto_trigger_rate_cb_;
};
}  // namespace pepperl_fuchs

#endif  // ROSNODE__SMARTRUNNER3DTOFNODE_HPP_
