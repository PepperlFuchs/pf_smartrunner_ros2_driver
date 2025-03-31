/* SPDX-License-Identifier: Apache-2.0 */

/* Copyright 2025 Pepperl+Fuchs SE
 *
 * Authors:
 *   Adrian Krzizok <git@breeze-innovations.com>
 *   Jan Hegner <opensource-jhe@de.pepperl-fuchs.com>
 *   Markus Moll <opensource-mmo@de.pepperl-fuchs.com>
 */

#ifndef ROSNODE__SMARTRUNNER2DNODE_HPP_
#define ROSNODE__SMARTRUNNER2DNODE_HPP_

#include "SmartRunnerNodeBase.hpp"

namespace pepperl_fuchs
{
class SmartRunner2dNode : public SmartRunnerNodeBase
{
public:
  SmartRunner2dNode();
  ~SmartRunner2dNode();

private:
  void setup_vsx(VsxSystemHandle *vsx) override;
  void on_data_received(VsxDataContainerHandle *dc) override;

  int autotrigger_enable_ = 0;
  int exposure_time_ = 0;
  int use_manual_exposure_time_ = 0;
  int flash_time_ = 0;
  int object_contrast_ = 0;
  int roi_min_x_ = 0;
  int roi_max_x_ = 0;
  int roi_min_z_ = 0;
  int roi_max_z_ = 0;
  int image_transfer_active_ = 0;
};
}  // namespace pepperl_fuchs

#endif  // ROSNODE__SMARTRUNNER2DNODE_HPP_
