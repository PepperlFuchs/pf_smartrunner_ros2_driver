/* SPDX-License-Identifier: Apache-2.0 */

/* Copyright 2025 Pepperl+Fuchs SE
 *
 * Authors:
 *   Adrian Krzizok <git@breeze-innovations.com>
 *   Jan Hegner <opensource-jhe@de.pepperl-fuchs.com>
 *   Markus Moll <opensource-mmo@de.pepperl-fuchs.com>
 */

#ifndef ROSNODE__SMARTRUNNERNODEBASE_HPP_
#define ROSNODE__SMARTRUNNERNODEBASE_HPP_

#include <atomic>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "VsxResourceManaged.hpp"

namespace pepperl_fuchs
{
class SmartRunnerNodeBase : public rclcpp::Node
{
public:
  virtual ~SmartRunnerNodeBase();

protected:
  SmartRunnerNodeBase(const char *name);

  void publish(std::unique_ptr<sensor_msgs::msg::PointCloud2> msg);
  void init(std::string device_id);
  void stop_grabber();

  VsxSystemHandle * get_vsx() const {return vsx_;}

private:
  VsxSystemHandleManaged connect_vsx(std::string device_type) const;
  virtual void setup_vsx(VsxSystemHandle *vsx);
  void start_grabber(VsxSystemHandleManaged vsx);
  void disconnect_vsx(VsxSystemHandle *vsx);

  //! Grab loop that will run in grab thread
  void grab_loop();

  //! Implemented by derived classes
  virtual void on_data_received(VsxDataContainerHandle *dc) = 0;

  // Common Parameters
  std::string frame_id_ = "";
  std::string device_ip_ = "";

  // Vsx-Interface
  VsxSystemHandleManaged vsx_;

  //! ROS publisher for publishing scan data
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_publisher_;

  // grab thread
  std::thread grab_thread_;
  std::atomic_bool running_ = true;
};
}  // namespace pepperl_fuchs

#endif  // ROSNODE__SMARTRUNNERNODEBASE_HPP_
