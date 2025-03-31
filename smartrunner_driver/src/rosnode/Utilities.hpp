/* SPDX-License-Identifier: Apache-2.0 */

/* Copyright 2025 Pepperl+Fuchs SE
 *
 * Authors:
 *   Markus Moll <opensource-mmo@de.pepperl-fuchs.com>
 */

#ifndef ROSNODE__UTILITIES_HPP_
#define ROSNODE__UTILITIES_HPP_

#include <string>
#include <utility>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "VsxResourceManaged.hpp"

namespace pepperl_fuchs
{
namespace parameters
{
struct Description
{
  std::string description;
};

struct AdditionalConstraints
{
  std::string constraints;
};

struct ReadOnly {};

struct FRange
{
  double min;
  double max;
  double step;
};

struct IRange
{
  int min;
  int max;
  int step;
};

namespace detail
{
inline void apply(rcl_interfaces::msg::ParameterDescriptor & descriptor, Description && d)
{
  descriptor.description = std::move(d.description);
}

inline void apply(rcl_interfaces::msg::ParameterDescriptor & descriptor, const Description & d)
{
  descriptor.description = d.description;
}

inline void apply(rcl_interfaces::msg::ParameterDescriptor & descriptor, AdditionalConstraints && c)
{
  descriptor.additional_constraints = std::move(c.constraints);
}

inline void apply(
  rcl_interfaces::msg::ParameterDescriptor & descriptor,
  const AdditionalConstraints & c)
{
  descriptor.additional_constraints = c.constraints;
}

inline void apply(rcl_interfaces::msg::ParameterDescriptor & descriptor, const ReadOnly &)
{
  descriptor.read_only = true;
}

inline void apply(rcl_interfaces::msg::ParameterDescriptor & descriptor, const FRange & r)
{
  rcl_interfaces::msg::FloatingPointRange range;
  range.from_value = r.min;
  range.to_value = r.max;
  range.step = r.step;
  descriptor.floating_point_range.clear();
  descriptor.floating_point_range.push_back(range);
}

inline void apply(rcl_interfaces::msg::ParameterDescriptor & descriptor, const IRange & r)
{
  rcl_interfaces::msg::IntegerRange range;
  range.from_value = r.min;
  range.to_value = r.max;
  range.step = r.step;
  descriptor.integer_range.clear();
  descriptor.integer_range.push_back(range);
}
}  // namespace detail

template<typename ValueT, typename ... Params>
void declparam(rclcpp::Node *node, std::string name, const ValueT & value, Params &&... params)
{
  if constexpr (sizeof...(params) > 0) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    (detail::apply(descriptor, std::forward<Params>(params)), ...);
    node->declare_parameter(name, value, descriptor);
  } else {
    node->declare_parameter(name, value);
  }
}

}  // namespace parameters

std::unique_ptr<sensor_msgs::msg::PointCloud2> makePointCloud2(
  VsxDataContainerHandle * dch,
  const std::string & sensor);

}  // namespace pepperl_fuchs

#endif  // ROSNODE__UTILITIES_HPP_
