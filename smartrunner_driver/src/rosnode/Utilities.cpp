/* SPDX-License-Identifier: Apache-2.0 */

/* Copyright 2025 Pepperl+Fuchs SE
 *
 * Authors:
 *   Markus Moll <opensource-mmo@de.pepperl-fuchs.com>
 */

#include "Utilities.hpp"

#include <limits>
#include <memory>
#include <string>
#include <cstring>
#include <utility>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "VsxResourceManaged.hpp"

namespace pepperl_fuchs
{
std::unique_ptr<sensor_msgs::msg::PointCloud2> makePointCloud2(
  VsxDataContainerHandle * dch,
  const std::string & sensor)
{
  using Iter = sensor_msgs::PointCloud2Iterator<float>;

  auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();

  // msg->header will be filled by the publish() function

  auto resize = [](auto & msg, uint32_t n)
    {
      auto modifier = sensor_msgs::PointCloud2Modifier(msg);
      modifier.resize(n);
    };

  auto init = [](auto & msg, uint32_t width, uint32_t height)
    {
      auto modifier = sensor_msgs::PointCloud2Modifier(msg);
      modifier.setPointCloud2Fields(3,
                "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                "z", 1, sensor_msgs::msg::PointField::FLOAT32);
      modifier.resize(width * height);
      msg.height = height;
      msg.width = width;
      msg.is_bigendian = false;
      msg.point_step = 3 * 4;
      msg.row_step = msg.point_step * msg.width;
      msg.is_dense = false;
    };

  if(sensor == "SMARTRUNNER") {
    VsxLineDataManaged lineData;
    auto status = vsx_GetLine(dch, "Line", lineData.GetPointerForInit());
    if(status == VSX_STATUS_SUCCESS) {
      init(*msg, lineData->width, 1);

      Iter x(*msg, "x");
      Iter y(*msg, "y");
      Iter z(*msg, "z");

      for( std::size_t i = 0; i < lineData->width; i++, ++x, ++y, ++z ) {
        *x = lineData->lines[0][i].x / 1000.0f;
        *y = 0.0f;
        *z = lineData->lines[0][i].z / 1000.0f;
      }
    }
  }

  if(sensor == "SR3D_STEREO" || sensor == "SR3D_TOF") {
    VsxImageManaged imageA;
    VsxImageManaged imageB;
    VsxImageManaged imageC;

    auto status_A = vsx_GetImage(dch, "CalibratedA", imageA.GetPointerForInit());
    auto status_B = vsx_GetImage(dch, "CalibratedB", imageB.GetPointerForInit());
    auto status_C = vsx_GetImage(dch, "CalibratedC", imageC.GetPointerForInit());

    if (status_A == VSX_STATUS_SUCCESS && status_B == VSX_STATUS_SUCCESS &&
      status_C == VSX_STATUS_SUCCESS)
    {
      int width = imageA->width;
      int height = imageA->height;
      auto format = imageA->format;
      int linePitch = imageA->linePitch;

      init(*msg, width, height);

      Iter x(*msg, "x");
      Iter y(*msg, "y");
      Iter z(*msg, "z");

      const unsigned char *bytesA = static_cast<const unsigned char *>(imageA->rawdata);
      const unsigned char *bytesB = static_cast<const unsigned char *>(imageB->rawdata);
      const unsigned char *bytesC = static_cast<const unsigned char *>(imageC->rawdata);

      const float nan = std::numeric_limits<float>::quiet_NaN();

      if(format == VSX_IMAGE_DATA2_FORMAT_COORD3D_A16) {     // TOF
        for(int row = 0; row < height; row++) {
          int index = row * linePitch;
          for(int col = 0; col < width; col++, ++x, ++y, ++z) {
            int16_t datax = 0;
            int16_t datay = 0;
            int16_t dataz = 0;

            std::memcpy(&dataz, bytesC + index, sizeof(uint16_t));

            if(dataz > 0) {
              std::memcpy(&datax, bytesA + index, sizeof(uint16_t));
              std::memcpy(&datay, bytesB + index, sizeof(uint16_t));

              *x = static_cast<float>(datax) / 1000.0f;
              *y = static_cast<float>(datay) / 1000.0f;
              *z = static_cast<float>(dataz) / 1000.0f;
            } else {
              *x = nan;
              *y = nan;
              *z = nan;
            }
            index += sizeof(uint16_t);
          }
        }
      } else if(format == VSX_IMAGE_DATA2_FORMAT_COORD3D_A32F) {   // Stereo
        for(int row = 0; row < height; row++) {
          int index = row * linePitch;
          for(int col = 0; col < width; col++, ++x, ++y, ++z) {
            float datax = nan;
            float datay = nan;
            float dataz = nan;

            std::memcpy(&dataz, bytesC + index, sizeof(float));

            if(dataz > 0.0f) {
              std::memcpy(&datax, bytesA + index, sizeof(float));
              std::memcpy(&datay, bytesB + index, sizeof(float));

              *x = datax / 1000.0f;
              *y = datay / 1000.0f;
              *z = dataz / 1000.0f;
            } else {
              *x = nan;
              *y = nan;
              *z = nan;
            }

            index += sizeof(float);
          }
        }
      }
    }
  }

  return msg;
}
}  // namespace pepperl_fuchs
