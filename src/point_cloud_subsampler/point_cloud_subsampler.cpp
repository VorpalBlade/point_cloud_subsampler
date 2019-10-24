// Point Cloud SubSampler - Subsamples point clouds
// Copyright (C) 2019  Arvid Norlander
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include "point_cloud_subsampler/point_cloud_subsampler.h"

#include <Eigen/Core>
#include <unordered_map>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <ros_spatial_utils/scaled_map.h>
#include <boost/functional/hash.hpp>
#include <tf2/convert.h>

namespace std
{
//! Specialisation for unordered_map key type
template <>
struct hash<Eigen::Vector2i>
{
  //! Functor operator
  inline size_t operator()(const Eigen::Vector2i& v) const
  {
    size_t h = 0;
    boost::hash_combine(h, v(0));
    boost::hash_combine(h, v(1));
    return h;
  }
};
}  // namespace std

namespace point_cloud_subsampler
{
namespace
{
using Bucketer = ros_spatial_utils::ScaledInfiniteMapLogic<2>;

using PointCoordinate = Bucketer::MapCoordinate;

//! Defines bucket data
struct PointBucket
{
  bool point_in_obstacle = false;
};
using BucketMap = std::unordered_map<PointCoordinate, PointBucket>;

}  // anonymous namespace

PointCloudSubSampler::PointCloudSubSampler(ros::NodeHandle& nh, ros::NodeHandle& nh_priv)
  : nh_(nh), tf_listener_(tf_buffer_), srv_(nh_priv)
{
  ros::SubscriberStatusCallback connect_cb = boost::bind(&PointCloudSubSampler::connectCb, this);
  pub_ = nh.advertise<sensor_msgs::PointCloud2>("subsampled_points", 10, connect_cb, connect_cb);

  srv_.setCallback(boost::bind(&PointCloudSubSampler::reconfig, this, _1, _2));
}

void PointCloudSubSampler::connectCb()
{
  // This allows us to only do work if someone is subscribed to us.
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  if (!pub_.getNumSubscribers())
  {
    pc_sub_.shutdown();
  }
  else
  {
    pc_sub_ = nh_.subscribe("points", 10, &PointCloudSubSampler::recvCallback, this);
  }
}

//! Implements the core logic, shared between fast version and iterator version.
#define CORE_LOGIC(x_expr, y_expr, z_expr)                                                                             \
  const float z = z_expr;                                                                                              \
  const float x = x_expr;                                                                                              \
  const float y = y_expr;                                                                                              \
  auto range = sqrtf(x * x + y * y);                                                                                   \
  if (range > cfg_.max_dist)                                                                                           \
  {                                                                                                                    \
    continue;                                                                                                          \
  }                                                                                                                    \
                                                                                                                       \
  const auto bin = bucket_assigner.worldToMap({ x, y });                                                               \
  auto output_it = output.find(bin);                                                                                   \
  if (output_it == output.end())                                                                                       \
  {                                                                                                                    \
    auto result = output.emplace(bin, PointBucket{});                                                                  \
    output_it = result.first;                                                                                          \
  }                                                                                                                    \
                                                                                                                       \
  if (z > cfg_.min_z && z < cfg_.max_z)                                                                                \
  {                                                                                                                    \
    output_it->second.point_in_obstacle = true;                                                                        \
  }

void PointCloudSubSampler::recvCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  geometry_msgs::TransformStamped transformStamped;
  sensor_msgs::PointCloud2 transformed;
  try
  {
    transformStamped = tf_buffer_.lookupTransform(cfg_.target_frame, msg->header.frame_id, msg->header.stamp,
                                                  ros::Duration(cfg_.tf_timeout));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    return;
  }
  tf2::doTransform(*msg, transformed, transformStamped);

  // Load structure of PointCloud2
  auto offsets = get_offsets(transformed);

  if (offsets.x < 0 || offsets.y < 0 || offsets.z < 0)
  {
    ROS_ERROR("PointCloudSubsampler: PointCloud2 missing one or more required "
              "fields! (x,y,z)");
    return;
  }

  Bucketer bucket_assigner;
  bucket_assigner.setResolutions({ cfg_.resolution, cfg_.resolution });

  BucketMap output;

  // Process input message
  if ((offsets.x == 0) && (offsets.y == 4) && (offsets.z == 8))
  {
    for (sensor_msgs::PointCloud2ConstIterator<float> it(transformed, "x"); it != it.end(); ++it)
    {
      CORE_LOGIC(it[0], it[1], it[2])
    }
  }
  else
  {
    ROS_WARN_ONCE("PointCloudSubsampler: PointCloud2 fields in unexpected "
                  "order. Using slower generic method.");

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(transformed, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(transformed, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(transformed, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
      CORE_LOGIC(*iter_x, *iter_y, *iter_z)
    }
  }

  // Prepare output message.
  sensor_msgs::PointCloud2Ptr output_msg(new sensor_msgs::PointCloud2());
  output_msg->header = msg->header;
  output_msg->header.frame_id = cfg_.target_frame;
  output_msg->height = 1;
  output_msg->width = static_cast<decltype(output_msg->width)>(output.size());

  sensor_msgs::PointCloud2Modifier modifier(*output_msg);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(output.size());
  sensor_msgs::PointCloud2Iterator<float> iter_x(*output_msg, "x");

  auto free_height = static_cast<float>(cfg_.free_z);
  auto occupied_height = static_cast<float>(cfg_.obstacle_z);

  for (const auto& entry : output)
  {
    auto world_coordinate = bucket_assigner.mapToWorld(entry.first);
    iter_x[0] = world_coordinate(0);
    iter_x[1] = world_coordinate(1);
    iter_x[2] = entry.second.point_in_obstacle ? occupied_height : free_height;
    ++iter_x;
  }

  // Publish output message
  pub_.publish(output_msg);
}

void PointCloudSubSampler::reconfig(PointCloudSubSamplerConfig& config, uint32_t)
{
  cfg_ = config;
}

PointCloudSubSampler::Offsets PointCloudSubSampler::get_offsets(const sensor_msgs::PointCloud2& msg)
{
  Offsets offsets;

  for (const auto& field : msg.fields)
  {
    if (field.datatype == sensor_msgs::PointField::FLOAT32)
    {
      if (field.name == "x")
      {
        offsets.x = static_cast<int>(field.offset);
      }
      else if (field.name == "y")
      {
        offsets.y = static_cast<int>(field.offset);
      }
      else if (field.name == "z")
      {
        offsets.z = static_cast<int>(field.offset);
      }
    }
  }

  return offsets;
}

}  // namespace point_cloud_subsampler
