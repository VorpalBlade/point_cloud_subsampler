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

#include <boost/thread/lock_guard.hpp>
#include <boost/thread/mutex.hpp>
#include <dynamic_reconfigure/server.h>
#include <message_filters/subscriber.h>
#include <point_cloud_subsampler/PointCloudSubSamplerConfig.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

// Warnings inside template instantiation
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <tf2_ros/message_filter.h>
#pragma GCC diagnostic pop

namespace point_cloud_subsampler
{
class PointCloudSubSampler
{
public:
  PointCloudSubSampler(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);

private:
  boost::mutex connect_mutex_;

  //! Connection callback
  void connectCb();

  //! Point cloud callback
  void recvCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

  ros::NodeHandle nh_;
  ros::Publisher pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  //! @name Subscribers
  //! @{

  //! Type of point cloud subscriber
  using PcSubscriber = message_filters::Subscriber<sensor_msgs::PointCloud2>;
  //! Type of point cloud filter
  using PcFilter = tf2_ros::MessageFilter<sensor_msgs::PointCloud2>;

  //! Special subscription used by @a laser_filter to sync the messages with TF
  //! transforms
  PcSubscriber pc_sub_;
  //! Message filter for point cloud.
  std::shared_ptr<PcFilter> pc_filter_;
  //! @}

  PointCloudSubSamplerConfig cfg_;
  dynamic_reconfigure::Server<PointCloudSubSamplerConfig> srv_;
  void reconfig(PointCloudSubSamplerConfig& config, uint32_t level);

  //! Offsets in PointCloud2 message
  struct Offsets
  {
    int x = -1;
    int y = -1;
    int z = -1;
  };

  static Offsets get_offsets(const sensor_msgs::PointCloud2& msg);
};

}  // namespace point_cloud_subsampler
