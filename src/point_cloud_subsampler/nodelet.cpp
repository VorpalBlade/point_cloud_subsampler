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
#include <memory>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace point_cloud_subsampler {

class PointCloudSubsamplerNodelet : public nodelet::Nodelet
{
public:
  PointCloudSubsamplerNodelet() {}
  ~PointCloudSubsamplerNodelet() {}

private:
  virtual void onInit()
  {
    node_.reset(
        new PointCloudSubSampler(getNodeHandle(), getPrivateNodeHandle()));
  }

  std::shared_ptr<PointCloudSubSampler> node_;
};

} // namespace point_cloud_subsampler

PLUGINLIB_EXPORT_CLASS(point_cloud_subsampler::PointCloudSubsamplerNodelet,
                       nodelet::Nodelet);
