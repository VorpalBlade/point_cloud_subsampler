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

#include <ros/ros.h>
#include <point_cloud_subsampler/point_cloud_subsampler.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "point_cloud_subsampler");
  ros::NodeHandle node;
  ros::NodeHandle priv_nh("~");

  point_cloud_subsampler::PointCloudSubSampler n(node, priv_nh);

  ros::spin();

  return 0;
}
