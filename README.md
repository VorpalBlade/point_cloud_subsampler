# Point cloud subsampler

Subsample a point cloud to lower density. Intended usage is as input to
a costmap or other obstacle avoidance system.

This nodelet splits the point cloud into grid cells and puts one z level into
each cell that contains an obstacle and another into each cell that contains no
obstacles.

This node will also remap into the target TF frame (**before** processing). This
is useful if your point cloud TF frame is in a weird orientation.

[[doc/rviz_subsampler.png|alt=Example of subsampling a pointcloud]]

## Topic

* `points` (`sensor_msgs/PointCloud2`) - Input high density cloud.
* `subsampled_points` (`sensor_msgs/PointCloud2`) - Output low density cloud.

The node only does processing if it has subscribers.

## Parameters

Dynamic reconfigure is supported.

* `min_z` (`double`, default: 0.0) - Minimum z for obstacles (m)
* `max_z` (`double`, default: 2.0) - Maximum z for obstacles (m)
* `free_z` (`double`, default: 0.0) - Published z for free space (m)
* `obstacle_z` (`double`, default: 1.0) - Published z for obstacles (m)
* `resolution` (`double`, default: 0.0) - Distance between output points (m)
* `max_dist` (`double`, default: 4.0) - Maximum Euclidean distance from sensor in the plane (m)
* `target_frame` (`string`, default: "base_footprint") - Target TF frame
