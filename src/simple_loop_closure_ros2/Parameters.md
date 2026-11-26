## Parameters
- mapped_cloud  
Set to "true" if the scan data to subscribe has already been mapped to the same coordinate system as the odometry.  
- time_stamp_tolerance  
Unit : sec  
Tolerance for odometry and scan data timestamps.  
- keyframe_dist_th  
Unit : m  
A frame is registered when the distance from the last registered frame is greater than this value.  
- keyframe_angular_dist_th  
Unit : rad  
A frame is registered when the angular distance from the last registered frame is greater than this value.  
- loop_search_time_diff_th  
Unit : sec  
Nodes with a time difference less than this value are excluded from the search.
- loop_search_dist_diff_th  
Unit : m  
Nodes with a trajectory distance difference less than this value are excluded from the search.  
- loop_search_angular_dist_th  
Unit : rad  
Nodes with angular distances greater than this value are excluded from the search.  
This value is provided because LiDAR with limited FoV often fails in registration when the difference in angles is large.  
- loop_search_frame_interval  
Unit : frame  
If there are multiple frames in the buffer, the loop search is skipped at this interval.  
- search_radius  
単位 : m  
Search radius.  
- target_frame_num  
Unit : frame  
Create a submap for registration by merging the frames before and after this value around the frame resulting from the radius search.  
- target_voxel_leaf_size  
Unit : m  
Leaf Size of Voxel Grid Filter for submap.  
- source_voxel_leaf_size  
Unit : m  
Leaf Size of Voxel Grid Filter for scan frames.  
- vis_map_voxel_leaf_size  
Unit : m  
Leaf Size of Voxel Grid Filter for point cloud map to be published to ROS.  
It affects only the point cloud to be Publish.  
- fitness_score_th  
If the Fitness Score of the ICP is below this value, the loop edge is registered.  
- vis_map_cloud_frame_interval  
Skip nodes at this interval when creating a point cloud map to publish.
It affects only the point cloud to be Publish.  

## 2026-11-09 additions

- odom_trans_var / odom_rot_var  
Unit : m^2 / rad^2  
Per-keyframe odometry noise (default 1e-4), in GTSAM order [Rx,Ry,Rz,Tx,Ty,Tz].
- loop_var_scale / loop_var_min  
Unit : - / m^2  
Loop variance = max(loop_var_min, ICP fitness * loop_var_scale).
- loop_constrain_z / loop_constrain_attitude  
If false (default), the loop edge does not constrain that DOF: z or
roll/pitch get loop_var_free. In 3-DOF mode the consistency gate is xy-only.
- loop_var_free  
Unit : m^2 / rad^2  
Variance assigned to a freed DOF. 1e4 is 8 orders weaker than the odometry
(1e-4), i.e. effectively a no-op. Do not use "half relaxed" values like 0.25.
- loop_max_trans_delta / loop_max_trans_delta_ratio  
Unit : m / -  
Consistency gate = max(loop_max_trans_delta, ratio * path length between the
two keyframes). The ratio term keeps long-baseline true closures while
rejecting short-baseline false matches (measured 7–10 m on the corridor).
- enable_go2_z / go2_pose_topic / go2_pose_timeout / go2_z_var  
Go2 relative-z factor: between consecutive keyframes, constrain
(z_i - z_{i-1}) to the /utlidar/robot_pose z difference. The absolute origin
cancels. go2_pose_timeout is in seconds; go2_z_var in m^2. Default off
pending cave validation.
- enable_go2_attitude / go2_attitude_var_deg  
Go2 roll/pitch prior from the same /utlidar/robot_pose quaternion. Yaw and
translation are free; the constant mounting offset is calibrated at the
first valid keyframe. Default off pending cave validation.
- icp_max_corr_dist / vis_incremental / vis_full_rebuild_interval_s  
ICP correspondence distance; incremental visualization-map cache switch and
periodic full rebuild interval.
## 2026-11-19 prior map and start gate

### Prior map (simple_loop_closure)

Off by default; with `enable_prior_map: false` the node behaves exactly as
before and logs `prior map: disabled`.

- `enable_prior_map` (false) — master switch.
- `prior_map_dir` ("") — directory written by `make_prior_map`:
  `poses.csv` (first line must be `# frame: body`) plus `frames/NNNNNN.pcd`
  (body-frame clouds, same convention as `mapped_cloud: true`). A missing
  directory, an asset without the frame header, or zero readable frames
  disables the feature with a WARN rather than aborting the node.
- `prior_search_radius` (1.5) — metres from the predicted pose to the
  nearest prior keyframe; no neighbour, no registration.
- `prior_max_trans_delta` (1.0) — **absolute** cap on the correction the
  prior edge may apply. Unlike `loop_max_trans_delta_ratio` this does not
  scale with path length: a prior correction is accumulated drift, not a
  proportional measurement.
- `prior_icp_max_corr_dist` (1.0) — separate ICP instance; tighter than the
  loop ICP (5 m) so a match cannot slide to a parallel branch behind a thin
  wall.
- `prior_overlap_min` (0.35), `prior_dedup_window` (20), `prior_var_scale`
  (1.0) — same meaning as the loop equivalents; the variance is
  `makeLoopVariances(fitness, prior_var_scale, ...)` projected to [yaw, x, y].
- A match additionally needs `priorSequenceAgrees`: two consecutive query
  keyframes must land on neighbouring prior keyframes, which removes
  single-frame stray matches. Logs: `Prior Accepted` / `Prior Rejected
  (<gate>)`; prior edges are kept out of the loop statistics on purpose.

The prior must be expressed in the **current base frame**; the offline tool
applies the translation (`--translate DX DY`), the crop (`--crop CX CY R`),
the base exclusion disc (`--cut CX CY R`) and the resampling (`--spacing`,
default 0.5 m).

### Start gate (point_lio)

- `wait_for_start` (false) — when true the node still comes up fully
  (filters, map, DDS) but drops every lidar/IMU sample until
  `/start_mapping` (`std_srvs/Empty`) is called, logging
  `waiting for /start_mapping` every 5 s. This keeps the map origin at the
  pose where the operator placed the robot at the base, instead of wherever
  it was during warm-up. Forwarded by
  `full_stack_simple_lc_mid360s.launch.py wait_for_start:=true`.
