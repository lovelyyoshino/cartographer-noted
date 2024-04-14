-- Copyright 2018 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua" --okagv

--okagv
options ={
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link", --base_link
  published_frame = "base_link", --base_link
  odom_frame = "odom", --odom
  provide_odom_frame = false, --true
  publish_frame_projected_to_2d = true, --false
  use_pose_extrapolator = true, --true
  use_odometry = false, --false
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1, --0
  num_multi_echo_laser_scans = 0, --1
  num_subdivisions_per_laser_scan = 1, --10
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3, --5e-3
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 0.5, --1.
  odometry_sampling_ratio = 1., --1.
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,

  root_file_directory = "/okagv",
  imu_publish_period_sec = 2e-2,
  use_pose_smoother = true, --true
  smoother_variety_distance = 0.05,
  relocalization_variety_distance = 0.1,
  time_delay_for_relocalization = 0.0,
  time_delay_for_finish_trajectory = 0.0,
  time_delay_for_delete_trajectory = 0.0,
  time_delay_for_start_trajectory = 0.0,

  num_event_threads = 4, --4
  num_grpc_threads = 4,
  server_address = "0.0.0.0:50051",
  uplink_server_address = "",
  upload_batch_size = 100,
  enable_ssl_encryption = false,
  enable_google_auth = false
}

MAP_BUILDER.collate_by_trajectory = false --true

--TRAJECTORY_BUILDER_2D.max_range = 20.
TRAJECTORY_BUILDER_2D.min_range = 0.5
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 60 --90
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5. --5.
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.insert_free_space = true --true
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 10. --1.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10. --10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40. --40.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.intensity_space_weight = 0.5 --1.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20 --20
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05 --0.05

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.pose_graph.constraint_builder.log_matches = false
MAP_BUILDER.pose_graph.optimize_every_n_nodes = 60 --90
MAP_BUILDER.pose_graph.optimization_problem.log_solver_summary = false
--MAP_BUILDER_SERVER.map_builder.pose_graph.constraint_builder.sampling_ratio = 0.5
MAP_BUILDER.num_background_threads = 1
--MAP_BUILDER_SERVER.map_builder.pose_graph.constraint_builder.loop_closure_translation_weight = 1.1e3
--MAP_BUILDER_SERVER.map_builder.pose_graph.constraint_builder.loop_closure_rotation_weight = 1.1e3
--MAP_BUILDER_SERVER.map_builder.pose_graph.constraint_builder.min_score = 0.7

--TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1 --0.1
--TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.) --20.
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

TRAJECTORY_BUILDER_2D.use_intensity_ceres_scan_matching = false

TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 1. --5.
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.2 --0.2
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = 1. --1.

MAP_BUILDER.pose_graph.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 7 --7
MAP_BUILDER.pose_graph.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.) --math.rad(30
MAP_BUILDER.pose_graph.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 7 --7

MAP_BUILDER.pose_graph.global_sampling_ratio = 0.003 --0.003
MAP_BUILDER.pose_graph.log_residual_histograms = false --true
MAP_BUILDER.pose_graph.global_constraint_search_after_n_seconds = 10 --10

MAP_BUILDER.pose_graph.constraint_builder.min_score = 0.55 --0.55
MAP_BUILDER.pose_graph.constraint_builder.sampling_ratio = 0.3 --0.3
MAP_BUILDER.pose_graph.constraint_builder.max_constraint_distance = 6 --15
MAP_BUILDER.pose_graph.constraint_builder.global_localization_min_score = 0.6 --0.6
MAP_BUILDER.pose_graph.constraint_builder.loop_closure_translation_weight = 1.1e4 --1.1e4
MAP_BUILDER.pose_graph.constraint_builder.loop_closure_rotation_weight = 1e5 --1e5
MAP_BUILDER.pose_graph.optimization_problem.huber_scale = 1e1 --1e1
MAP_BUILDER.pose_graph.optimization_problem.odometry_translation_weight = 1e1 --1e5
MAP_BUILDER.pose_graph.optimization_problem.odometry_rotation_weight = 1e1 --1e5
MAP_BUILDER.pose_graph.optimization_problem.ceres_solver_options.num_threads = 2 --7

--okagv
MAP_BUILDER.pose_graph.optimize_every_meter = 0.5 --0.5
MAP_BUILDER.pose_graph.constraint_builder.max_match_variety_distance = 1.0

MAP_BUILDER.pose_graph.constraint_builder.ceres_scan_matcher.occupied_space_weight = 20 --20.
MAP_BUILDER.pose_graph.constraint_builder.ceres_scan_matcher.translation_weight = 10 --10.
MAP_BUILDER.pose_graph.constraint_builder.ceres_scan_matcher.rotation_weight = 1. --1.

--TRAJECTORY_BUILDER_2D.submaps.num_range_data = 180

--MAP_BUILDER.pose_graph.has_overlapping_submaps_trimmer_2d = true --true --no this param
MAP_BUILDER.pose_graph.overlapping_submaps_trimmer_2d.fresh_submaps_count = 1 --1
MAP_BUILDER.pose_graph.overlapping_submaps_trimmer_2d.min_covered_area = 2 --2
MAP_BUILDER.pose_graph.overlapping_submaps_trimmer_2d.min_added_submaps_count = 5 --5

return options
