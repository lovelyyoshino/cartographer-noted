/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "absl/memory/memory.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer_ros/node.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/ros_log_sink.h"
#include "gflags/gflags.h"
#include "tf2_ros/transform_listener.h"

//okagv
#include "cartographer/cloud/map_builder_server_interface.h"
#include "cartographer_ros/map_builder_server_options.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer/metrics/register.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "cartographer/cloud/internal/map_builder_server.h"

DEFINE_bool(collect_metrics, false,
            "Activates the collection of runtime metrics. If activated, the "
            "metrics can be accessed via a ROS service.");
DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_string(load_state_filename, "",
              "If non-empty, filename of a .pbstream file to load, containing "
              "a saved SLAM state.");
DEFINE_bool(load_frozen_state, true,
            "Load the saved state as frozen (non-optimized) trajectories.");
DEFINE_bool(
    start_trajectory_with_default_topics, true,
    "Enable to immediately start the first trajectory with default topics.");
DEFINE_string(
    save_state_filename, "",
    "If non-empty, serialize state and write it to disk before shutting down.");

DEFINE_string(configuration_basename_server, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");

//okagv
DEFINE_string(configuration_slam_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_string(configuration_navigation_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");

namespace cartographer_ros {
namespace {

void Run() {
  constexpr double kTfBufferCacheTimeInSeconds = 10.;
  tf2_ros::Buffer tf_buffer{::ros::Duration(kTfBufferCacheTimeInSeconds)};
  tf2_ros::TransformListener tf(tf_buffer);
  NodeOptions node_options;
  //node_options = LoadNodeOptions(FLAGS_configuration_directory,
  //                 FLAGS_configuration_basename_server);
  
  TrajectoryOptions trajectory_options;
  std::tie(node_options, trajectory_options) =
       LoadOptions(FLAGS_configuration_directory,
                   FLAGS_configuration_basename_server);
  
  //auto map_builder = absl::make_unique<cartographer::mapping::MapBuilder>(
  //    node_options.map_builder_options);

  //LOG(INFO) << "Peak.ding FLAGS_configuration_directory " << FLAGS_configuration_directory;
  //LOG(INFO) << "Peak.ding FLAGS_configuration_slam_basename " << FLAGS_configuration_slam_basename;
  //LOG(INFO) << "Peak.ding FLAGS_configuration_navigation_basename " << FLAGS_configuration_navigation_basename;

  /*
  cartographer::cloud::proto::MapBuilderServerOptions
      map_builder_server_options =
          cartographer::cloud::LoadMapBuilderServerOptions(
              FLAGS_configuration_directory,
              FLAGS_configuration_basename_server);
              */

  //slam
  cartographer::cloud::proto::MapBuilderServerOptions
      map_builder_slam_options =
          cartographer::cloud::LoadMapBuilderServerOptions(
              FLAGS_configuration_directory,
              FLAGS_configuration_slam_basename);

  //navigation
  cartographer::cloud::proto::MapBuilderServerOptions
      map_builder_navigation_options =
          cartographer::cloud::LoadMapBuilderServerOptions(
              FLAGS_configuration_directory,
              FLAGS_configuration_navigation_basename);

  //LOG(INFO) << "Peak,ding DDDDD " << map_builder_navigation_options.map_builder_options().pose_graph_options().optimize_every_n_nodes();

  //LOG(INFO) << "Peak.ding map_builder";
  auto map_builder =
      std::make_shared<cartographer::mapping::MapBuilder>(  // okagv default is
                                                            // absl::make_unique
         const_cast<::cartographer::mapping::proto::MapBuilderOptions&>(map_builder_slam_options.map_builder_options()));

  std::unique_ptr<cartographer::cloud::MapBuilderServerInterface>
      map_builder_server =
          absl::make_unique<cartographer::cloud::MapBuilderServer>(
              map_builder_slam_options, map_builder);

  //LOG(INFO) << "Peak.ding map_builder_server start";
  map_builder_server->Start();
  
  //Node node(node_options, std::move(map_builder), &tf_buffer, FLAGS_collect_metrics);
  //LOG(INFO) << "Peak.ding Node start";
  Node node(node_options, map_builder, &tf_buffer, FLAGS_collect_metrics);

  /*          
  if (!FLAGS_load_state_filename.empty()) {
    node.LoadState(FLAGS_load_state_filename, FLAGS_load_frozen_state);
  }
  */
  //LOG(INFO) << "Peak.ding root_file_directory " << node_options.root_file_directory;

  if (!FLAGS_configuration_directory.empty()) {
    node.SetConfigurationParam(FLAGS_configuration_directory,
                               FLAGS_configuration_slam_basename,
                               FLAGS_configuration_navigation_basename);

    node.RecordMapBuilderOption(map_builder_slam_options,
                                map_builder_navigation_options);
  }

  if (!node_options.root_file_directory.empty()) {
    //node.LoadMaps(node_options.root_file_directory + "/map");
  }

  if (FLAGS_start_trajectory_with_default_topics) {
    // node.StartTrajectoryWithDefaultTopics(trajectory_options);
  }

  //ros::AsyncSpinner async_spinner(2);
  //async_spinner.start();
  //::ros::waitForShutdown();

  //ros::MultiThreadedSpinner spinner(2);
  //spinner.spin();
  //LOG(INFO) << "Peak.ding Ros Start";
  ::ros::spin();
  map_builder_server->Shutdown();
  node.FinishAllTrajectories();
  node.RunFinalOptimization();

  if (!FLAGS_save_state_filename.empty()) {
    node.SerializeState(FLAGS_save_state_filename,
                        true /* include_unfinished_submaps */);
  }
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_slam_basename.empty())
      << "-FLAGS_configuration_slam_basename is missing.";
  CHECK(!FLAGS_configuration_navigation_basename.empty())
      << "-FLAGS_configuration_localization_basename is missing.";

  ::ros::init(argc, argv, "cartographer_node");
  ::ros::start();

  cartographer_ros::ScopedRosLogSink ros_log_sink;
  cartographer_ros::Run();
  ::ros::shutdown();
}
