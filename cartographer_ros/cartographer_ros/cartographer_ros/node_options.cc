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

#include "cartographer_ros/node_options.h"

#include <vector>

#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/mapping/map_builder.h"
#include "glog/logging.h"

namespace cartographer_ros {

NodeOptions CreateNodeOptions(
    ::cartographer::common::LuaParameterDictionary* const
        lua_parameter_dictionary) {
  NodeOptions options;
  options.map_builder_options =
      ::cartographer::mapping::CreateMapBuilderOptions(
          lua_parameter_dictionary->GetDictionary("map_builder").get());
  options.map_frame = lua_parameter_dictionary->GetString("map_frame");
  options.lookup_transform_timeout_sec =
      lua_parameter_dictionary->GetDouble("lookup_transform_timeout_sec");
  options.submap_publish_period_sec =
      lua_parameter_dictionary->GetDouble("submap_publish_period_sec");
  options.pose_publish_period_sec =
      lua_parameter_dictionary->GetDouble("pose_publish_period_sec");
  options.trajectory_publish_period_sec =
      lua_parameter_dictionary->GetDouble("trajectory_publish_period_sec");
  if (lua_parameter_dictionary->HasKey("use_pose_extrapolator")) {
    options.use_pose_extrapolator =
        lua_parameter_dictionary->GetBool("use_pose_extrapolator");

  //okagv
    options.root_file_directory =
        lua_parameter_dictionary->GetString("root_file_directory");
    options.imu_publish_period_sec =
        lua_parameter_dictionary->GetDouble("imu_publish_period_sec");
    options.use_pose_smoother =
        lua_parameter_dictionary->GetBool("use_pose_smoother");
    options.smoother_variety_distance =
        lua_parameter_dictionary->GetDouble("smoother_variety_distance");
    options.relocalization_variety_distance =
        lua_parameter_dictionary->GetDouble("relocalization_variety_distance");
    options.time_delay_for_relocalization =
        lua_parameter_dictionary->GetDouble("time_delay_for_relocalization");
    options.time_delay_for_finish_trajectory =
        lua_parameter_dictionary->GetDouble("time_delay_for_finish_trajectory");
    options.time_delay_for_delete_trajectory =
        lua_parameter_dictionary->GetDouble("time_delay_for_delete_trajectory");
    options.time_delay_for_start_trajectory =
        lua_parameter_dictionary->GetDouble("time_delay_for_start_trajectory");
  }
  return options;
}

std::tuple<NodeOptions, TrajectoryOptions> LoadOptions(
    const std::string& configuration_directory,
    const std::string& configuration_basename) {
  auto file_resolver =
      absl::make_unique<cartographer::common::ConfigurationFileResolver>(
          std::vector<std::string>{configuration_directory});
  const std::string code =
      file_resolver->GetFileContentOrDie(configuration_basename);

  cartographer::common::LuaParameterDictionary lua_parameter_dictionary(
      code,  std::move(file_resolver));

  return std::make_tuple(CreateNodeOptions(&lua_parameter_dictionary),
                         CreateTrajectoryOptions(&lua_parameter_dictionary));
}

NodeOptions LoadNodeOptions(
    const std::string& configuration_directory,
    const std::string& configuration_basename){
          auto file_resolver =
      absl::make_unique<cartographer::common::ConfigurationFileResolver>(
          std::vector<std::string>{configuration_directory});
  const std::string code =
      file_resolver->GetFileContentOrDie(configuration_basename);
  cartographer::common::LuaParameterDictionary lua_parameter_dictionary(
      code, std::move(file_resolver));

  return CreateNodeOptions(&lua_parameter_dictionary);
    }

}  // namespace cartographer_ros
