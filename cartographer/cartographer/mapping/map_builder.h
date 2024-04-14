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

#ifndef CARTOGRAPHER_MAPPING_MAP_BUILDER_H_
#define CARTOGRAPHER_MAPPING_MAP_BUILDER_H_

#include <memory>

#include "cartographer/common/thread_pool.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/mapping/pose_graph.h"
#include "cartographer/mapping/proto/map_builder_options.pb.h"
#include "cartographer/sensor/collator_interface.h"


using TrajectoryType 
      = cartographer::mapping::MapBuilderInterface::TrajectoryType;

namespace cartographer {
namespace mapping {

proto::MapBuilderOptions CreateMapBuilderOptions(
    common::LuaParameterDictionary *const parameter_dictionary);

// Wires up the complete SLAM stack with TrajectoryBuilders (for local submaps)
// and a PoseGraph for loop closure.
class MapBuilder : public MapBuilderInterface {
 public:
  //explicit MapBuilder(const proto::MapBuilderOptions &options); //default
   explicit MapBuilder(proto::MapBuilderOptions &options); //okagv
  ~MapBuilder() override {}

  MapBuilder(const MapBuilder &) = delete;
  MapBuilder &operator=(const MapBuilder &) = delete;

  int AddTrajectoryBuilder(
      const std::set<SensorId> &expected_sensor_ids,
      const proto::TrajectoryBuilderOptions &trajectory_options,
      LocalSlamResultCallback local_slam_result_callback) override;

  int AddTrajectoryForDeserialization(
      const proto::TrajectoryBuilderOptionsWithSensorIds
          &options_with_sensor_ids_proto) override;

  void FinishTrajectory(int trajectory_id) override;

  std::string SubmapToProto(const SubmapId &submap_id,
                            proto::SubmapQuery::Response *response) override;

  void SerializeState(bool include_unfinished_submaps,
                      io::ProtoStreamWriterInterface *writer) override;

  bool SerializeStateToFile(bool include_unfinished_submaps,
                            const std::string &filename) override;
  
  bool SerializeStateToFileWithId(int trajectory_id,
                            bool include_unfinished_submaps,
                            const std::string &filename) override;

  std::map<int, int> LoadState(io::ProtoStreamReaderInterface *reader,
                               const cartographer::mapping::PoseGraphInterface::TrajectoryState& state) override;

  std::map<int, int> LoadStateFromFile(const std::string &filename,
                                       const cartographer::mapping::PoseGraphInterface::TrajectoryState& state) override;

  //okagv 
  void SetTrajectoryTypeWithId(TrajectoryType type, int id) override;

  //okagv
  TrajectoryType GetTrajectoryTypeWithId(int id) override;
  
  //okagv
  TrajectoryType GetWorkingTrajectoryType() override;

  //okagv
  int GetTrajectoryIdByName(std::string) override;

  //okagv
  void DeleteTrajectory(int trajectory_id) override;

  //okagv
  bool SerializeStateToFileAfterUpdate(bool include_unfinished_submaps,
                                    const std::string& filename) override;
  
  mapping::PoseGraphInterface *pose_graph() override {
    return pose_graph_.get();
  }

  int num_trajectory_builders() const override {
    return trajectory_builders_.size();
  }

  mapping::TrajectoryBuilderInterface *GetTrajectoryBuilder(
      int trajectory_id) const override {
    return trajectory_builders_.at(trajectory_id).get();
  }

  const std::map<int, proto::TrajectoryBuilderOptionsWithSensorIds>
      &GetAllTrajectoryBuilderOptions() const override {
    return all_trajectory_builder_options_;
  }

  //okagv
  void RegisterClientIdForTrajectory(const std::string& client_id, int trajectory_id) override;

  //okagv
  void SetMapBuilderOptions(proto::MapBuilderOptions &option_reset) override;

 private:

  //const proto::MapBuilderOptions options_; 
  //okagv
  proto::MapBuilderOptions options_; 

  common::ThreadPool thread_pool_;

  std::unique_ptr<PoseGraph> pose_graph_;

  std::unique_ptr<sensor::CollatorInterface> sensor_collator_;
  //std::vector<std::unique_ptr<mapping::TrajectoryBuilderInterface>>
  //    trajectory_builders_;
  //std::vector<proto::TrajectoryBuilderOptionsWithSensorIds>
  //    all_trajectory_builder_options_;   

  std::map<int, std::unique_ptr<mapping::TrajectoryBuilderInterface>> 
      trajectory_builders_; 
  std::map<int, proto::TrajectoryBuilderOptionsWithSensorIds> 
      all_trajectory_builder_options_; 

  TrajectoryType current_trajectory_type_ = TrajectoryType::IDLE;
  // 1000-default load map trajectory id
  // 1001-default slam map trajectory id
  // 1002-default localization map trajectory id
  int current_trajectory_id_ = 1000;

  //okagv
  std::map</*trajectory_id*/int, /*trajectory_name*/std::string> client_ids_;

};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_MAP_BUILDER_H_
