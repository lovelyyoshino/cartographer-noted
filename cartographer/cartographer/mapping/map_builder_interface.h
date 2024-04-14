/*
 * Copyright 2017 The Cartographer Authors
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

#ifndef CARTOGRAPHER_MAPPING_MAP_BUILDER_INTERFACE_H_
#define CARTOGRAPHER_MAPPING_MAP_BUILDER_INTERFACE_H_

#include <set>
#include <string>
#include <vector>

#include "Eigen/Geometry"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/io/proto_stream_interface.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/pose_graph_interface.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/mapping/trajectory_builder_interface.h"

//okagv
#include "cartographer/mapping/pose_extrapolator_interface.h"
#include "cartographer/mapping/proto/map_builder_options.pb.h"

namespace cartographer {
namespace mapping {

// This interface is used for both library and RPC implementations.
// Implementations wire up the complete SLAM stack.
class MapBuilderInterface {

//okagv
struct OkagvOrderStartTrajectory{
     std::string trajectory_type;
     std::string trajectory_id;
     bool use_initial_pose;
     ::cartographer::transform::Rigid3d initial_pose;
     std::string relative_to_trajectory_id;
};

//okagv
struct OkagvOrderSaveTrajectory{
      std::string filename;
      bool include_unfinished_submaps;
};

 public:
  using LocalSlamResultCallback =
      TrajectoryBuilderInterface::LocalSlamResultCallback;

  using SensorId = TrajectoryBuilderInterface::SensorId;

  enum class TrajectoryType { SLAM, LOAD, NAVIGATION, RELOCALIZAION, IDLE, ABORTION};

  MapBuilderInterface() {}
  virtual ~MapBuilderInterface() {}

  MapBuilderInterface(const MapBuilderInterface&) = delete;
  MapBuilderInterface& operator=(const MapBuilderInterface&) = delete;

  // Creates a new trajectory builder and returns its index.
  virtual int AddTrajectoryBuilder(
      const std::set<SensorId>& expected_sensor_ids,
      const proto::TrajectoryBuilderOptions& trajectory_options,
      LocalSlamResultCallback local_slam_result_callback) = 0;

  // Creates a new trajectory and returns its index. Querying the trajectory
  // builder for it will return 'nullptr'.
  virtual int AddTrajectoryForDeserialization(
      const proto::TrajectoryBuilderOptionsWithSensorIds&
          options_with_sensor_ids_proto) = 0;

  // Returns the 'TrajectoryBuilderInterface' corresponding to the specified
  // 'trajectory_id' or 'nullptr' if the trajectory has no corresponding
  // builder.
  virtual mapping::TrajectoryBuilderInterface* GetTrajectoryBuilder(
      int trajectory_id) const = 0;

  // Marks the TrajectoryBuilder corresponding to 'trajectory_id' as finished,
  // i.e. no further sensor data is expected.
  virtual void FinishTrajectory(int trajectory_id) = 0;

  // Fills the SubmapQuery::Response corresponding to 'submap_id'. Returns an
  // error string on failure, or an empty string on success.
  virtual std::string SubmapToProto(const SubmapId& submap_id,
                                    proto::SubmapQuery::Response* response) = 0;

  // Serializes the current state to a proto stream. If
  // 'include_unfinished_submaps' is set to true, unfinished submaps, i.e.
  // submaps that have not yet received all rangefinder data insertions, will
  // be included in the serialized state.
  virtual void SerializeState(bool include_unfinished_submaps,
                              io::ProtoStreamWriterInterface* writer) = 0;

  // Serializes the current state to a proto stream file on the host system. If
  // 'include_unfinished_submaps' is set to true, unfinished submaps, i.e.
  // submaps that have not yet received all rangefinder data insertions, will
  // be included in the serialized state.
  // Returns true if the file was successfully written.
  virtual bool SerializeStateToFile(bool include_unfinished_submaps,
                                    const std::string& filename) = 0;

  // Loads the SLAM state from a proto stream. Returns the remapping of new
  // trajectory_ids.
  virtual std::map<int /* trajectory id in proto */, int /* trajectory id */>
  LoadState(io::ProtoStreamReaderInterface* reader, const cartographer::mapping::PoseGraphInterface::TrajectoryState& state) = 0;

  // Loads the SLAM state from a pbstream file. Returns the remapping of new
  // trajectory_ids.
  virtual std::map<int /* trajectory id in proto */, int /* trajectory id */>
  LoadStateFromFile(const std::string& filename, const cartographer::mapping::PoseGraphInterface::TrajectoryState& state) = 0;

  virtual int num_trajectory_builders() const = 0;

  virtual mapping::PoseGraphInterface* pose_graph() = 0;

  virtual const std::map<int, proto::TrajectoryBuilderOptionsWithSensorIds>&
  GetAllTrajectoryBuilderOptions() const = 0;

  //okagv
  virtual void SetTrajectoryTypeWithId(TrajectoryType type, int id) = 0;

  //okagv
  virtual TrajectoryType GetTrajectoryTypeWithId(int id) = 0;

  //okagv
  virtual  void DeleteTrajectory(int trajectory_id) = 0;

  //okagv
  virtual  bool SerializeStateToFileWithId(int trajectory_id,
                            bool include_unfinished_submaps,
                            const std::string &filename) = 0;
  //okagv
  virtual bool SerializeStateToFileAfterUpdate(bool include_unfinished_submaps,
                                    const std::string& filename) = 0;

  //okagv
  virtual TrajectoryType GetWorkingTrajectoryType() = 0;

  virtual int GetTrajectoryIdByName(std::string name) = 0;

  //okagv
  virtual void RegisterClientIdForTrajectory(const std::string& client_id, int trajectory_id) = 0;

   //okagv
   OkagvOrderStartTrajectory okagv_order_start_trajectory_;
   OkagvOrderSaveTrajectory okagv_order_save_trajectory_;

   virtual void SetMapBuilderOptions(proto::MapBuilderOptions &option_reset) = 0;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_MAP_BUILDER_INTERFACE_H_
