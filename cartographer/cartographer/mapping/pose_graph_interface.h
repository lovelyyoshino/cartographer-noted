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

#ifndef CARTOGRAPHER_MAPPING_POSE_GRAPH_INTERFACE_H_
#define CARTOGRAPHER_MAPPING_POSE_GRAPH_INTERFACE_H_

#include <chrono>
#include <vector>

#include "absl/types/optional.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/transform/rigid_transform.h"

#include "cartographer/mapping/proto/pose_graph_options.pb.h"

namespace cartographer {
namespace mapping {

class PoseGraphInterface {
 public:
  // A "constraint" as in the paper by Konolige, Kurt, et al. "Efficient sparse
  // pose adjustment for 2d mapping." Intelligent Robots and Systems (IROS),
  // 2010 IEEE/RSJ International Conference on (pp. 22--29). IEEE, 2010.
  struct Constraint {
    struct Pose {
      transform::Rigid3d zbar_ij;
      double translation_weight;
      double rotation_weight;
    };

    SubmapId submap_id;  // 'i' in the paper.
    NodeId node_id;      // 'j' in the paper.

    // Pose of the node 'j' relative to submap 'i'.
    Pose pose;

    // Differentiates between intra-submap (where node 'j' was inserted into
    // submap 'i') and inter-submap constraints (where node 'j' was not inserted
    // into submap 'i').
    enum Tag { INTRA_SUBMAP, INTER_SUBMAP } tag;
  };

  struct LandmarkNode {
    struct LandmarkObservation {
      int trajectory_id;
      common::Time time;
      transform::Rigid3d landmark_to_tracking_transform;
      double translation_weight;
      double rotation_weight;
    };
    std::vector<LandmarkObservation> landmark_observations;
    absl::optional<transform::Rigid3d> global_landmark_pose;
    bool frozen = false;
    //okagv
    double confidence_score;
  };

  struct SubmapPose {
    int version;
    transform::Rigid3d pose;
  };

  struct SubmapData {
    std::shared_ptr<const Submap> submap;
    transform::Rigid3d pose;
  };

  struct TrajectoryData {
    double gravity_constant = 9.8;
    std::array<double, 4> imu_calibration{{1., 0., 0., 0.}};
    absl::optional<transform::Rigid3d> fixed_frame_origin_in_map;
  };

  enum class TrajectoryState { ACTIVE, FINISHED, FROZEN, DELETED, IDLE};

  enum class ThreadPoolState { ACTIVE, IDLE};

  enum class OKagvOrder { START, FINISH, WAIT, SAVE, LOCALIZE};

  enum class OKagvState {PROCESSING, SUCCESS, FAIL};

  /*---------------------------------------------------------------
                           OKagvFeedback
     0 - no error
     1 -  
     2 -  
     3 -
     4 -
     5 -
     6 -
     7 -
     8 -
     9 -
  ----------------------------------------------------------------*/

    struct OKagvFeedback{
      int code;
      std::string message;
      OKagvState state;
  };

  using GlobalSlamOptimizationCallback =
      std::function<void(const std::map<int /* trajectory_id */, SubmapId>&,
                         const std::map<int /* trajectory_id */, NodeId>&)>;

  PoseGraphInterface() {}
  virtual ~PoseGraphInterface() {}

  PoseGraphInterface(const PoseGraphInterface&) = delete;
  PoseGraphInterface& operator=(const PoseGraphInterface&) = delete;

  // Waits for all computations to finish and computes optimized poses.
  virtual void RunFinalOptimization() = 0;

  // Returns data for all submaps.
  virtual MapById<SubmapId, SubmapData> GetAllSubmapData() const = 0;

  // okagv
  virtual MapById<SubmapId, SubmapData> GetAllSubmapDataAfterUpdate() const = 0;

  // Returns the global poses for all submaps.
  virtual MapById<SubmapId, SubmapPose> GetAllSubmapPoses() const = 0;

  // Returns the transform converting data in the local map frame (i.e. the
  // continuous, non-loop-closed frame) into the global map frame (i.e. the
  // discontinuous, loop-closed frame).
  virtual transform::Rigid3d GetLocalToGlobalTransform(
      int trajectory_id) const = 0;

  // Returns the current optimized trajectories.
  virtual MapById<NodeId, TrajectoryNode> GetTrajectoryNodes() const = 0;

  // okagv
  virtual MapById<NodeId, TrajectoryNode> GetTrajectoryNodesAfterUpdate() const = 0;

  // Returns the current optimized trajectory poses.
  virtual MapById<NodeId, TrajectoryNodePose> GetTrajectoryNodePoses()
      const = 0;

  // Returns the states of trajectories.
  virtual std::map<int, TrajectoryState> GetTrajectoryStates() const = 0;

  // okagv
  virtual OKagvOrder GetOKagv_Order() const = 0;
  virtual void SetOKagv_Order(const OKagvOrder& order) = 0;
  virtual bool LocalizeOKagvPoses(const bool use_initial_pose,
                                  const int trajectory_id,
                                  const transform::Rigid3d initial_pose) = 0;

  virtual OKagvFeedback GetOKagv_Feedback() const = 0;
  virtual void SetOKagv_Feedback(const OKagvFeedback& order) = 0;

  // Returns the current optimized landmark poses.
  virtual std::map<std::string, transform::Rigid3d> GetLandmarkPoses()
      const = 0;
  //okagv
  virtual std::map<std::string, transform::Rigid3d> GetLandmarkPosesWithId(int trajectory_id) const = 0;
  //okagv
  virtual std::map<std::string, transform::Rigid3d> GetLandmarkPosesAfterUpdate() const = 0;

  // Sets global pose of landmark 'landmark_id' to given 'global_pose'.
  virtual void SetLandmarkPose(const std::string& landmark_id,
                               const transform::Rigid3d& global_pose,
                               const bool frozen = false) = 0;

  // Deletes a trajectory asynchronously.
  virtual void DeleteTrajectory(int trajectory_id) = 0;

  // Checks if the given trajectory is finished.
  virtual bool IsTrajectoryFinished(int trajectory_id) const = 0;

  // Checks if the given trajectory is frozen.
  virtual bool IsTrajectoryFrozen(int trajectory_id) const = 0;

  // Returns the trajectory data.
  virtual std::map<int, TrajectoryData> GetTrajectoryData() const = 0;
    
  // okagv
  virtual std::map<int, TrajectoryData> GetTrajectoryDataAfterUpdate() const = 0;

  // Returns the collection of constraints.
  virtual std::vector<Constraint> constraints() const = 0;

  // okagv
  virtual std::vector<Constraint> constraintsWithId(int trajectory_id) const = 0;

  // okagv
  virtual std::vector<Constraint> constraintsAfterUpdate() const = 0;

  // Serializes the constraints and trajectories. If
  // 'include_unfinished_submaps' is set to 'true', unfinished submaps, i.e.
  // submaps that have not yet received all rangefinder data insertions, will
  // be included, otherwise not.
  virtual proto::PoseGraph ToProto(bool include_unfinished_submaps) const = 0;

  //okagv
  virtual proto::PoseGraph ToProtoWithId(int trajectory_id, bool include_unfinished_submaps) const = 0;

  //okagv
  virtual proto::PoseGraph ToProtoWithUpdate(bool include_unfinished_submaps) const = 0;

  // Sets the callback function that is invoked whenever the global optimization
  // problem is solved.
  virtual void SetGlobalSlamOptimizationCallback(
      GlobalSlamOptimizationCallback callback) = 0;

  //okagv
  virtual bool IsTrajectoryExist(int trajectory_id) const = 0;

  //okagv
  virtual void GetCovarianceScore(double& score, bool& is_update) = 0;
  //okagv
  virtual void SetTrajectoryState(int trajectory_id, TrajectoryState state) = 0;
  //okagv
  virtual void SetWorkingTrajectoryType(uint8_t type) = 0;
  //okagv
  virtual uint8_t GetWorkingTrajectoryType() = 0;
  //okagv
  virtual void SetCovarianceScore(double score) = 0;
  //okagv
  virtual void SetConstraintBuilderMinScore(double score) = 0;

  
  //okagv
  virtual void StopDrainWorkQueue() = 0;
  //okagv 
  virtual void StartDrainWorkQueue() = 0;
  //okagv
  virtual void SetThreadPoolState(ThreadPoolState type) = 0;
  //okagv
  virtual ThreadPoolState GetThreadPoolState() = 0;

  //okagv
  virtual void SetPoseGraphOption(proto::PoseGraphOptions& option_reset) = 0;
  

};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_POSE_GRAPH_INTERFACE_H_
