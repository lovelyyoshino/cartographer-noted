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

#ifndef CARTOGRAPHER_CLOUD_INTERNAL_CLIENT_POSE_GRAPH_STUB_H_
#define CARTOGRAPHER_CLOUD_INTERNAL_CLIENT_POSE_GRAPH_STUB_H_

#include "cartographer/mapping/pose_graph_interface.h"
#include "grpc++/grpc++.h"

namespace cartographer {
namespace cloud {

class PoseGraphStub : public ::cartographer::mapping::PoseGraphInterface {
 public:
  PoseGraphStub(std::shared_ptr<::grpc::Channel> client_channel,
                const std::string& client_id);

  PoseGraphStub(const PoseGraphStub&) = delete;
  PoseGraphStub& operator=(const PoseGraphStub&) = delete;

  void RunFinalOptimization() override;
  mapping::MapById<mapping::SubmapId, SubmapData> GetAllSubmapData()
      const override;
  mapping::MapById<mapping::SubmapId, SubmapPose> GetAllSubmapPoses()
      const override;
  transform::Rigid3d GetLocalToGlobalTransform(
      int trajectory_id) const override;
  mapping::MapById<mapping::NodeId, mapping::TrajectoryNode>
  GetTrajectoryNodes() const override;
  mapping::MapById<mapping::NodeId, mapping::TrajectoryNodePose>
  GetTrajectoryNodePoses() const override;
  std::map<int, TrajectoryState> GetTrajectoryStates() const override;
  std::map<std::string, transform::Rigid3d> GetLandmarkPoses() const override;
  //okagv
  std::map<std::string, transform::Rigid3d> GetLandmarkPosesWithId(int trajectory_id) const override;
  //okagv
    mapping::MapById<mapping::SubmapId, SubmapData> GetAllSubmapDataAfterUpdate()
      const override;
  //okagv
    mapping::MapById<mapping::NodeId, mapping::TrajectoryNode>
  GetTrajectoryNodesAfterUpdate() const override;
  //okagv
  std::map<std::string, transform::Rigid3d> GetLandmarkPosesAfterUpdate() const override;

  void SetLandmarkPose(const std::string& landmark_id,
                       const transform::Rigid3d& global_pose,
                       const bool frozen = false) override;
  void DeleteTrajectory(int trajectory_id) override;
  bool IsTrajectoryFinished(int trajectory_id) const override;
  bool IsTrajectoryFrozen(int trajectory_id) const override;
  std::map<int, mapping::PoseGraphInterface::TrajectoryData> GetTrajectoryData()
      const override;
  std::vector<Constraint> constraints() const override;
  //okagv
  std::vector<Constraint> constraintsWithId(int trajectory_id) const override;
  //okagv
    std::map<int, mapping::PoseGraphInterface::TrajectoryData> GetTrajectoryDataAfterUpdate()
      const override;
  //okagv
  std::vector<Constraint> constraintsAfterUpdate() const override;

  mapping::proto::PoseGraph ToProto(
      bool include_unfinished_submaps) const override;
  void SetGlobalSlamOptimizationCallback(
      GlobalSlamOptimizationCallback callback) override;

  //okagv
  OKagvOrder GetOKagv_Order() const override;
  void SetOKagv_Order(const OKagvOrder& order) override;
  bool LocalizeOKagvPoses(const bool use_initial_pose,
                          const int trajectory_id,
                          const transform::Rigid3d initial_pose) override;  

  OKagvFeedback GetOKagv_Feedback() const override;
  void SetOKagv_Feedback(const OKagvFeedback& feedback) override;                                         

   //okagv
  bool IsTrajectoryExist(int trajectory_id) const override;

  //okagv
  void GetCovarianceScore(double& score, bool& is_update) override;

  //okagv
  void SetTrajectoryState(int trajectoy_id, TrajectoryState state);

      //okagv
  void SetWorkingTrajectoryType(uint8_t type) override;

  //okagv
  uint8_t GetWorkingTrajectoryType() override;

  //okagv
  void SetCovarianceScore(double score) override;

  //okagv
  void SetConstraintBuilderMinScore(double score) override;

  // okagv
  mapping::proto::PoseGraph ToProtoWithId(
      int trajectory_id, bool include_unfinished_submaps) const override;

  // okagv
  mapping::proto::PoseGraph ToProtoWithUpdate(
      bool include_unfinished_submaps) const override;

  // okagv
  void StopDrainWorkQueue() override;
  // okagv
  void StartDrainWorkQueue() override;
  // okagv
  void SetThreadPoolState(
      mapping::PoseGraphInterface::ThreadPoolState type) override;
  // okagv
  mapping::PoseGraphInterface::ThreadPoolState GetThreadPoolState() override;
  
  //okagv
  void SetPoseGraphOption(mapping::proto::PoseGraphOptions& option_reset) override;

 private:
  std::shared_ptr<::grpc::Channel> client_channel_;
  const std::string client_id_;

  mapping::PoseGraphInterface::OKagvOrder current_okagv_order_ =
          mapping::PoseGraphInterface::OKagvOrder::WAIT;
};

}  // namespace cloud
}  // namespace cartographer

#endif  // CARTOGRAPHER_CLOUD_INTERNAL_CLIENT_POSE_GRAPH_STUB_H_
