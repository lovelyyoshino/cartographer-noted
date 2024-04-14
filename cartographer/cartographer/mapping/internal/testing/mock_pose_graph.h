/*
 * Copyright 2018 The Cartographer Authors
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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_TESTING_MOCK_POSE_GRAPH_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_TESTING_MOCK_POSE_GRAPH_H_

#include "cartographer/mapping/pose_graph_interface.h"
#include "glog/logging.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace mapping {
namespace testing {

class MockPoseGraph : public mapping::PoseGraphInterface {
 public:
  MockPoseGraph() = default;
  ~MockPoseGraph() override = default;

  MOCK_METHOD0(RunFinalOptimization, void());
  MOCK_CONST_METHOD0(GetAllSubmapData,
                     mapping::MapById<mapping::SubmapId, SubmapData>());
  MOCK_CONST_METHOD0(GetAllSubmapPoses,
                     mapping::MapById<mapping::SubmapId, SubmapPose>());
  MOCK_CONST_METHOD1(GetLocalToGlobalTransform, transform::Rigid3d(int));
  MOCK_CONST_METHOD0(
      GetTrajectoryNodes,
      mapping::MapById<mapping::NodeId, mapping::TrajectoryNode>());
  MOCK_CONST_METHOD0(
      GetTrajectoryNodePoses,
      mapping::MapById<mapping::NodeId, mapping::TrajectoryNodePose>());
  MOCK_CONST_METHOD0(
      GetTrajectoryStates,
      std::map<int, mapping::PoseGraphInterface::TrajectoryState>());
  MOCK_CONST_METHOD0(GetLandmarkPoses,
                     std::map<std::string, transform::Rigid3d>());
  MOCK_METHOD3(SetLandmarkPose,
               void(const std::string&, const transform::Rigid3d&, const bool));
  MOCK_METHOD1(DeleteTrajectory, void(int));
  MOCK_CONST_METHOD1(IsTrajectoryFinished, bool(int));
  MOCK_CONST_METHOD1(IsTrajectoryFrozen, bool(int));
  MOCK_CONST_METHOD0(
      GetTrajectoryData,
      std::map<int, mapping::PoseGraphInterface::TrajectoryData>());
  MOCK_CONST_METHOD0(constraints, std::vector<Constraint>());
  MOCK_CONST_METHOD1(ToProto, mapping::proto::PoseGraph(bool));
  MOCK_METHOD1(SetGlobalSlamOptimizationCallback,
               void(GlobalSlamOptimizationCallback callback));

  MOCK_CONST_METHOD0(GetOKagv_Order, mapping::PoseGraphInterface::OKagvOrder());

  MOCK_METHOD1(SetOKagv_Order,
               void(const mapping::PoseGraphInterface::OKagvOrder&));

  MOCK_CONST_METHOD0(GetOKagv_Feedback, mapping::PoseGraphInterface::OKagvFeedback());

  MOCK_METHOD1(SetOKagv_Feedback,
               void(const mapping::PoseGraphInterface::OKagvFeedback&));

  MOCK_CONST_METHOD1(IsTrajectoryExist, bool(int));

  MOCK_METHOD2(GetCovarianceScore, void(double&, bool&));

  MOCK_METHOD2(SetTrajectoryState, void(int, mapping::PoseGraphInterface::TrajectoryState));

  MOCK_METHOD3(LocalizeOKagvPoses, bool(const bool, const int, const transform::Rigid3d));

  MOCK_METHOD1(SetWorkingTrajectoryType, void(uint8_t));

  MOCK_METHOD0(GetWorkingTrajectoryType, uint8_t());

  MOCK_METHOD1(SetCovarianceScore, void(double));
  MOCK_METHOD1(SetConstraintBuilderMinScore, void(double));
  MOCK_CONST_METHOD0(constraintsWithId, std::vector<Constraint>(int));

  MOCK_CONST_METHOD1(GetLandmarkPosesWithId,
                     std::map<std::string, transform::Rigid3d>(int));

  MOCK_CONST_METHOD0(GetAllSubmapDataAfterUpdate,
                     mapping::MapById<mapping::SubmapId, SubmapData>());
  MOCK_CONST_METHOD0(
      GetTrajectoryNodesAfterUpdate,
      mapping::MapById<mapping::NodeId, mapping::TrajectoryNode>());
  MOCK_CONST_METHOD0(
      GetTrajectoryDataAfterUpdate,
      std::map<int, mapping::PoseGraphInterface::TrajectoryData>());
  MOCK_CONST_METHOD0(constraintsAfterUpdate, std::vector<Constraint>());

  MOCK_CONST_METHOD0(GetLandmarkPosesAfterUpdate,
                     std::map<std::string, transform::Rigid3d>());

  MOCK_METHOD0(StopDrainWorkQueue, void());

  MOCK_METHOD0(StartDrainWorkQueue, void());

  MOCK_METHOD1(SetThreadPoolState,
               void(mapping::PoseGraphInterface::ThreadPoolState));

  MOCK_METHOD0(GetThreadPoolState,
               mapping::PoseGraphInterface::ThreadPoolState());

  MOCK_METHOD1(SetPoseGraphOption, void(mapping::proto::PoseGraphOptions&));
};

}  // namespace testing
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_TESTING_MOCK_POSE_GRAPH_H_
