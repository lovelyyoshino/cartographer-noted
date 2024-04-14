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

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_MAP_BUILDER_BRIDGE_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_MAP_BUILDER_BRIDGE_H

#include <memory>
#include <set>
#include <string>
#include <unordered_map>

#include "absl/synchronization/mutex.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/mapping/pose_graph_interface.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer/mapping/trajectory_builder_interface.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/sensor_bridge.h"
#include "cartographer_ros/tf_bridge.h"
#include "cartographer_ros/trajectory_options.h"
#include "cartographer_ros_msgs/SubmapEntry.h"
#include "cartographer_ros_msgs/SubmapList.h"
#include "cartographer_ros_msgs/SubmapQuery.h"
#include "cartographer_ros_msgs/TrajectoryQuery.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/OccupancyGrid.h"

#include "cartographer_ros/map_builder_server_options.h"

// Abseil unfortunately pulls in winnt.h, which #defines DELETE.
// Clean up to unbreak visualization_msgs::Marker::DELETE.
#ifdef DELETE
#undef DELETE
#endif
#include "visualization_msgs/MarkerArray.h"

//okagv
#include "cartographer/mapping/map_builder_interface.h"
//okagv
#include "cartographer_ros_msgs/StartTrajectory.h"
#include "cartographer_ros_msgs/WriteState.h"

namespace cartographer_ros {

using TrajectoryType =
::cartographer::mapping::MapBuilderInterface::TrajectoryType;

using ThreadPoolState =
    ::cartographer::mapping::PoseGraphInterface::ThreadPoolState;

class MapBuilderBridge {
 public:
  struct LocalTrajectoryData {
    // Contains the trajectory data received from local SLAM, after
    // it had processed accumulated 'range_data_in_local' and estimated
    // current 'local_pose' at 'time'.
    struct LocalSlamData {
      ::cartographer::common::Time time;
      ::cartographer::transform::Rigid3d local_pose;
      ::cartographer::sensor::RangeData range_data_in_local;
    };

    std::shared_ptr<const LocalSlamData> local_slam_data;
    cartographer::transform::Rigid3d local_to_map;
    std::unique_ptr<cartographer::transform::Rigid3d> published_to_tracking;
    TrajectoryOptions trajectory_options;
    //okagv

    double covariance_score;
    bool is_update_score;
  };

  MapBuilderBridge(
      const NodeOptions& node_options,
      std::shared_ptr<cartographer::mapping::MapBuilderInterface> map_builder,
      tf2_ros::Buffer* tf_buffer);

  MapBuilderBridge(const MapBuilderBridge&) = delete;
  MapBuilderBridge& operator=(const MapBuilderBridge&) = delete;

  void LoadState(const std::string& state_filename, bool load_frozen_state);
  int AddTrajectory(
      const std::set<
          ::cartographer::mapping::TrajectoryBuilderInterface::SensorId>&
          expected_sensor_ids,
      const TrajectoryOptions& trajectory_options);
  void FinishTrajectory(int trajectory_id);
  void RunFinalOptimization();
  bool SerializeState(const std::string& filename,
                      const bool include_unfinished_submaps);
  //okagv
  bool SerializeStateWithId(const std::string& filename,
                            int &trajectory_id,
                            const bool include_unfinished_submaps);

  void HandleSubmapQuery(
      cartographer_ros_msgs::SubmapQuery::Request& request,
      cartographer_ros_msgs::SubmapQuery::Response& response);
  void HandleTrajectoryQuery(
      cartographer_ros_msgs::TrajectoryQuery::Request& request,
      cartographer_ros_msgs::TrajectoryQuery::Response& response);

  std::map<int /* trajectory_id */,
           ::cartographer::mapping::PoseGraphInterface::TrajectoryState>
  GetTrajectoryStates();
  cartographer_ros_msgs::SubmapList GetSubmapList();
  std::unordered_map<int, LocalTrajectoryData> GetLocalTrajectoryData()
      LOCKS_EXCLUDED(mutex_);
  visualization_msgs::MarkerArray GetTrajectoryNodeList();
  visualization_msgs::MarkerArray GetLandmarkPosesList();
  visualization_msgs::MarkerArray GetConstraintList();

  SensorBridge* sensor_bridge(int trajectory_id);

  //okagv
    int AddTrajectoryWithId(
      const int trajectory_id,
      const std::set<
          ::cartographer::mapping::TrajectoryBuilderInterface::SensorId>&
          expected_sensor_ids,
      const TrajectoryOptions& trajectory_options);

  //okagv
  ::cartographer::mapping::PoseGraphInterface::OKagvOrder
  GetOKagv_Order();
  //okagv
  void SetOKagv_Order(::cartographer::mapping::PoseGraphInterface::OKagvOrder order);
  //okagv
  void SetTrajectoryStates(int trajectory_id, 
                           ::cartographer::mapping::PoseGraphInterface::TrajectoryState state);

  //okagv
  ::cartographer::mapping::PoseGraphInterface::OKagvFeedback GetOKagv_Feedback();
  void SetOKagv_Feedback(::cartographer::mapping::PoseGraphInterface::OKagvFeedback feedback);

  //okagv
  void SetInitialPose(bool &use_initial_pose,
                      int &trajectory_id,
                      ::cartographer::transform::Rigid3d &initial_pose);

    //okagv
  void LoadTrajectory(
      int trajectory_id,
      const std::string& state_filename, 
      ::cartographer::mapping::PoseGraphInterface::TrajectoryState state);
  void DeleteTrajectory(
      int trajectory_id);
  void LocalizeTrajectory(
      int trajectory_id,
      bool use_initial_pose,
      const geometry_msgs::Pose initial_pose);

  //okagv
  void SetTrajectoryTypeWithId(TrajectoryType type, int id);

  //okagv
  TrajectoryType GetTrajectoryTypeWithId(int id);

  //okagv
  std::shared_ptr<cartographer::mapping::MapBuilderInterface> map_builder(){ return map_builder_;}

  //okagv
  void GetOkagvOrderStartTrajectoryRequest(::cartographer_ros_msgs::StartTrajectory::Request& request);
  //okagv
  void GetOkagvOrderSaveTrajectoryRequest(::cartographer_ros_msgs::WriteState::Request& request);

  //okagv
  void RegisterClientIdForTrajectory(int trajectory_id, std::string trajectory_name);

  //okagv
  ThreadPoolState GetThreadPoolState();

  //okagv
  void SetMapBuilderOptions(cartographer::cloud::proto::MapBuilderServerOptions& option);

 private:
  void OnLocalSlamResult(const int trajectory_id,
                         const ::cartographer::common::Time time,
                         const ::cartographer::transform::Rigid3d local_pose,
                         ::cartographer::sensor::RangeData range_data_in_local)
      LOCKS_EXCLUDED(mutex_);

  absl::Mutex mutex_;
  const NodeOptions node_options_;
  std::unordered_map<int,
                     std::shared_ptr<const LocalTrajectoryData::LocalSlamData>>
      local_slam_data_ GUARDED_BY(mutex_);
  std::shared_ptr<cartographer::mapping::MapBuilderInterface> map_builder_;
  tf2_ros::Buffer* const tf_buffer_;

  std::unordered_map<std::string /* landmark ID */, int> landmark_to_index_;

  // These are keyed with 'trajectory_id'.
  std::unordered_map<int, TrajectoryOptions> trajectory_options_;
  std::unordered_map<int, std::unique_ptr<SensorBridge>> sensor_bridges_;
  std::unordered_map<int, size_t> trajectory_to_highest_marker_id_;
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_MAP_BUILDER_BRIDGE_H
