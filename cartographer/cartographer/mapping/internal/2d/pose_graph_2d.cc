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

#include "cartographer/mapping/internal/2d/pose_graph_2d.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <sstream>
#include <string>

#include "Eigen/Eigenvalues"
#include "absl/memory/memory.h"
#include "cartographer/common/math.h"
#include "cartographer/mapping/internal/2d/overlapping_submaps_trimmer_2d.h"
#include "cartographer/mapping/proto/pose_graph/constraint_builder_options.pb.h"
#include "cartographer/sensor/compressed_point_cloud.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"




using TrajectoryType = 
      cartographer::mapping::MapBuilderInterface::TrajectoryType;

namespace cartographer {
namespace mapping {

static auto* kWorkQueueDelayMetric = metrics::Gauge::Null();
static auto* kWorkQueueSizeMetric = metrics::Gauge::Null();
static auto* kConstraintsSameTrajectoryMetric = metrics::Gauge::Null();
static auto* kConstraintsDifferentTrajectoryMetric = metrics::Gauge::Null();
static auto* kActiveSubmapsMetric = metrics::Gauge::Null();
static auto* kFrozenSubmapsMetric = metrics::Gauge::Null();
static auto* kDeletedSubmapsMetric = metrics::Gauge::Null();

PoseGraph2D::PoseGraph2D(
    proto::PoseGraphOptions& options,
    std::unique_ptr<optimization::OptimizationProblem2D> optimization_problem,
    common::ThreadPool* thread_pool)
    : options_(options),
      optimization_problem_(std::move(optimization_problem)),
      constraint_builder_(*options_.mutable_constraint_builder_options(), thread_pool), //okagv use mutable_constraint_builder_options
      thread_pool_(thread_pool) {
  if (options.has_overlapping_submaps_trimmer_2d()) {
    LOG(INFO) << "okagv has_overlapping_submaps_trimmer_2d";
    const auto& trimmer_options = options.overlapping_submaps_trimmer_2d();
    AddTrimmer(absl::make_unique<OverlappingSubmapsTrimmer2D>(
        trimmer_options.fresh_submaps_count(),
        trimmer_options.min_covered_area(),
        trimmer_options.min_added_submaps_count()));
  }
  //okagv
  //trimmers_thread_ = new std::thread(&PoseGraph2D::TrimSubmapTimer,this);
}

PoseGraph2D::~PoseGraph2D() {
  WaitForAllComputations();
  absl::MutexLock locker(&work_queue_mutex_);
  CHECK(work_queue_ == nullptr);
}

std::vector<SubmapId> PoseGraph2D::InitializeGlobalSubmapPoses(
    const int trajectory_id, const common::Time time,
    const std::vector<std::shared_ptr<const Submap2D>>& insertion_submaps) {
  CHECK(!insertion_submaps.empty());
  const auto& submap_data = optimization_problem_->submap_data();

  if (insertion_submaps.size() == 1) {

    // If we don't already have an entry for the first submap, add one.
    if (submap_data.SizeOfTrajectoryOrZero(trajectory_id) == 0) {
      if (data_.initial_trajectory_poses.count(trajectory_id) > 0) {

        data_.trajectory_connectivity_state.Connect(
            trajectory_id,
            data_.initial_trajectory_poses.at(trajectory_id).to_trajectory_id,
            time);
      }
      //LOG(INFO) << "okagv InitializeGlobalSubmapPoses 1"; 
      optimization_problem_->AddSubmap(
          trajectory_id, transform::Project2D(
                             ComputeLocalToGlobalTransform(
                                 data_.global_submap_poses_2d, trajectory_id) *
                             insertion_submaps[0]->local_pose()));
    }
    CHECK_EQ(1, submap_data.SizeOfTrajectoryOrZero(trajectory_id));
    const SubmapId submap_id{trajectory_id, 0};
    //LOG(INFO) << "okagv data_.submap_data 1";
    CHECK(data_.submap_data.at(submap_id).submap == insertion_submaps.front());
    return {submap_id};
  }
  CHECK_EQ(2, insertion_submaps.size());
  const auto end_it = submap_data.EndOfTrajectory(trajectory_id);
  CHECK(submap_data.BeginOfTrajectory(trajectory_id) != end_it);
  const SubmapId last_submap_id = std::prev(end_it)->id;
  //LOG(INFO) << "okagv data_.submap_data 2";
  if (data_.submap_data.at(last_submap_id).submap ==
      insertion_submaps.front()) {
    // In this case, 'last_submap_id' is the ID of
    // 'insertions_submaps.front()' and 'insertions_submaps.back()' is new.
    const auto& first_submap_pose = submap_data.at(last_submap_id).global_pose;
    optimization_problem_->AddSubmap(
        trajectory_id,
        first_submap_pose *
            constraints::ComputeSubmapPose(*insertion_submaps[0]).inverse() *
            constraints::ComputeSubmapPose(*insertion_submaps[1]));
    return {last_submap_id,
            SubmapId{trajectory_id, last_submap_id.submap_index + 1}};
  }
  //LOG(INFO) << "okagv data_.submap_data 3";
  CHECK(data_.submap_data.at(last_submap_id).submap ==
        insertion_submaps.back());
  const SubmapId front_submap_id{trajectory_id,
                                 last_submap_id.submap_index - 1};
  //LOG(INFO) << "okagv data_.submap_data 4";
  CHECK(data_.submap_data.at(front_submap_id).submap ==
        insertion_submaps.front());
  return {front_submap_id, last_submap_id};
}

NodeId PoseGraph2D::AppendNode(
    std::shared_ptr<const TrajectoryNode::Data> constant_data,
    const int trajectory_id,
    const std::vector<std::shared_ptr<const Submap2D>>& insertion_submaps,
    const transform::Rigid3d& optimized_pose) {
  absl::MutexLock locker(&mutex_);
  AddTrajectoryIfNeeded(trajectory_id);
  if (!CanAddWorkItemModifying(trajectory_id)) {
    LOG(WARNING) << "AddNode was called for finished or deleted trajectory.";
  }
  const NodeId node_id = data_.trajectory_nodes.Append(
      trajectory_id, TrajectoryNode{constant_data, optimized_pose});
  ++data_.num_trajectory_nodes;

  // Test if the 'insertion_submap.back()' is one we never saw before.
  if (data_.submap_data.SizeOfTrajectoryOrZero(trajectory_id) == 0 ||
      std::prev(data_.submap_data.EndOfTrajectory(trajectory_id))
              ->data.submap != insertion_submaps.back()) {
    // We grow 'data_.submap_data' as needed. This code assumes that the first
    // time we see a new submap is as 'insertion_submaps.back()'.
    const SubmapId submap_id =
        data_.submap_data.Append(trajectory_id, InternalSubmapData());
    //LOG(INFO) << "okagv data_.submap_data 5";    
    data_.submap_data.at(submap_id).submap = insertion_submaps.back();
    LOG(INFO) << "Inserted submap " << submap_id << ".";
    kActiveSubmapsMetric->Increment();
  }

  //okagv
  //is_node_exist = true;
  return node_id;
}

NodeId PoseGraph2D::AddNode(
    std::shared_ptr<const TrajectoryNode::Data> constant_data,
    const int trajectory_id,
    const std::vector<std::shared_ptr<const Submap2D>>& insertion_submaps) {

  const transform::Rigid3d optimized_pose(
      GetLocalToGlobalTransform(trajectory_id) * constant_data->local_pose); 

  const NodeId node_id = AppendNode(constant_data, trajectory_id,
                                    insertion_submaps, optimized_pose);   

  //LOG(INFO) << "okagv AddNode " << constant_data->time;                
  // We have to check this here, because it might have changed by the time we
  // execute the lambda.
  
  //okagv
  //LOG(INFO) << "okagv AddWorkItem AddNode";
  const bool newly_finished_submap =
      insertion_submaps.front()->insertion_finished();
  AddWorkItem([=]() LOCKS_EXCLUDED(mutex_) {
    return ComputeConstraintsForNode(node_id, insertion_submaps,
                                     newly_finished_submap);
  });
  
  return node_id;
}

void PoseGraph2D::AddWorkItem(
    const std::function<WorkItem::Result()>& work_item) {

    //okagv
  //if(is_start_clear_work_queue == true) return;

  absl::MutexLock locker(&work_queue_mutex_);
  if (work_queue_ == nullptr) {
    work_queue_ = absl::make_unique<WorkQueue>();
    auto task = absl::make_unique<common::Task>();
    task->SetWorkItem([this]() { DrainWorkQueue(); });
    thread_pool_->Schedule(std::move(task));
  }
  const auto now = std::chrono::steady_clock::now();
  work_queue_->push_back({now, work_item});
  kWorkQueueSizeMetric->Set(work_queue_->size());
  kWorkQueueDelayMetric->Set(
      std::chrono::duration_cast<std::chrono::duration<double>>(
          now - work_queue_->front().time)
          .count());

  //LOG(INFO) << "work_queue_ size is " << work_queue_->size();
}

void PoseGraph2D::AddTrajectoryIfNeeded(const int trajectory_id) {
  data_.trajectories_state[trajectory_id];
  CHECK(data_.trajectories_state.at(trajectory_id).state !=
        TrajectoryState::FINISHED);
  //CHECK(data_.trajectories_state.at(trajectory_id).state !=
  //      TrajectoryState::DELETED);
  CHECK(data_.trajectories_state.at(trajectory_id).deletion_state ==
        InternalTrajectoryState::DeletionState::NORMAL);
  data_.trajectory_connectivity_state.Add(trajectory_id);
  // Make sure we have a sampler for this trajectory.
  if (!global_localization_samplers_[trajectory_id]) {
    global_localization_samplers_[trajectory_id] =
        absl::make_unique<common::FixedRatioSampler>(
            options_.global_sampling_ratio());
  }
}

void PoseGraph2D::AddImuData(const int trajectory_id,
                             const sensor::ImuData& imu_data) {
  //LOG(INFO) << "okagv AddWorkItem AddImuData";
  AddWorkItem([=]() LOCKS_EXCLUDED(mutex_) {
    absl::MutexLock locker(&mutex_);
    if (CanAddWorkItemModifying(trajectory_id)) {
      optimization_problem_->AddImuData(trajectory_id, imu_data);
    }
    return WorkItem::Result::kDoNotRunOptimization;
  });
}

void PoseGraph2D::AddOdometryData(const int trajectory_id,
                                  const sensor::OdometryData& odometry_data) {
  //LOG(INFO) << "okagv AddWorkItem AddOdometryData";
  AddWorkItem([=]() LOCKS_EXCLUDED(mutex_) {
    absl::MutexLock locker(&mutex_);
    if (CanAddWorkItemModifying(trajectory_id)) {
      optimization_problem_->AddOdometryData(trajectory_id, odometry_data);
    }
    return WorkItem::Result::kDoNotRunOptimization;
  });
}

void PoseGraph2D::AddFixedFramePoseData(
    const int trajectory_id,
    const sensor::FixedFramePoseData& fixed_frame_pose_data) {
  //LOG(INFO) << "okagv AddWorkItem AddFixedFramePoseData";
  AddWorkItem([=]() LOCKS_EXCLUDED(mutex_) {
    absl::MutexLock locker(&mutex_);
    if (CanAddWorkItemModifying(trajectory_id)) {
      optimization_problem_->AddFixedFramePoseData(trajectory_id,
                                                   fixed_frame_pose_data);
    }
    return WorkItem::Result::kDoNotRunOptimization;
  });
}

void PoseGraph2D::AddLandmarkData(int trajectory_id,
                                  const sensor::LandmarkData& landmark_data) {
    //LOG(INFO) << "okagv AddWorkItem AddLandmarkData";
  AddWorkItem([=]() LOCKS_EXCLUDED(mutex_) {
    absl::MutexLock locker(&mutex_);
    if (CanAddWorkItemModifying(trajectory_id)) {
      /*
      for (const auto& observation : landmark_data.landmark_observations) {
        data_.landmark_nodes[observation.id].landmark_observations.emplace_back(
            PoseGraphInterface::LandmarkNode::LandmarkObservation{
                trajectory_id, landmark_data.time,
                observation.landmark_to_tracking_transform,
                observation.translation_weight, observation.rotation_weight});
      }
      */
      if (landmark_data.landmark_observations[0].type == "qrcode") {
        // for qrcode
        for (const auto& observation : landmark_data.landmark_observations) {
          // initial landmark_nodes
          // const transform::Rigid3d global_pose =
          // observation.landmark_to_map_transform;

          data_.landmark_nodes[observation.id]
              .landmark_observations.emplace_back(
                  PoseGraphInterface::LandmarkNode::LandmarkObservation{
                      trajectory_id, landmark_data.time,
                      observation.landmark_to_tracking_transform,
                      observation.translation_weight,
                      observation.rotation_weight});

          data_.landmark_nodes[observation.id].confidence_score = 0.65;
          // data_.landmark_nodes[observation.id].global_landmark_pose =
          // global_pose;
        }
      } else if (landmark_data.landmark_observations[0].type == "reflector") {
        if (current_trajectory_type_ == TrajectoryType::NAVIGATION &&
            last_decrease_covariance_score_ == 0.0)
          return WorkItem::Result::kDoNotRunOptimization;
        // above if means: do not use reflector to localize unless already know
        // the robot global position
        for (const auto& observation : landmark_data.landmark_observations) {
          if (observation.id == "unknown") {
            // initial landmark_nodes
            const transform::Rigid3d global_pose =
                observation.landmark_to_map_transform;

            if (data_.landmark_nodes.size() == 0) {
              int landmark_count = data_.landmark_nodes.size();
              std::string landmark_name = std::to_string(landmark_count);

              data_.landmark_nodes[landmark_name]
                  .landmark_observations.emplace_back(
                      PoseGraphInterface::LandmarkNode::LandmarkObservation{
                          trajectory_id, landmark_data.time,
                          observation.landmark_to_tracking_transform,
                          observation.translation_weight,
                          observation.rotation_weight});

              data_.landmark_nodes[landmark_name].confidence_score = 0.0;
              data_.landmark_nodes[landmark_name].global_landmark_pose =
                  global_pose;
            }

            // search or insert new landmark
            bool is_landmark_exist = false;
            for (const auto& landmark : data_.landmark_nodes) {
              // Landmark without value has not been optimized yet.
              if (!landmark.second.global_landmark_pose.has_value()) continue;

              double distance = (global_pose.inverse() *
                                 landmark.second.global_landmark_pose.value())
                                    .translation()
                                    .norm();
              /*
                            int last_landmark_count =
              data_.landmark_nodes[landmark.first]
                                            .landmark_observations.size();
              common::Time last_time =
                  data_.landmark_nodes[landmark.first]
                      .landmark_observations[last_landmark_count - 1]
                      .time;

                                  if (last_time - landmark_data.time <
              common::FromSeconds(1.0)) {
                  // data_.landmark_nodes[landmark.first].global_landmark_pose =
                  // global_pose;
                  data_.landmark_nodes[landmark.first].confidence_score += 0.1;
                  is_landmark_exist = true;
                  LOG(INFO)
                      << "okagv landmark confidence_score "
                      << data_.landmark_nodes[landmark.first].confidence_score;
                  break;
                } else {
                  data_.landmark_nodes[landmark.first].confidence_score -= 0.1;
                }
              */

              if (distance < 0.3) {
                // data_.landmark_nodes[landmark.first].global_landmark_pose =
                // global_pose;
                data_.landmark_nodes[landmark.first].confidence_score += 0.1;
                is_landmark_exist = true;

                data_.landmark_nodes[landmark.first]
                    .landmark_observations.emplace_back(
                        PoseGraphInterface::LandmarkNode::LandmarkObservation{
                            trajectory_id, landmark_data.time,
                            observation.landmark_to_tracking_transform,
                            observation.translation_weight,
                            observation.rotation_weight});

                //LOG(INFO)
                //    << "okagv landmark confidence_score "
                //    << data_.landmark_nodes[landmark.first].confidence_score;
              } else {
              }
            }

            if (is_landmark_exist == false) {
              int landmark_count = data_.landmark_nodes.size();
              std::string landmark_name = std::to_string(landmark_count);

              data_.landmark_nodes[landmark_name]
                  .landmark_observations.emplace_back(
                      PoseGraphInterface::LandmarkNode::LandmarkObservation{
                          trajectory_id, landmark_data.time,
                          observation.landmark_to_tracking_transform,
                          observation.translation_weight,
                          observation.rotation_weight});

              data_.landmark_nodes[landmark_name].confidence_score = 0.0;
              data_.landmark_nodes[landmark_name].global_landmark_pose =
                  global_pose;
            }

          } else {  // load data from the file
            data_.landmark_nodes[observation.id]
                .landmark_observations.emplace_back(
                    PoseGraphInterface::LandmarkNode::LandmarkObservation{
                        trajectory_id, landmark_data.time,
                        observation.landmark_to_tracking_transform,
                        observation.translation_weight,
                        observation.rotation_weight});

            data_.landmark_nodes[observation.id].global_landmark_pose =
                landmark_data.global_landmark_pose;
          }
        }
      } else if (landmark_data.landmark_observations[0].type ==
                 "reflector_combined") {
        for (const auto& observation : landmark_data.landmark_observations) {
          data_.landmark_nodes[observation.id]
              .landmark_observations.emplace_back(
                  PoseGraphInterface::LandmarkNode::LandmarkObservation{
                      trajectory_id, landmark_data.time,
                      observation.landmark_to_tracking_transform,
                      observation.translation_weight,
                      observation.rotation_weight});

          data_.landmark_nodes[observation.id].confidence_score = 0.65;
          // data_.landmark_nodes[observation.id].global_landmark_pose =
          // global_pose;
        }
      }
      else if(landmark_data.landmark_observations[0].type ==
                 "apriltag")
      {
        // for apriltag
        for (const auto& observation : landmark_data.landmark_observations) {
          // initial landmark_nodes
          // const transform::Rigid3d global_pose =
          // observation.landmark_to_map_transform;

          data_.landmark_nodes[observation.id]
              .landmark_observations.emplace_back(
                  PoseGraphInterface::LandmarkNode::LandmarkObservation{
                      trajectory_id, landmark_data.time,
                      observation.landmark_to_tracking_transform,
                      observation.translation_weight,
                      observation.rotation_weight});

          data_.landmark_nodes[observation.id].confidence_score = 0.65;
          // data_.landmark_nodes[observation.id].global_landmark_pose =
          // global_pose;
        }
      }
    }
    return WorkItem::Result::kDoNotRunOptimization;
  });
}

void PoseGraph2D::ComputeConstraint(const NodeId& node_id,
                                    const SubmapId& submap_id) {
  bool maybe_add_local_constraint = false;
  bool maybe_add_global_constraint = false;
  bool maybe_add_navigation_constraint = false;
  const TrajectoryNode::Data* constant_data;
  const Submap2D* submap;
  {
    absl::MutexLock locker(&mutex_);
    //LOG(INFO) << "okagv data_.submap_data 6";
    CHECK(data_.submap_data.at(submap_id).state == SubmapState::kFinished);
    //LOG(INFO) << "okagv data_.submap_data 7";
    if (!data_.submap_data.at(submap_id).submap->insertion_finished()) {
      // Uplink server only receives grids when they are finished, so skip
      // constraint search before that.
      return;
    }

    const common::Time node_time = GetLatestNodeTime(node_id, submap_id);
    const common::Time last_connection_time =
        data_.trajectory_connectivity_state.LastConnectionTime(
            node_id.trajectory_id, submap_id.trajectory_id);
    if (node_id.trajectory_id == submap_id.trajectory_id ||
        node_time <
            last_connection_time +
                common::FromSeconds(
                    options_.global_constraint_search_after_n_seconds())) {
      // If the node and the submap belong to the same trajectory or if there
      // has been a recent global constraint that ties that node's trajectory to
      // the submap's trajectory, it suffices to do a match constrained to a
      // local search window.
      //LOG(INFO) << "okagv maybe_add_local_constraint";
      maybe_add_local_constraint = true;
    } else if (global_localization_samplers_[node_id.trajectory_id]->Pulse()) {
      maybe_add_global_constraint = false; //true
    }
    else if(current_trajectory_type_ == TrajectoryType::NAVIGATION)
    {
      maybe_add_navigation_constraint = true;
    }
    
    constant_data = data_.trajectory_nodes.at(node_id).constant_data.get();
    //LOG(INFO) << "okagv data_.submap_data 8";
    submap = static_cast<const Submap2D*>(
        data_.submap_data.at(submap_id).submap.get());
  }

  if (maybe_add_local_constraint) {
    const transform::Rigid2d initial_relative_pose =
        optimization_problem_->submap_data()
            .at(submap_id)
            .global_pose.inverse() *
        optimization_problem_->node_data().at(node_id).global_pose_2d;
    constraint_builder_.MaybeAddConstraint(
        submap_id, submap, node_id, constant_data, initial_relative_pose);

  } else if (maybe_add_global_constraint) {
    constraint_builder_.MaybeAddGlobalConstraint(submap_id, submap, node_id,
                                                 constant_data);
  } else if (maybe_add_navigation_constraint) {
    const transform::Rigid2d initial_relative_pose =
        optimization_problem_->submap_data()
            .at(submap_id)
            .global_pose.inverse() *
        optimization_problem_->node_data().at(node_id).global_pose_2d;
    constraint_builder_.MaybeAddNavigationConstraint(
        submap_id, submap, node_id, constant_data, initial_relative_pose);
  }
}

    //okagv
void PoseGraph2D::ComputeConstraintForRelocalization(const NodeId& node_id, 
                                            const SubmapId& submap_id,
                                            const transform::Rigid2d initial_relative_pose){
  const TrajectoryNode::Data* constant_data;
  const Submap2D* submap;

  {
    absl::MutexLock locker(&mutex_);
    
    constant_data = data_.trajectory_nodes.at(node_id).constant_data.get();
    //LOG(INFO) << "okagv data_.submap_data 9";
    submap = static_cast<const Submap2D*>(
        data_.submap_data.at(submap_id).submap.get());
  }

  if(add_relocalization_constraint){
    
  //   const transform::Rigid2d initial_relative_pose 
  //                  = transform::Project2D(initial_pose);
        
    constraint_builder_.MaybeAddRelocalizationConstraint(
        submap_id, submap, node_id, constant_data, initial_relative_pose);
  }
  
}

WorkItem::Result PoseGraph2D::ComputeConstraintsForNode(
    const NodeId& node_id,
    std::vector<std::shared_ptr<const Submap2D>> insertion_submaps,
    const bool newly_finished_submap) {
  std::vector<SubmapId> submap_ids;
  std::vector<SubmapId> finished_submap_ids;
  std::set<NodeId> newly_finished_submap_node_ids;
  {
    absl::MutexLock locker(&mutex_);

    //okagv
    /*
    if(data_.submap_data.SizeOfTrajectoryOrZero(current_localization_trajectory_id_) == 0)
    {
      LOG(WARNING) << "OLD data quit";
      return WorkItem::Result::kDoNotRunOptimization;
    }
    */

   //LOG(INFO) << "okagv ComputeConstraintsForNode";

    const auto& constant_data =
        data_.trajectory_nodes.at(node_id).constant_data;
    submap_ids = InitializeGlobalSubmapPoses(
        node_id.trajectory_id, constant_data->time, insertion_submaps);
    CHECK_EQ(submap_ids.size(), insertion_submaps.size());
    const SubmapId matching_id = submap_ids.front();
    const transform::Rigid2d local_pose_2d =
        transform::Project2D(constant_data->local_pose *
                             transform::Rigid3d::Rotation(
                                 constant_data->gravity_alignment.inverse()));
    const transform::Rigid2d global_pose_2d =
        optimization_problem_->submap_data().at(matching_id).global_pose *
        constraints::ComputeSubmapPose(*insertion_submaps.front()).inverse() *
        local_pose_2d;
    optimization_problem_->AddTrajectoryNode(
        matching_id.trajectory_id,
        optimization::NodeSpec2D{constant_data->time, local_pose_2d,
                                 global_pose_2d,
                                 constant_data->gravity_alignment});
    for (size_t i = 0; i < insertion_submaps.size(); ++i) {
      const SubmapId submap_id = submap_ids[i];
      // Even if this was the last node added to 'submap_id', the submap will
      // only be marked as finished in 'data_.submap_data' further below.
      //LOG(INFO) << "okagv data_.submap_data 10";

      //cartographer author
      //CHECK(data_.submap_data.at(submap_id).state ==
      //      SubmapState::kNoConstraintSearch);

      //okagv
      if (data_.submap_data.at(submap_id).state !=
          SubmapState::kNoConstraintSearch) { 

        constraint_builder_.NotifyEndOfNode();
        return WorkItem::Result::kDoNotRunOptimization;
      }

      //LOG(INFO) << "okagv data_.submap_data 11";
      data_.submap_data.at(submap_id).node_ids.emplace(node_id);
      const transform::Rigid2d constraint_transform =
          constraints::ComputeSubmapPose(*insertion_submaps[i]).inverse() *
          local_pose_2d;
      data_.constraints.push_back(
          Constraint{submap_id,
                     node_id,
                     {transform::Embed3D(constraint_transform),
                      options_.matcher_translation_weight(),
                      options_.matcher_rotation_weight()},
                     Constraint::INTRA_SUBMAP});
      // LOG(INFO) << "okagv add INTRA_SUBMAP submap_id " << submap_id << "
      // node_id " << node_id;
    }

    // TODO(gaschler): Consider not searching for constraints against
    // trajectories scheduled for deletion.
    // TODO(danielsievers): Add a member variable and avoid having to copy
    // them out here.
    for (const auto& submap_id_data : data_.submap_data) {
      if (submap_id_data.data.state == SubmapState::kFinished) {
        CHECK_EQ(submap_id_data.data.node_ids.count(node_id), 0);
        finished_submap_ids.emplace_back(submap_id_data.id);
      }
    }
    if (newly_finished_submap) {
      const SubmapId newly_finished_submap_id = submap_ids.front();
      //LOG(INFO) << "okagv data_.submap_data 12";
      InternalSubmapData& finished_submap_data =
          data_.submap_data.at(newly_finished_submap_id);
      CHECK(finished_submap_data.state == SubmapState::kNoConstraintSearch);
      finished_submap_data.state = SubmapState::kFinished;
      newly_finished_submap_node_ids = finished_submap_data.node_ids;
    }
  }

  // okagv, compute constraint when robot move certain distance
  if (!last_node_global_pose.IsValid()) {
    // okagv
    last_node_global_pose = data_.trajectory_nodes.at(node_id).global_pose;
    last_time = data_.trajectory_nodes.at(node_id).constant_data->time; 

    constraint_builder_.NotifyEndOfNode();
    return WorkItem::Result::kDoNotRunOptimization;
  }

  if (current_trajectory_type_ == TrajectoryType::SLAM) {
    // std::priority_queue<submap_sort> submap_sort_1;
    for (const auto& submap_id : finished_submap_ids) {
      ComputeConstraint(node_id, submap_id);

      // okagv,decreace the back-end computation.
      /*
      const transform::Rigid2d initial_relative_pose =
          optimization_problem_->submap_data()
              .at(submap_id)
              .global_pose.inverse() *
          optimization_problem_->node_data().at(node_id).global_pose_2d;

      if (initial_relative_pose.translation().norm() < 6.0) {
        submap_sort_1.emplace(initial_relative_pose.translation().norm(),
                              submap_id.trajectory_id, submap_id.submap_index);
      }
      */
      // LOG(INFO) << "okagv current node_id: " << node_id.node_index << "
      // finished_submap_ids: " << submap_id.submap_index;
    }

    // only use the nearist 1 or 2 or 3 constraint
    /*
    for (int i = 0; i < 2; i++) {
      if (submap_sort_1.empty()) break;
      SubmapId id(submap_sort_1.top().trajectory_id, submap_sort_1.top().index);
      ComputeConstraint(node_id, id);
      // LOG(INFO) << "okagv submap_sort_1 : distance " <<
      submap_sort_1.top().distance;
      submap_sort_1.pop();
    }
    */

    if (newly_finished_submap) {
      const SubmapId newly_finished_submap_id = submap_ids.front();
      // We have a new completed submap, so we look into adding constraints
      // for old nodes.

      // LOG(INFO) << "okagv newly_finished_submap happen";
      // std::priority_queue<submap_sort> submap_sort_2;
      for (const auto& node_id_data : optimization_problem_->node_data()) {
        const NodeId& node_id = node_id_data.id;

        // ComputeConstraint(node_id, newly_finished_submap_id);
        if (newly_finished_submap_node_ids.count(node_id) == 0) {
          // okagv
          /*
          const transform::Rigid2d initial_relative_pose =
              optimization_problem_->submap_data()
                  .at(newly_finished_submap_id)
                  .global_pose.inverse() *
              optimization_problem_->node_data().at(node_id).global_pose_2d;

          if (initial_relative_pose.translation().norm() < 6.0) {
            submap_sort_2.emplace(initial_relative_pose.translation().norm(),
                                  node_id.trajectory_id, node_id.node_index);
          }
          */

          ComputeConstraint(node_id, newly_finished_submap_id);

          // LOG(INFO) << "okagv old node_id: " << node_id.node_index
          //           << "newly_finished_submap_id: "
          //           << newly_finished_submap_id.submap_index;
          //}
        }
      }

      // only use the nearist 1 or 2 or 3 constraint
      /*
      for (int i = 0; i < 2; i++) {
        if (submap_sort_2.empty()) break;
        NodeId id(submap_sort_2.top().trajectory_id, submap_sort_2.top().index);
        ComputeConstraint(id, newly_finished_submap_id);
        // LOG(INFO) << "okagv submap_sort_2 : distance " <<
        submap_sort_2.top().distance;
        submap_sort_2.pop();
      }
      */
    }
  } else if (current_trajectory_type_ == TrajectoryType::NAVIGATION) {
    // LOG(INFO) << "okagv current type is NAVIGATION";

    double distance =
        (data_.trajectory_nodes.at(node_id).global_pose.translation() -
         last_node_global_pose.translation())
            .norm();

    common::Time current_time =
        data_.trajectory_nodes.at(node_id).constant_data->time;

    if (distance > options_.optimize_every_meter() ||
        (current_time >
         last_time + common::FromSeconds(
                         options_.local_constraint_search_after_n_seconds()))) {
      for (const auto& submap_id : finished_submap_ids) {
        ComputeConstraint(node_id, submap_id);
      }
    } else {
      constraint_builder_.NotifyEndOfNode();
      return WorkItem::Result::kDoNotRunOptimization;
    }

    if (distance > options_.optimize_every_meter()) {
      last_node_global_pose = data_.trajectory_nodes.at(node_id).global_pose;
    }

    if (current_time >
        last_time + common::FromSeconds(
                        options_.local_constraint_search_after_n_seconds())) {
      last_time = data_.trajectory_nodes.at(node_id).constant_data->time;
    }

  } else if (current_trajectory_type_ == TrajectoryType::RELOCALIZAION) {
    if (covariance_score_ > 0.0) {
      current_trajectory_type_ = TrajectoryType::NAVIGATION;
    }
  }

  current_node_global_pose = data_.trajectory_nodes.at(node_id).global_pose;

  // okagv, if compute constraint , need zero it.
  // num_nodes_since_last_loop_closure_ = 0;
  //
  constraint_builder_.NotifyEndOfNode();
  absl::MutexLock locker(&mutex_);
  if (current_trajectory_type_ == TrajectoryType::SLAM) {
    ++num_nodes_since_last_loop_closure_;
    if (options_.optimize_every_n_nodes() > 0 &&
        num_nodes_since_last_loop_closure_ >
            options_.optimize_every_n_nodes()) {
      return WorkItem::Result::kRunOptimization;
    }
    return WorkItem::Result::kDoNotRunOptimization;
  } else if (current_trajectory_type_ == TrajectoryType::NAVIGATION) {
    return WorkItem::Result::kRunOptimization;
  } else {
    return WorkItem::Result::kDoNotRunOptimization;
  }

  /*
  absl::MutexLock locker(&mutex_);
  ++num_nodes_since_last_loop_closure_;

  if (options_.optimize_every_n_nodes() > 0 &&
      num_nodes_since_last_loop_closure_ > options_.optimize_every_n_nodes()) {
    return WorkItem::Result::kRunOptimization;
  }
  return WorkItem::Result::kDoNotRunOptimization;
  */
}

common::Time PoseGraph2D::GetLatestNodeTime(const NodeId& node_id,
                                            const SubmapId& submap_id) const {
  common::Time time = data_.trajectory_nodes.at(node_id).constant_data->time;
  //LOG(INFO) << "okagv data_.submap_data 13";
  const InternalSubmapData& submap_data = data_.submap_data.at(submap_id);
  if (!submap_data.node_ids.empty()) {
    //LOG(INFO) << "okagv data_.submap_data 14";
    const NodeId last_submap_node_id =
        *data_.submap_data.at(submap_id).node_ids.rbegin();
    time = std::max(
        time,
        data_.trajectory_nodes.at(last_submap_node_id).constant_data->time);
  }
  return time;
}

void PoseGraph2D::UpdateTrajectoryConnectivity(const Constraint& constraint) {
  CHECK_EQ(constraint.tag, Constraint::INTER_SUBMAP);
  const common::Time time =
      GetLatestNodeTime(constraint.node_id, constraint.submap_id);
  data_.trajectory_connectivity_state.Connect(
      constraint.node_id.trajectory_id, constraint.submap_id.trajectory_id,
      time);
}

void PoseGraph2D::DeleteTrajectoriesIfNeeded() {

  TrimmingHandle trimming_handle(this);

  for (auto& it : data_.trajectories_state) {
    if (it.second.deletion_state ==
        InternalTrajectoryState::DeletionState::WAIT_FOR_DELETION) {
      // TODO(gaschler): Consider directly deleting from data_, which may be
      // more complete.
      auto submap_ids = trimming_handle.GetSubmapIds(it.first);
      for (auto& submap_id : submap_ids) {
        //LOG(INFO) << "okagv TrimSubmap " << submap_id;
        trimming_handle.TrimSubmap(submap_id);

        //okagv
        //data_.global_submap_poses_2d.Trim(submap_id);
      }

      //okagv
      //LOG(INFO) << "okagv data_.trajectories_state.erase " << it.first;
      data_.trajectories_state.erase(it.first);
      data_.initial_trajectory_poses.erase(it.first);
      trimmers_.clear();

      //okagv
      /*
      auto node_ids = trimming_handle.GetNodeIds(it.first);
      for(auto& node_id : node_ids)
      {
         optimization_problem_->TrimTrajectoryNode(node_id);
      }
      */

      //it.second.state = TrajectoryState::DELETED;
      //it.second.deletion_state = InternalTrajectoryState::DeletionState::NORMAL;

      /*
      auto submap_ids_test = trimming_handle.GetSubmapIds(it.first);
      LOG(INFO) << "okagv optimization_problem_ submap_ids size " << submap_ids_test.size();
      auto node_ids_test = trimming_handle.GetNodeIds(it.first);
      LOG(INFO) << "okagv optimization_problem_ node_ids size " << node_ids_test.size();
      */
    }

    /*
    LOG(INFO) << "data_.submap_data.size() " << data_.submap_data.size();
    LOG(INFO) << "data_.trajectory_nodes.size() "
              << data_.trajectory_nodes.size();
              */
  }
}

void PoseGraph2D::DeleteTrajectoriesIfNeeded(int trajectory_id) {
  TrimmingHandle trimming_handle(this);

  for (auto& it : data_.trajectories_state) {
    if (it.second.deletion_state ==
        InternalTrajectoryState::DeletionState::WAIT_FOR_DELETION) {
      // TODO(gaschler): Consider directly deleting from data_, which may be
      // more complete.
      auto submap_ids = trimming_handle.GetSubmapIds(it.first);
      for (auto& submap_id : submap_ids) {
        // LOG(INFO) << "okagv TrimSubmap " << submap_id;
        trimming_handle.TrimSubmap(submap_id);
      }

      // okagv
      data_.trajectories_state.erase(it.first);
      data_.initial_trajectory_poses.erase(it.first);
      trimmers_.clear();
    }
  }
}

void PoseGraph2D::HandleWorkQueue(
    const constraints::ConstraintBuilder2D::Result& result) {
  {
    absl::MutexLock locker(&mutex_);
    data_.constraints.insert(data_.constraints.end(), result.begin(),
                             result.end());
    //LOG(INFO) << "okagv data_.constraints.size " << data_.constraints.size();
  }

   //okagv
    //LOG(WARNING) << "okagv Start RunOptimization";

    //LOG(INFO) << "okagv HandleWorkQueue 0";
      //DeleteTrajectoriesIfNeeded();
   
    //LOG(INFO) << "okagv HandleWorkQueue 1";
    RunOptimization();

     //LOG(INFO) << "okagv HandleWorkQueue 2";
    if (global_slam_optimization_callback_) {
      std::map<int, NodeId> trajectory_id_to_last_optimized_node_id;
      std::map<int, SubmapId> trajectory_id_to_last_optimized_submap_id;
      {
        absl::MutexLock locker(&mutex_);
        const auto& submap_data = optimization_problem_->submap_data();
        const auto& node_data = optimization_problem_->node_data();
        for (const int trajectory_id : node_data.trajectory_ids()) {
          if (node_data.SizeOfTrajectoryOrZero(trajectory_id) == 0 ||
              submap_data.SizeOfTrajectoryOrZero(trajectory_id) == 0) {
            continue;
          }
          trajectory_id_to_last_optimized_node_id.emplace(
              trajectory_id,
              std::prev(node_data.EndOfTrajectory(trajectory_id))->id);
          trajectory_id_to_last_optimized_submap_id.emplace(
              trajectory_id,
              std::prev(submap_data.EndOfTrajectory(trajectory_id))->id);
        }
      }
      global_slam_optimization_callback_(
          trajectory_id_to_last_optimized_submap_id,
          trajectory_id_to_last_optimized_node_id);
    }
  

     //LOG(INFO) << "okagv HandleWorkQueue 3";
  {
    absl::MutexLock locker(&mutex_);
    for (const Constraint& constraint : result) {
      UpdateTrajectoryConnectivity(constraint);
    }
    //DeleteTrajectoriesIfNeeded();

    //okagv, use a independent thread to do this work
    /*
    {
    absl::MutexLock locker(&trimmer_mutex_);
    start_check_and_trim_submap_ = true;
    LOG(INFO) << "start_check_and_trim_submap_ is true";
    }
    */

    //LOG(INFO) << "okagv trimmers_ start";
    TrimmingHandle trimming_handle(this);
    for (auto& trimmer : trimmers_) {
      // okagv
      //LOG(INFO) << "okagv stable_localization_count "
                //<< stable_localization_count;

      if (current_trajectory_type_ == TrajectoryType::NAVIGATION &&
          stable_localization_count < 5)
        continue;  // okagv add

      trimmer->Trim(&trimming_handle);
    }
    trimmers_.erase(
        std::remove_if(trimmers_.begin(), trimmers_.end(),
                       [](std::unique_ptr<PoseGraphTrimmer>& trimmer) {
                         return trimmer->IsFinished();
                       }),
        trimmers_.end());

    //LOG(INFO) << "okagv trimmers_ end";
    
    num_nodes_since_last_loop_closure_ = 0;

    // Update the gauges that count the current number of constraints.
    double inter_constraints_same_trajectory = 0;
    double inter_constraints_different_trajectory = 0;
    for (const auto& constraint : data_.constraints) {
      if (constraint.tag ==
          cartographer::mapping::PoseGraph::Constraint::INTRA_SUBMAP) {
        continue;
      }
      if (constraint.node_id.trajectory_id ==
          constraint.submap_id.trajectory_id) {
        ++inter_constraints_same_trajectory;
      } else {
        ++inter_constraints_different_trajectory;
      }
    }
    kConstraintsSameTrajectoryMetric->Set(inter_constraints_same_trajectory);
    kConstraintsDifferentTrajectoryMetric->Set(
        inter_constraints_different_trajectory);
  }

    //LOG(INFO) << "okagv HandleWorkQueue 4";
  DrainWorkQueue();
}

void PoseGraph2D::DrainWorkQueue() {
  process_work_queue = true; //okagv
  size_t work_queue_size;
  //LOG(INFO) << "okagv DrainWorkQueue process_work_queue is TRUE";

  /*
  current_thread_pool_state_ = ThreadPoolState::ACTIVE;

  if (is_start_clear_work_queue == true) {
    LOG(INFO) << "okagv start clear work_queue";
    if (!work_queue_->empty()) {
      work_queue_->clear();
    }
    LOG(INFO) << "okagv clear done";
    is_end_clear_work_queue = true;
    //is_start_clear_work_queue = false;
    current_thread_pool_state_ = ThreadPoolState::IDLE;
    return;
  } else {
    //LOG(INFO) << "okagv start clear work_queue false";
  }
  */

  while (process_work_queue) {
    std::function<WorkItem::Result()> work_item;
    {
      absl::MutexLock locker(&work_queue_mutex_);
      if (work_queue_->empty()) {
        work_queue_.reset();
        //LOG(INFO) << "okagv work_queue_ is empty";
        return;
        //okagv
        //continue;
      }
      work_item = work_queue_->front().task;
      work_queue_->pop_front();
      work_queue_size = work_queue_->size();
      kWorkQueueSizeMetric->Set(work_queue_size);
    }
    process_work_queue = work_item() == WorkItem::Result::kDoNotRunOptimization;
    //LOG(INFO) << "okagv Do work_item";
  }

  //LOG(INFO) << "okagv process_work_queue is false";
  // We have to optimize again.
  //absl::MutexLock locker(&work_queue_mutex_); //okagv
  //LOG(INFO) << "okagv Do HandleWorkQueue";
  constraint_builder_.WhenDone(
      [this](const constraints::ConstraintBuilder2D::Result& result) {
        HandleWorkQueue(result);
      });
}

void PoseGraph2D::WaitForAllComputations() {
  int num_trajectory_nodes;
  {
    absl::MutexLock locker(&mutex_);
    num_trajectory_nodes = data_.num_trajectory_nodes;
    //LOG(INFO) << "okagv num_trajectory_nodes size " << num_trajectory_nodes;
  }

  const int num_finished_nodes_at_start =
      constraint_builder_.GetNumFinishedNodes();

  auto report_progress = [this, num_trajectory_nodes,
                          num_finished_nodes_at_start]() {
    // Log progress on nodes only when we are actually processing nodes.
    if (num_trajectory_nodes != num_finished_nodes_at_start) {
      std::ostringstream progress_info;
      progress_info << "Optimizing: " << std::fixed << std::setprecision(1)
                    << 100. *
                           (constraint_builder_.GetNumFinishedNodes() -
                            num_finished_nodes_at_start) /
                           (num_trajectory_nodes - num_finished_nodes_at_start)
                    << "%...";
      std::cout << "\r\x1b[K" << progress_info.str() << std::flush;
    }
  };

  // First wait for the work queue to drain so that it's safe to schedule
  // a WhenDone() callback.
  {
    const auto predicate = [this]()
                               EXCLUSIVE_LOCKS_REQUIRED(work_queue_mutex_) {
                                 return work_queue_ == nullptr;
                               };
    absl::MutexLock locker(&work_queue_mutex_);
    while (!work_queue_mutex_.AwaitWithTimeout(
        absl::Condition(&predicate),
        absl::FromChrono(common::FromSeconds(5.)))) {  //1.0
      report_progress();
    }
  }

  // Now wait for any pending constraint computations to finish.
  absl::MutexLock locker(&mutex_);

  bool notification = false;
  constraint_builder_.WhenDone(
      [this,
       &notification](const constraints::ConstraintBuilder2D::Result& result)
          LOCKS_EXCLUDED(mutex_) {
            absl::MutexLock locker(&mutex_);
            data_.constraints.insert(data_.constraints.end(), result.begin(),
                                     result.end());
            notification = true;
          });
  const auto predicate = [&notification]() EXCLUSIVE_LOCKS_REQUIRED(mutex_) {
    return notification;
  };
  while (!mutex_.AwaitWithTimeout(absl::Condition(&predicate),
                                  absl::FromChrono(common::FromSeconds(5.)))) { //1.0
    report_progress();
  }

  //cartographer
  //CHECK_EQ(constraint_builder_.GetNumFinishedNodes(), num_trajectory_nodes);
  //okagv
  CHECK_GE(constraint_builder_.GetNumFinishedNodes(), num_trajectory_nodes);
  std::cout << "\r\x1b[KOptimizing: Done.     " << std::endl;
}

void PoseGraph2D::WaitForAllComputationsThenFinishTrajectory(
    int trajectory_id) {
  int num_trajectory_nodes;
  {
    absl::MutexLock locker(&mutex_);
    num_trajectory_nodes = data_.num_trajectory_nodes;
    // LOG(INFO) << "okagv num_trajectory_nodes size " <<
    // num_trajectory_nodes;
  }

  const int num_finished_nodes_at_start =
      constraint_builder_.GetNumFinishedNodes();

  auto report_progress = [this, num_trajectory_nodes,
                          num_finished_nodes_at_start]() {
    // Log progress on nodes only when we are actually processing nodes.
    if (num_trajectory_nodes != num_finished_nodes_at_start) {
      std::ostringstream progress_info;
      progress_info << "Optimizing: " << std::fixed << std::setprecision(1)
                    << 100. *
                           (constraint_builder_.GetNumFinishedNodes() -
                            num_finished_nodes_at_start) /
                           (num_trajectory_nodes - num_finished_nodes_at_start)
                    << "%...";
      std::cout << "\r\x1b[K" << progress_info.str() << std::flush;
    }
  };

  // First wait for the work queue to drain so that it's safe to schedule
  // a WhenDone() callback.
  {
    const auto predicate = [this]()
                               EXCLUSIVE_LOCKS_REQUIRED(work_queue_mutex_) {
                                 return work_queue_ == nullptr;
                               };
    absl::MutexLock locker(&work_queue_mutex_);
    while (!work_queue_mutex_.AwaitWithTimeout(
        absl::Condition(&predicate),
        absl::FromChrono(common::FromSeconds(5.)))) {  // 1.0
      report_progress();
    }
  }

  // Now wait for any pending constraint computations to finish.
  absl::MutexLock locker(&mutex_);

  bool notification = false;
  constraint_builder_.WhenDone(
      [this, &notification,
       &trajectory_id](const constraints::ConstraintBuilder2D::Result& result)
          LOCKS_EXCLUDED(mutex_) {
            absl::MutexLock locker(&mutex_);
            data_.constraints.insert(data_.constraints.end(), result.begin(),
                                     result.end());

            data_.trajectories_state[trajectory_id].state =
                TrajectoryState::FINISHED;

            for (const auto& submap :
                 data_.submap_data.trajectory(trajectory_id)) {
              data_.submap_data.at(submap.id).state = SubmapState::kFinished;
            }

            notification = true;
          });
  const auto predicate = [&notification]() EXCLUSIVE_LOCKS_REQUIRED(mutex_) {
    return notification;
  };
  while (!mutex_.AwaitWithTimeout(
      absl::Condition(&predicate),
      absl::FromChrono(common::FromSeconds(5.)))) {  // 1.0
    report_progress();
  }

  // cartographer
  // CHECK_EQ(constraint_builder_.GetNumFinishedNodes(), num_trajectory_nodes);
  // okagv
  CHECK_GE(constraint_builder_.GetNumFinishedNodes(), num_trajectory_nodes);
  std::cout << "\r\x1b[WaitForAllComputationsThenFinishTrajectory: Done.     " << std::endl;
}

void PoseGraph2D::WaitForAllComputationsThenDeleteTrajectory(
    int trajectory_id) {
  int num_trajectory_nodes;
  {
    absl::MutexLock locker(&mutex_);
    num_trajectory_nodes = data_.num_trajectory_nodes;
    // LOG(INFO) << "okagv num_trajectory_nodes size " <<
    // num_trajectory_nodes;
  }

  const int num_finished_nodes_at_start =
      constraint_builder_.GetNumFinishedNodes();

  auto report_progress = [this, num_trajectory_nodes,
                          num_finished_nodes_at_start]() {
    // Log progress on nodes only when we are actually processing nodes.
    if (num_trajectory_nodes != num_finished_nodes_at_start) {
      std::ostringstream progress_info;
      progress_info << "Optimizing: " << std::fixed << std::setprecision(1)
                    << 100. *
                           (constraint_builder_.GetNumFinishedNodes() -
                            num_finished_nodes_at_start) /
                           (num_trajectory_nodes - num_finished_nodes_at_start)
                    << "%...";
      std::cout << "\r\x1b[K" << progress_info.str() << std::flush;
    }
  };

  // First wait for the work queue to drain so that it's safe to schedule
  // a WhenDone() callback.
  {
    const auto predicate = [this]()
                               EXCLUSIVE_LOCKS_REQUIRED(work_queue_mutex_) {
                                 return work_queue_ == nullptr;
                               };
    absl::MutexLock locker(&work_queue_mutex_);
    while (!work_queue_mutex_.AwaitWithTimeout(
        absl::Condition(&predicate),
        absl::FromChrono(common::FromSeconds(5.)))) {  // 1.0
      report_progress();
    }
  }

  // Now wait for any pending constraint computations to finish.
  absl::MutexLock locker(&mutex_);

  bool notification = false;
  constraint_builder_.WhenDone(
      [this, &notification,
       &trajectory_id](const constraints::ConstraintBuilder2D::Result& result)
          LOCKS_EXCLUDED(mutex_) {
            absl::MutexLock locker(&mutex_);
            data_.constraints.insert(data_.constraints.end(), result.begin(),
                                     result.end());

            // LOG(INFO) << "okagv DeleteTrajectory 0" << trajectory_id;
            //LOG(INFO) << "okagv Block 0";
            auto it = data_.trajectories_state.find(trajectory_id);
            if (it == data_.trajectories_state.end()) {
              LOG(WARNING)
                  << "Skipping request to delete non-existing trajectory_id: "
                  << trajectory_id;
              return;
            }
            it->second.deletion_state =
                InternalTrajectoryState::DeletionState::SCHEDULED_FOR_DELETION;

            CHECK(data_.trajectories_state.at(trajectory_id).state !=
                  TrajectoryState::ACTIVE);
            CHECK(data_.trajectories_state.at(trajectory_id).state !=
                  TrajectoryState::DELETED);
            CHECK(
                data_.trajectories_state.at(trajectory_id).deletion_state ==
                InternalTrajectoryState::DeletionState::SCHEDULED_FOR_DELETION);
            data_.trajectories_state.at(trajectory_id).deletion_state =
                InternalTrajectoryState::DeletionState::WAIT_FOR_DELETION;

            //LOG(INFO) << "okagv Block 1";
            TrimmingHandle trimming_handle(this);

            //LOG(INFO) << "okagv Block 2 data_.trajectories_state.size() "
            //          << data_.trajectories_state.size();

            std::vector<int> removed_id;
            for (std::map<int, InternalTrajectoryState>::iterator it = data_.trajectories_state.begin();
                 it != data_.trajectories_state.end(); it++) {
              //LOG(INFO) << "okagv Block 3 trajectory_id " << it->first;
              if (it->second.deletion_state ==
                  InternalTrajectoryState::DeletionState::WAIT_FOR_DELETION) {
                // TODO(gaschler): Consider directly deleting from data_, which
                // may be more complete.
                auto submap_ids = trimming_handle.GetSubmapIds(it->first);
                for (auto& submap_id : submap_ids) {
                  //LOG(INFO) << "okagv Block 4 " << submap_id;
                  trimming_handle.TrimSubmap(submap_id);
                }

                //LOG(INFO) << "okagv Block 5";
                removed_id.push_back(it->first);
              }
            }

            for (auto it : removed_id) {
              // okagv
              data_.trajectories_state.erase(it);
              data_.initial_trajectory_poses.erase(it);
              //LOG(INFO) << "okagv Block 6";
            }

            //old way of cartographer
            /*
            for (auto& it : data_.trajectories_state) {
              LOG(INFO) << "okagv Block 3 trajectory_id " << it.first;
              if (it.second.deletion_state ==
                  InternalTrajectoryState::DeletionState::WAIT_FOR_DELETION) {
                // TODO(gaschler): Consider directly deleting from data_, which
                // may be more complete.
                auto submap_ids = trimming_handle.GetSubmapIds(it.first);
                for (auto& submap_id : submap_ids) {
                  LOG(INFO) << "okagv Block 4 " << submap_id;
                  trimming_handle.TrimSubmap(submap_id);

                }

                LOG(INFO) << "okagv Block 5";
                // okagv
                data_.trajectories_state.erase(it.first);
                data_.initial_trajectory_poses.erase(it.first);
                LOG(INFO) << "okagv Block 6";
              }
            }
            */

            // okagv
            //LOG(INFO) << "okagv Block 7";
            if (data_.trajectories_state.empty()) {
              trimmers_.clear();
            }

            notification = true;
          });
  const auto predicate = [&notification]() EXCLUSIVE_LOCKS_REQUIRED(mutex_) {
    return notification;
  };
  while (!mutex_.AwaitWithTimeout(
      absl::Condition(&predicate),
      absl::FromChrono(common::FromSeconds(5.)))) {  // 1.0
    report_progress();
  }

  // cartographer
  // CHECK_EQ(constraint_builder_.GetNumFinishedNodes(), num_trajectory_nodes);
  // okagv
  CHECK_GE(constraint_builder_.GetNumFinishedNodes(), num_trajectory_nodes);
  std::cout << "\r\x1b[WaitForAllComputationsThenDeleteTrajectory: Done.     "
            << std::endl;
}

void PoseGraph2D::DeleteTrajectory(const int trajectory_id) {
  //{
  // absl::MutexLock locker_others(&work_queue_mutex_);

  /// absl::MutexLock locker(&mutex_);

  //LOG(INFO) << "okagv DeleteTrajectory " << trajectory_id;

  // while (!process_work_queue) {
  // LOG(WARNING) << "okagv Wait for process_work_queue true";
  // absl::SleepFor(absl::Milliseconds(50));
  //}

  WaitForAllComputationsThenDeleteTrajectory(trajectory_id);
  return;

  // below is old way
  // okagv
  WaitForAllComputations();

  //LOG(INFO) << "okagv then DeleteTrajectory " << trajectory_id;
  {
    absl::MutexLock locker(&mutex_);

    // LOG(INFO) << "okagv What's happen?";
    // ClearWorkQueue();

    // absl::MutexLock locker_others(&work_queue_mutex_);

    // okagv ,clear all work_queue_
    /*
    //LOG(INFO) << "okagv work_queue_ size " << work_queue_->
    while (!work_queue_->empty()) {
      work_queue_->pop_front();
      LOG(WARNING) << "okagv work_queue pop task ,remain ";
                   //<< work_queue_->size() << " task";
    }
    */
    //

    // LOG(INFO) << "okagv DeleteTrajectory 0" << trajectory_id;
    auto it = data_.trajectories_state.find(trajectory_id);
    if (it == data_.trajectories_state.end()) {
      LOG(WARNING) << "Skipping request to delete non-existing trajectory_id: "
                   << trajectory_id;
      return;
    }
    it->second.deletion_state =
        InternalTrajectoryState::DeletionState::SCHEDULED_FOR_DELETION;
    //}

    /*
    AddWorkItem([this, trajectory_id]() LOCKS_EXCLUDED(mutex_) {
      absl::MutexLock locker(&mutex_);
      //LOG(INFO) << "okagv AddWorkItem DeleteTrajectory " << trajectory_id;
      CHECK(data_.trajectories_state.at(trajectory_id).state !=
            TrajectoryState::ACTIVE);
      CHECK(data_.trajectories_state.at(trajectory_id).state !=
            TrajectoryState::DELETED);
      CHECK(data_.trajectories_state.at(trajectory_id).deletion_state ==
            InternalTrajectoryState::DeletionState::SCHEDULED_FOR_DELETION);
      data_.trajectories_state.at(trajectory_id).deletion_state =
          InternalTrajectoryState::DeletionState::WAIT_FOR_DELETION;
      //return WorkItem::Result::kDoNotRunOptimization;
      return WorkItem::Result::kRunOptimization; //okagv
    });
    */

    // LOG(INFO) << "okagv DeleteTrajectory 1" << trajectory_id;
    // okagvF
    CHECK(data_.trajectories_state.at(trajectory_id).state !=
          TrajectoryState::ACTIVE);
    CHECK(data_.trajectories_state.at(trajectory_id).state !=
          TrajectoryState::DELETED);
    CHECK(data_.trajectories_state.at(trajectory_id).deletion_state ==
          InternalTrajectoryState::DeletionState::SCHEDULED_FOR_DELETION);
    data_.trajectories_state.at(trajectory_id).deletion_state =
        InternalTrajectoryState::DeletionState::WAIT_FOR_DELETION;

    // DeleteTrajectoriesIfNeeded(trajectory_id);

    // LOG(INFO) << "okagv DeleteTrajectory 2" << trajectory_id;
    TrimmingHandle trimming_handle(this);

    // LOG(INFO) << "okagv DeleteTrajectory 3" << trajectory_id;
    for (auto& it : data_.trajectories_state) {
      if (it.second.deletion_state ==
          InternalTrajectoryState::DeletionState::WAIT_FOR_DELETION) {
        // TODO(gaschler): Consider directly deleting from data_, which may be
        // more complete.
        auto submap_ids = trimming_handle.GetSubmapIds(it.first);
        for (auto& submap_id : submap_ids) {
          // LOG(INFO) << "okagv Start TrimSubmap " << submap_id;
          trimming_handle.TrimSubmap(submap_id);
          // LOG(INFO) << "okagv End TrimSubmap " << submap_id;
        }

        // okagv
        data_.trajectories_state.erase(it.first);
        data_.initial_trajectory_poses.erase(it.first);
        // trimmers_.clear();
      }
    }

    // okagv
    if (data_.trajectories_state.empty()) {
      // LOG(INFO) << "okagv trimmers_ clear";
      trimmers_.clear();
    }
  }
  // LOG(INFO) << "okagv DeleteTrajectory 4" << trajectory_id;
}

void PoseGraph2D::ClearWorkQueue() {
  /*
{
absl::MutexLock locker(&work_queue_mutex_);

LOG(INFO) << "okagv ClearWorkQueue 4";
if (work_queue_->empty()) {
  //LOG(INFO) << "okagv WorkQueue empty";
} else {
  work_queue_->pop_front();
  //LOG(INFO) << "okagv WorkQueue not empty";
}

// while(!)
// work_queue_->pop_front();
}
*/
}

void PoseGraph2D::FinishTrajectory(const int trajectory_id) {

  /*
  AddWorkItem([this, trajectory_id]() LOCKS_EXCLUDED(mutex_) {
    absl::MutexLock locker(&mutex_);
    LOG(INFO) << "okagv AddWorkItem FinishTrajectory " << trajectory_id;
    CHECK(!IsTrajectoryFinished(trajectory_id));
    data_.trajectories_state[trajectory_id].state = TrajectoryState::FINISHED;

    for (const auto& submap : data_.submap_data.trajectory(trajectory_id)) {
      // LOG(INFO) << "okagv data_.submap_data 15";
      data_.submap_data.at(submap.id).state = SubmapState::kFinished;
    }

    return WorkItem::Result::kDoNotRunOptimization;  // okagv
  });
  */
  //
  //okagv
  WaitForAllComputationsThenFinishTrajectory(trajectory_id);
  return;

  // below is old way;
  WaitForAllComputations();
 
  //LOG(INFO) << "okagv then FinishTrajectory " << trajectory_id;
  {
  absl::MutexLock locker(&mutex_);

  CHECK(!IsTrajectoryFinished(trajectory_id));
  data_.trajectories_state[trajectory_id].state = TrajectoryState::FINISHED;

  for (const auto& submap : data_.submap_data.trajectory(trajectory_id)) {
    data_.submap_data.at(submap.id).state = SubmapState::kFinished;
  }

  }

}

bool PoseGraph2D::IsTrajectoryFinished(const int trajectory_id) const {
  return data_.trajectories_state.count(trajectory_id) != 0 &&
         data_.trajectories_state.at(trajectory_id).state ==
             TrajectoryState::FINISHED;
}

void PoseGraph2D::FreezeTrajectory(const int trajectory_id) {
  {
    absl::MutexLock locker(&mutex_);
    data_.trajectory_connectivity_state.Add(trajectory_id);
  }

  //LOG(INFO) << "okagv AddWorkItem FreezeTrajectory";
  AddWorkItem([this, trajectory_id]() LOCKS_EXCLUDED(mutex_) {
    absl::MutexLock locker(&mutex_);
    CHECK(!IsTrajectoryFrozen(trajectory_id));
    // Connect multiple frozen trajectories among each other.
    // This is required for localization against multiple frozen trajectories
    // because we lose inter-trajectory constraints when freezing.
    for (const auto& entry : data_.trajectories_state) {
      const int other_trajectory_id = entry.first;
      if (!IsTrajectoryFrozen(other_trajectory_id)) {
        continue;
      }
      if (data_.trajectory_connectivity_state.TransitivelyConnected(
              trajectory_id, other_trajectory_id)) {
        // Already connected, nothing to do.
        continue;
      }
      data_.trajectory_connectivity_state.Connect(
          trajectory_id, other_trajectory_id, common::FromUniversal(0));
    }
    data_.trajectories_state[trajectory_id].state = TrajectoryState::FROZEN;
    return WorkItem::Result::kDoNotRunOptimization;
  });
}

bool PoseGraph2D::IsTrajectoryFrozen(const int trajectory_id) const {
  return data_.trajectories_state.count(trajectory_id) != 0 &&
         data_.trajectories_state.at(trajectory_id).state ==
             TrajectoryState::FROZEN;
}

void PoseGraph2D::AddSubmapFromProto(
    const transform::Rigid3d& global_submap_pose, const proto::Submap& submap) {

  if (!submap.has_submap_2d()) {
    //LOG(INFO) << "okagv AddSubmapFromProto 0";
    return;
  }
  
  //LOG(INFO) << "okagv data_.submap_data.size " << data_.submap_data.size();
  const SubmapId submap_id = {submap.submap_id().trajectory_id(),
                              submap.submap_id().submap_index()};

  const transform::Rigid2d global_submap_pose_2d =
      transform::Project2D(global_submap_pose);
  {
    absl::MutexLock locker(&mutex_);

    const std::shared_ptr<const Submap2D> submap_ptr =
        std::make_shared<const Submap2D>(submap.submap_2d(),
                                         &conversion_tables_);
                                         
    AddTrajectoryIfNeeded(submap_id.trajectory_id);
    if (!CanAddWorkItemModifying(submap_id.trajectory_id))
    {
          //LOG(INFO) << "okagv AddSubmapFromProto 1";
          return;
    }

    data_.submap_data.Insert(submap_id, InternalSubmapData());
    data_.submap_data.at(submap_id).submap = submap_ptr;
    data_.global_submap_poses_2d.Insert(
        submap_id, optimization::SubmapSpec2D{global_submap_pose_2d});
  }

  // TODO(MichaelGrupp): MapBuilder does freezing before deserializing submaps,
  // so this should be fine.
  if (IsTrajectoryFrozen(submap_id.trajectory_id)) {
    kFrozenSubmapsMetric->Increment();
  } else {
    kActiveSubmapsMetric->Increment();
  }

  //LOG(INFO) << "okagv AddWorkItem AddSubmapFromProto";
  AddWorkItem(
      [this, submap_id, global_submap_pose_2d]() LOCKS_EXCLUDED(mutex_) {
        absl::MutexLock locker(&mutex_);
        //LOG(INFO) << "okagv data_.submap_data 17";
        data_.submap_data.at(submap_id).state = SubmapState::kFinished;
        optimization_problem_->InsertSubmap(submap_id, global_submap_pose_2d);
        return WorkItem::Result::kDoNotRunOptimization;
        //return WorkItem::Result::kRunOptimization; //okagv
      });
}

void PoseGraph2D::AddNodeFromProto(const transform::Rigid3d& global_pose,
                                   const proto::Node& node) {
  const NodeId node_id = {node.node_id().trajectory_id(),
                          node.node_id().node_index()};
  std::shared_ptr<const TrajectoryNode::Data> constant_data =
      std::make_shared<const TrajectoryNode::Data>(FromProto(node.node_data()));

      //LOG(INFO) << "okagv AddNodeFromProto " << constant_data->time;

  {
    absl::MutexLock locker(&mutex_);
    AddTrajectoryIfNeeded(node_id.trajectory_id);
    if (!CanAddWorkItemModifying(node_id.trajectory_id)) return;
    data_.trajectory_nodes.Insert(node_id,
                                  TrajectoryNode{constant_data, global_pose});
  }

    //LOG(INFO) << "okagv AddWorkItem AddNodeFromProto";
  AddWorkItem([this, node_id, global_pose]() LOCKS_EXCLUDED(mutex_) {
    absl::MutexLock locker(&mutex_);
    const auto& constant_data =
        data_.trajectory_nodes.at(node_id).constant_data;
    const auto gravity_alignment_inverse = transform::Rigid3d::Rotation(
        constant_data->gravity_alignment.inverse());
    optimization_problem_->InsertTrajectoryNode(
        node_id,
        optimization::NodeSpec2D{
            constant_data->time,
            transform::Project2D(constant_data->local_pose *
                                 gravity_alignment_inverse),
            transform::Project2D(global_pose * gravity_alignment_inverse),
            constant_data->gravity_alignment});
    return WorkItem::Result::kDoNotRunOptimization;
  });
}

void PoseGraph2D::SetTrajectoryDataFromProto(
    const proto::TrajectoryData& data) {
  TrajectoryData trajectory_data;
  // gravity_constant and imu_calibration are omitted as its not used in 2d

  if (data.has_fixed_frame_origin_in_map()) {
    trajectory_data.fixed_frame_origin_in_map =
        transform::ToRigid3(data.fixed_frame_origin_in_map());

     //LOG(INFO) << "okagv AddWorkItem SetTrajectoryDataFromProto";
    const int trajectory_id = data.trajectory_id();
    AddWorkItem([this, trajectory_id, trajectory_data]()
                    LOCKS_EXCLUDED(mutex_) {
                      absl::MutexLock locker(&mutex_);
                      if (CanAddWorkItemModifying(trajectory_id)) {
                        optimization_problem_->SetTrajectoryData(
                            trajectory_id, trajectory_data);
                      }
                      return WorkItem::Result::kDoNotRunOptimization;
                    });
  }
}

void PoseGraph2D::AddNodeToSubmap(const NodeId& node_id,
                                  const SubmapId& submap_id) {

       //LOG(INFO) << "okagv AddWorkItem AddNodeToSubmap";
  AddWorkItem([this, node_id, submap_id]() LOCKS_EXCLUDED(mutex_) {
    absl::MutexLock locker(&mutex_);
    if (CanAddWorkItemModifying(submap_id.trajectory_id)) {
      //LOG(INFO) << "okagv data_.submap_data 18";
      data_.submap_data.at(submap_id).node_ids.insert(node_id);
    }
    return WorkItem::Result::kDoNotRunOptimization;
  });
}

void PoseGraph2D::AddSerializedConstraints(
    const std::vector<Constraint>& constraints) {

             //LOG(INFO) << "okagv AddWorkItem AddSerializedConstraints";
  AddWorkItem([this, constraints]() LOCKS_EXCLUDED(mutex_) {
    absl::MutexLock locker(&mutex_);
    for (const auto& constraint : constraints) {
      //LOG(INFO) << "okagv constraint.node_id " << constraint.node_id;
      CHECK(data_.trajectory_nodes.Contains(constraint.node_id));
      CHECK(data_.submap_data.Contains(constraint.submap_id));
      CHECK(data_.trajectory_nodes.at(constraint.node_id).constant_data !=
            nullptr);
            //LOG(INFO) << "okagv data_.submap_data 19";
      CHECK(data_.submap_data.at(constraint.submap_id).submap != nullptr);
      switch (constraint.tag) {
        case Constraint::Tag::INTRA_SUBMAP:
        //LOG(INFO) << "okagv data_.submap_data 20";
          CHECK(data_.submap_data.at(constraint.submap_id)
                    .node_ids.emplace(constraint.node_id)
                    .second);
          break;
        case Constraint::Tag::INTER_SUBMAP:
          UpdateTrajectoryConnectivity(constraint);
          break;
      }
      const Constraint::Pose pose = {
          constraint.pose.zbar_ij *
              transform::Rigid3d::Rotation(
                  data_.trajectory_nodes.at(constraint.node_id)
                      .constant_data->gravity_alignment.inverse()),
          constraint.pose.translation_weight, constraint.pose.rotation_weight};

      data_.constraints.push_back(Constraint{
          constraint.submap_id, constraint.node_id, pose, constraint.tag});
    }
    //LOG(INFO) << "Loaded " << constraints.size() << " constraints.";
    return WorkItem::Result::kDoNotRunOptimization;
  });
}

void PoseGraph2D::AddTrimmer(std::unique_ptr<PoseGraphTrimmer> trimmer) {
  // C++11 does not allow us to move a unique_ptr into a lambda.
  PoseGraphTrimmer* const trimmer_ptr = trimmer.release();
               //LOG(INFO) << "okagv AddWorkItem AddTrimmer";
  AddWorkItem([this, trimmer_ptr]() LOCKS_EXCLUDED(mutex_) {
    absl::MutexLock locker(&mutex_);
    trimmers_.emplace_back(trimmer_ptr);
    return WorkItem::Result::kDoNotRunOptimization;
  });
}

void PoseGraph2D::RunFinalOptimization() {
  {
                   //LOG(INFO) << "okagv AddWorkItem RunFinalOptimization";
    AddWorkItem([this]() LOCKS_EXCLUDED(mutex_) {
      absl::MutexLock locker(&mutex_);
      optimization_problem_->SetMaxNumIterations(
          options_.max_num_final_iterations());
      return WorkItem::Result::kRunOptimization;
    });
    AddWorkItem([this]() LOCKS_EXCLUDED(mutex_) {
      absl::MutexLock locker(&mutex_);
      optimization_problem_->SetMaxNumIterations(
          options_.optimization_problem_options()
              .ceres_solver_options()
              .max_num_iterations());
      return WorkItem::Result::kDoNotRunOptimization;
    });
  }
  WaitForAllComputations();
}

void PoseGraph2D::RunOptimization() {
  if (optimization_problem_->submap_data().empty()) {
    return;
  }

  // No other thread is accessing the optimization_problem_,
  // data_.constraints, data_.frozen_trajectories and data_.landmark_nodes
  // when executing the Solve. Solve is time consuming, so not taking the mutex
  // before Solve to avoid blocking foreground processing.
  //LOG(INFO) << "okagv start RunOptimization";
  /*
  if(current_trajectory_type_ == TrajectoryType::RELOCALIZAION)
  {
      current_trajectory_type_ = TrajectoryType::NAVIGATION;
  }
  */

  optimization_problem_->Solve(data_.constraints, GetTrajectoryStates(),
                               data_.landmark_nodes);

  //okagv author add it , but okagv remove it
  absl::MutexLock locker(&mutex_);

  const auto& submap_data = optimization_problem_->submap_data();
  const auto& node_data = optimization_problem_->node_data();

  for (const int trajectory_id : node_data.trajectory_ids()) {
    for (const auto& node : node_data.trajectory(trajectory_id)) {
      auto& mutable_trajectory_node = data_.trajectory_nodes.at(node.id);
      mutable_trajectory_node.global_pose =
          transform::Embed3D(node.data.global_pose_2d) *
          transform::Rigid3d::Rotation(
              mutable_trajectory_node.constant_data->gravity_alignment);
    }

    // Extrapolate all point cloud poses that were not included in the
    // 'optimization_problem_' yet.
          //LOG(INFO) << "okagv RunOptimization 2"; 
    const auto local_to_new_global =
        ComputeLocalToGlobalTransform(submap_data, trajectory_id);
              //LOG(INFO) << "okagv RunOptimization 3"; 
    const auto local_to_old_global = ComputeLocalToGlobalTransform(
        data_.global_submap_poses_2d, trajectory_id);
    const transform::Rigid3d old_global_to_new_global =
        local_to_new_global * local_to_old_global.inverse();

    const NodeId last_optimized_node_id =
        std::prev(node_data.EndOfTrajectory(trajectory_id))->id;
    auto node_it =
        std::next(data_.trajectory_nodes.find(last_optimized_node_id));

    for (; node_it != data_.trajectory_nodes.EndOfTrajectory(trajectory_id);
         ++node_it) {
      auto& mutable_trajectory_node = data_.trajectory_nodes.at(node_it->id);
      mutable_trajectory_node.global_pose =
          old_global_to_new_global * mutable_trajectory_node.global_pose;
    }
  }

  for (const auto& landmark : optimization_problem_->landmark_data()) {
    data_.landmark_nodes[landmark.first].global_landmark_pose = landmark.second;
  }
  data_.global_submap_poses_2d = submap_data;
}

bool PoseGraph2D::CanAddWorkItemModifying(int trajectory_id) {
  auto it = data_.trajectories_state.find(trajectory_id);
  if (it == data_.trajectories_state.end()) {
    return true;
  }
  if (it->second.state == TrajectoryState::FINISHED) {
    // TODO(gaschler): Replace all FATAL to WARNING after some testing.
    // okagv okagv do as gaschler do
    LOG(WARNING) << "trajectory_id " << trajectory_id
               << " has finished "
                  "but modification is requested, skipping.";
    return false;
  }
  if (it->second.deletion_state !=
      InternalTrajectoryState::DeletionState::NORMAL) {
    LOG(WARNING) << "trajectory_id " << trajectory_id
               << " has been scheduled for deletion "
                  "but modification is requested, skipping.";
    return false;
  }
  
  if (it->second.state == TrajectoryState::DELETED) {
    LOG(WARNING) << "trajectory_id " << trajectory_id
               << " has been deleted "
                  "but modification is requested, skipping.";
    return false;
  }
  
  return true;
}

MapById<NodeId, TrajectoryNode> PoseGraph2D::GetTrajectoryNodes() const {
  absl::MutexLock locker(&mutex_);
  /*
  LOG(INFO) << "okagv test GetTrajectoryNodes";
  for (const auto& node_id_data : data_.trajectory_nodes) {
        LOG(INFO) << "okagv " << node_id_data.id;
  }
  */

  return data_.trajectory_nodes;
}

MapById<NodeId, TrajectoryNode> PoseGraph2D::GetTrajectoryNodesAfterUpdate() const {
  absl::MutexLock locker(&mutex_);

    MapById<NodeId, TrajectoryNode> nodes;
    int old_map_last_node_index = 0;

  for (const auto& node_id_data : data_.trajectory_nodes) {
    if (node_id_data.id.trajectory_id == 0) {
      nodes.Insert(node_id_data.id, node_id_data.data);
      old_map_last_node_index = node_id_data.id.node_index ;
    } else if (node_id_data.id.trajectory_id == 1001) {

      //int loaded_map_node_count = data_.trajectory_nodes.EndOfTrajectory(0)->id.node_index;
      //LOG(INFO) << "okagv old_map_last_node_index " << old_map_last_node_index;
      //LOG(INFO) << "okagv navigation node " <<  node_id_data.id.node_index +  old_map_last_node_index + 2;
      NodeId id(0,node_id_data.id.node_index + old_map_last_node_index + 2);
      nodes.Insert(id, node_id_data.data);
    }
  }
  return nodes;

}

MapById<NodeId, TrajectoryNodePose> PoseGraph2D::GetTrajectoryNodePoses()
    const {
  MapById<NodeId, TrajectoryNodePose> node_poses;
  absl::MutexLock locker(&mutex_);
  for (const auto& node_id_data : data_.trajectory_nodes) {
    absl::optional<TrajectoryNodePose::ConstantPoseData> constant_pose_data;
    if (node_id_data.data.constant_data != nullptr) {
      constant_pose_data = TrajectoryNodePose::ConstantPoseData{
          node_id_data.data.constant_data->time,
          node_id_data.data.constant_data->local_pose};
    }
    node_poses.Insert(
        node_id_data.id,
        TrajectoryNodePose{node_id_data.data.global_pose, constant_pose_data});
  }
  return node_poses;
}

std::map<int, PoseGraphInterface::TrajectoryState>
PoseGraph2D::GetTrajectoryStates() const {
  std::map<int, PoseGraphInterface::TrajectoryState> trajectories_state;
  absl::MutexLock locker(&mutex_);
  for (const auto& it : data_.trajectories_state) {
    trajectories_state[it.first] = it.second.state;
  }
  return trajectories_state;
}

PoseGraphInterface::OKagvOrder
PoseGraph2D::GetOKagv_Order() const {
    return current_okagv_order_;
}

void PoseGraph2D::SetOKagv_Order(const OKagvOrder& order) {
    current_okagv_order_ = order;
}

PoseGraphInterface::OKagvFeedback PoseGraph2D::GetOKagv_Feedback() const {
    return current_okagv_feedback_;
}

void PoseGraph2D::SetOKagv_Feedback(const OKagvFeedback& feedback) {
  current_okagv_feedback_ = feedback;
}

bool PoseGraph2D::LocalizeOKagvPoses(const bool use_initial_pose,
                                     const int trajectory_id,
                                     const transform::Rigid3d initial_pose) {
  //reset the flag before receive new node id
  /*
  while(1)
  {
    absl::SleepFor(absl::Milliseconds(100));
    LOG(INFO) << "is_end_clear_work_queue is true";
  }
  */
  int wait_count = 0;
  int wait_time = 20;

    while (!data_.submap_data.SizeOfTrajectoryOrZero(current_localization_trajectory_id_)) {
    //LOG(WARNING) << "okagv Wait for node_id";
    absl::SleepFor(absl::Milliseconds(wait_time));
      wait_count++;
      if(wait_count*wait_time > 10*1000)
      {
        LOG(INFO) << "Failed to Localize map because of time out";
        return false;
      }
  } 

  absl::MutexLock locker(&mutex_);

  if (!initial_pose.IsValid()) return false;

  // set the localization map's id
  current_map_trajectory_id_ = trajectory_id;
  current_trajectory_type_ = TrajectoryType::RELOCALIZAION;


  //LOG(INFO) << "okagv AddWorkItem LocalizeOKagvPoses";
    AddWorkItem([=]() LOCKS_EXCLUDED(mutex_) {
    return LocalizeOKagvPosesWorkItem(use_initial_pose, trajectory_id,
                                     initial_pose);
  });

  /*
  if (!use_initial_pose) return false;
  //
  covariance_score_ = 0;
  std::vector<SubmapId> submap_ids;

  submap_ids = FindPoseOfSubmapInTrajectory(initial_pose);

  for (size_t i = 0; i < submap_ids.size(); i++) {
    LOG(INFO) << "okagv submap trajectory_id "
              << submap_ids[i].trajectory_id;
    LOG(INFO) << "okagv submap submap_index " << submap_ids[i].submap_index;
    const NodeId last_node =
        std::prev(optimization_problem_->node_data().EndOfTrajectory(
                      current_localization_trajectory_id_))
            ->id;
    const SubmapId submap_found = submap_ids[i];

    //
    const transform::Rigid2d initial_pose_2d =
        transform::Project2D(initial_pose);

    const transform::Rigid2d initial_relative_pose =
        optimization_problem_->submap_data()
            .at(submap_found)
            .global_pose.inverse() *
        initial_pose_2d;

    add_relocalization_constraint = true;
    ComputeConstraintForRelocalization(last_node, submap_found,
                                       initial_relative_pose);
    add_relocalization_constraint = false;
  }

  // set only the localization map work
  for (const auto& it : data_.trajectories_state) {
    if (it.first == trajectory_id) {
      SetTrajectoryState(it.first, TrajectoryState::FROZEN);
    } else if (it.first == 1001) {
      continue;
    } else {
      SetTrajectoryState(it.first, TrajectoryState::IDLE);
    }
  }

  // absl::SleepFor(absl::Milliseconds(1000));
  constraint_builder_.NotifyEndOfNode();  // okagv remove

  AddWorkItem([=]() LOCKS_EXCLUDED(mutex_) {
    absl::MutexLock locker(&mutex_);
    return WorkItem::Result::kRunOptimization;
  });

  return true;
  */

 return true;
}

WorkItem::Result PoseGraph2D::LocalizeOKagvPosesWorkItem(
    const bool use_initial_pose, const int trajectory_id,
    const transform::Rigid3d initial_pose) {
  if (!use_initial_pose) return WorkItem::Result::kDoNotRunOptimization;

  //
  covariance_score_ = 0;
  constraint_builder_.SetMatchScore(covariance_score_);
  
  std::vector<SubmapId> submap_ids;
  submap_ids = FindPoseOfSubmapInTrajectory(initial_pose);

  for (size_t i = 0; i < submap_ids.size(); i++) {
    LOG(INFO) << "okagv submap trajectory_id "
              << submap_ids[i].trajectory_id;
    LOG(INFO) << "okagv submap submap_index " << submap_ids[i].submap_index;
    const NodeId last_node =
        std::prev(optimization_problem_->node_data().EndOfTrajectory(
                      current_localization_trajectory_id_))
            ->id;
    const SubmapId submap_found = submap_ids[i];

    //
    const transform::Rigid2d initial_pose_2d =
        transform::Project2D(initial_pose);

    const transform::Rigid2d initial_relative_pose =
        optimization_problem_->submap_data()
            .at(submap_found)
            .global_pose.inverse() *
        initial_pose_2d;

    add_relocalization_constraint = true;
    ComputeConstraintForRelocalization(last_node, submap_found,
                                       initial_relative_pose);
    add_relocalization_constraint = false;
  }

  // absl::SleepFor(absl::Milliseconds(1000));
  constraint_builder_.NotifyEndOfNode();  // okagv remove

  return WorkItem::Result::kRunOptimization;
}

std::vector<SubmapId> PoseGraph2D::FindPoseOfSubmapInTrajectory(
    const transform::Rigid3d& initial_pose) const {
  // find the nearist submap of initial_pose
  //double min_distance = INT_MAX;
  SubmapId nearist_submapId(current_map_trajectory_id_, 0);

  std::priority_queue<submap_sort> submap_sort_by_distance;

  for (const auto& submap_id_data :
       data_.submap_data.trajectory(current_map_trajectory_id_)) {
    // if(submap_id_data.id.trajectory_id != current_map_trajectory_id_)
    // continue;

    auto submap_data = GetSubmapDataUnderLock(submap_id_data.id);

    double distance =
        (submap_data.pose.inverse() * initial_pose).translation().norm();

    submap_sort_by_distance.emplace(distance, current_map_trajectory_id_,
                                    submap_id_data.id.submap_index);
    /*
    if(distance < min_distance)
    {
      min_distance = distance;
      nearist_submapId.submap_index = submap_id_data.id.submap_index;
    }
    */
  }

  /*
  std::vector<SubmapId> submap_ids;
  submap_ids.push_back(nearist_submapId);

  //check the prev of nearist submap
  SubmapId
  prev_nearist_submapId(current_map_trajectory_id_,nearist_submapId.submap_index
  - 1); if(data_.submap_data.Contains(prev_nearist_submapId))
  {
      submap_ids.push_back(prev_nearist_submapId);
  }

  //check the back of nearist submap
  SubmapId
  back_nearist_submapId(current_map_trajectory_id_,nearist_submapId.submap_index
  + 1); if(data_.submap_data.Contains(back_nearist_submapId))
  {
      submap_ids.push_back(back_nearist_submapId);
  }
  */

  std::vector<SubmapId> submap_ids;

  for (int i = 0; i < submap_sort_by_distance.size(); i++) {

    if(i == 3) break;
    SubmapId id(submap_sort_by_distance.top().trajectory_id,
                submap_sort_by_distance.top().index);
    submap_ids.push_back(id);
    //LOG(INFO) << "Localize trajectory " << id.trajectory_id << " submap index "
    //          << id.submap_index;
    submap_sort_by_distance.top().distance;
    submap_sort_by_distance.pop();
  }

  return submap_ids;
}

std::map<std::string, transform::Rigid3d> PoseGraph2D::GetLandmarkPoses()
    const {
  std::map<std::string, transform::Rigid3d> landmark_poses;
  absl::MutexLock locker(&mutex_);
  for (const auto& landmark : data_.landmark_nodes) {
    // Landmark without value has not been optimized yet.
    if (!landmark.second.global_landmark_pose.has_value()) continue;
    landmark_poses[landmark.first] =
        landmark.second.global_landmark_pose.value();

    if(landmark_poses[landmark.first].translation().x() == 0.0 &&
    landmark_poses[landmark.first].translation().y() == 0.0)
    {
        //LOG(INFO) << "okagv check landmark id " << landmark.first;
    }

  }
  return landmark_poses;
}

std::map<std::string, transform::Rigid3d> PoseGraph2D::GetLandmarkPosesWithId(int trajectory_id)
    const {
  std::map<std::string, transform::Rigid3d> landmark_poses;
  absl::MutexLock locker(&mutex_);
  for (const auto& landmark : data_.landmark_nodes) {
    
    if (landmark.second.landmark_observations.size() == 0 ||
        landmark.second.landmark_observations[0].trajectory_id !=
            trajectory_id) {
      continue;
    }
    // Landmark without value has not been optimized yet.
    if (!landmark.second.global_landmark_pose.has_value()) continue;
    landmark_poses[landmark.first] =
        landmark.second.global_landmark_pose.value();

  }
  return landmark_poses;
}

std::map<std::string, transform::Rigid3d> PoseGraph2D::GetLandmarkPosesAfterUpdate()
    const {
  std::map<std::string, transform::Rigid3d> landmark_poses;
  absl::MutexLock locker(&mutex_);
  for (const auto& landmark : data_.landmark_nodes) {
    // Landmark without value has not been optimized yet.
    if (!landmark.second.global_landmark_pose.has_value()) continue;
    landmark_poses[landmark.first] =
        landmark.second.global_landmark_pose.value();

    if(landmark_poses[landmark.first].translation().x() == 0.0 &&
    landmark_poses[landmark.first].translation().y() == 0.0)
    {
        //LOG(INFO) << "okagv check landmark id " << landmark.first;
    }

  }
  return landmark_poses;
}

void PoseGraph2D::SetLandmarkPose(const std::string& landmark_id,
                                  const transform::Rigid3d& global_pose,
                                  const bool frozen) {
    //LOG(INFO) << "okagv AddWorkItem SetLandmarkPose";
  AddWorkItem([=]() LOCKS_EXCLUDED(mutex_) {
    absl::MutexLock locker(&mutex_);
    data_.landmark_nodes[landmark_id].global_landmark_pose = global_pose;
    data_.landmark_nodes[landmark_id].frozen = frozen;
    return WorkItem::Result::kDoNotRunOptimization;
  });
}

sensor::MapByTime<sensor::ImuData> PoseGraph2D::GetImuData() const {
  absl::MutexLock locker(&mutex_);
  return optimization_problem_->imu_data();
}

sensor::MapByTime<sensor::ImuData> PoseGraph2D::GetImuDataAfterUpdate() const {
  absl::MutexLock locker(&mutex_);
  return optimization_problem_->imu_data();
}

sensor::MapByTime<sensor::OdometryData> PoseGraph2D::GetOdometryData() const {
  absl::MutexLock locker(&mutex_);
  return optimization_problem_->odometry_data();
}

std::map<std::string /* landmark ID */, PoseGraphInterface::LandmarkNode>
PoseGraph2D::GetLandmarkNodes() const {
  absl::MutexLock locker(&mutex_);
  return data_.landmark_nodes;
}

std::map<int, PoseGraphInterface::TrajectoryData>
PoseGraph2D::GetTrajectoryData() const {
  absl::MutexLock locker(&mutex_);
  return optimization_problem_->trajectory_data();
}

std::map<int, PoseGraphInterface::TrajectoryData>
PoseGraph2D::GetTrajectoryDataAfterUpdate() const {
  absl::MutexLock locker(&mutex_);

  return optimization_problem_->trajectory_data();

  /*
  std::map<int, PoseGraphInterface::TrajectoryData> first_trajectory_data_;
  auto all_data = optimization_problem_->trajectory_data();
  if (all_data.size() == 2) {
    first_trajectory_data_[0] = optimization_problem_->trajectory_data().at(0);
  } else {
    first_trajectory_data_[0] =
        optimization_problem_->trajectory_data().at(1001);
  }
  return first_trajectory_data_;
  */
}

sensor::MapByTime<sensor::FixedFramePoseData>
PoseGraph2D::GetFixedFramePoseData() const {
  absl::MutexLock locker(&mutex_);
  return optimization_problem_->fixed_frame_pose_data();
}

std::vector<PoseGraphInterface::Constraint> PoseGraph2D::constraints() const {
  std::vector<PoseGraphInterface::Constraint> result;
  absl::MutexLock locker(&mutex_);
  for (const Constraint& constraint : data_.constraints) {
    result.push_back(Constraint{
        constraint.submap_id, constraint.node_id,
        Constraint::Pose{constraint.pose.zbar_ij *
                             transform::Rigid3d::Rotation(
                                 data_.trajectory_nodes.at(constraint.node_id)
                                     .constant_data->gravity_alignment),
                         constraint.pose.translation_weight,
                         constraint.pose.rotation_weight},
        constraint.tag});
  }
  return result;
}

std::vector<PoseGraphInterface::Constraint>
PoseGraph2D::constraintsAfterUpdate() const {
  std::vector<PoseGraphInterface::Constraint> result;
  absl::MutexLock locker(&mutex_);

  int old_map_last_submap_index = -1;
  int old_map_last_node_index = -1;

  for (const Constraint& constraint : data_.constraints) {
    SubmapId submap_id(0, constraint.submap_id.submap_index);
    NodeId node_id(0, constraint.node_id.node_index);

    //LOG(INFO) << "okagv constraintsAfterUpdate " << constraint.submap_id
    //          << " " << constraint.node_id;

    if (constraint.submap_id.trajectory_id == 0 &&
        constraint.node_id.trajectory_id == 0) {
      // LOG(INFO) << "okagv constraintsAfterUpdate " <<
      // constraint.submap_id << " " << constraint.node_id;
      result.push_back(Constraint{
          constraint.submap_id, constraint.node_id,
          Constraint::Pose{constraint.pose.zbar_ij *
                               transform::Rigid3d::Rotation(
                                   data_.trajectory_nodes.at(constraint.node_id)
                                       .constant_data->gravity_alignment),
                           constraint.pose.translation_weight,
                           constraint.pose.rotation_weight},
          constraint.tag});

      if (old_map_last_submap_index < constraint.submap_id.submap_index) {
        old_map_last_submap_index = constraint.submap_id.submap_index;
        // LOG(INFO) << "okagv constraints last submap_index " <<
        // old_map_last_submap_index;
      }

      if (old_map_last_node_index < constraint.node_id.node_index) {
        old_map_last_node_index = constraint.node_id.node_index;
        // LOG(INFO) << "okagv constraints last node_index " <<
        // old_map_last_node_index;
      }

      continue;
    } else {
      if (constraint.submap_id.trajectory_id == 1001) {
        int submap_end_index =
            old_map_last_submap_index;  // data_.submap_data.EndOfTrajectory(0)->id.submap_index;
        submap_id.submap_index =
            constraint.submap_id.submap_index + submap_end_index + 1;
        // LOG(INFO) << "okagv 1001 submap_end_index " << submap_end_index;
      }

      if (constraint.node_id.trajectory_id == 1001) {
        int node_end_index =
            old_map_last_node_index;  // data_.trajectory_nodes.EndOfTrajectory(0)->id.node_index;
          //LOG(INFO) << "okagv 1001 node_end_index " << node_end_index;
        node_id.node_index = constraint.node_id.node_index + node_end_index + 2;

      }
    }

    // LOG(INFO) << "okagv 1001 constraintsAfterUpdate " << submap_id << " "
    // << node_id;
    result.push_back(Constraint{
        submap_id, node_id,
        Constraint::Pose{constraint.pose.zbar_ij *
                             transform::Rigid3d::Rotation(
                                 data_.trajectory_nodes.at(constraint.node_id)
                                     .constant_data->gravity_alignment),
                         constraint.pose.translation_weight,
                         constraint.pose.rotation_weight},
        constraint.tag});
  }
  return result;
}

// okagv
std::vector<PoseGraphInterface::Constraint> PoseGraph2D::constraintsWithId(int trajectory_id)
    const {
  std::vector<PoseGraphInterface::Constraint> result;
  absl::MutexLock locker(&mutex_);
  for (const Constraint& constraint : data_.constraints) {
    if(constraint.node_id.trajectory_id != constraint.submap_id.trajectory_id ||
       constraint.submap_id.trajectory_id != trajectory_id ||
       constraint.node_id.trajectory_id != trajectory_id) continue; 

    result.push_back(Constraint{
        constraint.submap_id, constraint.node_id,
        Constraint::Pose{constraint.pose.zbar_ij *
                             transform::Rigid3d::Rotation(
                                 data_.trajectory_nodes.at(constraint.node_id)
                                     .constant_data->gravity_alignment),
                         constraint.pose.translation_weight,
                         constraint.pose.rotation_weight},
        constraint.tag});
  }
  return result;
}

void PoseGraph2D::SetInitialTrajectoryPose(const int from_trajectory_id,
                                           const int to_trajectory_id,
                                           const transform::Rigid3d& pose,
                                           const common::Time time) {
  absl::MutexLock locker(&mutex_);
  data_.initial_trajectory_poses[from_trajectory_id] =
      InitialTrajectoryPose{to_trajectory_id, pose, time};
}

transform::Rigid3d PoseGraph2D::GetInterpolatedGlobalTrajectoryPose(
    const int trajectory_id, const common::Time time) const {
  CHECK_GT(data_.trajectory_nodes.SizeOfTrajectoryOrZero(trajectory_id), 0);
  const auto it = data_.trajectory_nodes.lower_bound(trajectory_id, time);
  if (it == data_.trajectory_nodes.BeginOfTrajectory(trajectory_id)) {
    return data_.trajectory_nodes.BeginOfTrajectory(trajectory_id)
        ->data.global_pose;
  }
  if (it == data_.trajectory_nodes.EndOfTrajectory(trajectory_id)) {
    return std::prev(data_.trajectory_nodes.EndOfTrajectory(trajectory_id))
        ->data.global_pose;
  }
  return transform::Interpolate(
             transform::TimestampedTransform{std::prev(it)->data.time(),
                                             std::prev(it)->data.global_pose},
             transform::TimestampedTransform{it->data.time(),
                                             it->data.global_pose},
             time)
      .transform;
}

transform::Rigid3d PoseGraph2D::GetLocalToGlobalTransform(
    const int trajectory_id) const {
  absl::MutexLock locker(&mutex_); //okagv remove test
  //LOG(INFO) << "okagv GetLocalToGlobalTransform";
  //LOG(INFO) << "okagv GetLocalToGlobalTransform 4";
  return ComputeLocalToGlobalTransform(data_.global_submap_poses_2d,
                                       trajectory_id);
}

std::vector<std::vector<int>> PoseGraph2D::GetConnectedTrajectories() const {
  absl::MutexLock locker(&mutex_);
  return data_.trajectory_connectivity_state.Components();
}

PoseGraphInterface::SubmapData PoseGraph2D::GetSubmapData(
    const SubmapId& submap_id) const {
  absl::MutexLock locker(&mutex_);
  return GetSubmapDataUnderLock(submap_id);
}

MapById<SubmapId, PoseGraphInterface::SubmapData>
PoseGraph2D::GetAllSubmapData() const {
  absl::MutexLock locker(&mutex_);
  return GetSubmapDataUnderLock();
}

MapById<SubmapId, PoseGraphInterface::SubmapData>
PoseGraph2D::GetAllSubmapDataAfterUpdate() const {
  absl::MutexLock locker(&mutex_);
  return GetSubmapDataAfterUpdateUnderLock();
}

MapById<SubmapId, PoseGraphInterface::SubmapPose>
PoseGraph2D::GetAllSubmapPoses() const {
  absl::MutexLock locker(&mutex_);
  MapById<SubmapId, SubmapPose> submap_poses;
  for (const auto& submap_id_data : data_.submap_data) {
    auto submap_data = GetSubmapDataUnderLock(submap_id_data.id);
    submap_poses.Insert(
        submap_id_data.id,
        PoseGraph::SubmapPose{submap_data.submap->num_range_data(),
                              submap_data.pose});
  }
  return submap_poses;
}

transform::Rigid3d PoseGraph2D::ComputeLocalToGlobalTransform(
    const MapById<SubmapId, optimization::SubmapSpec2D>& global_submap_poses,
    const int trajectory_id) const {
  auto begin_it = global_submap_poses.BeginOfTrajectory(trajectory_id);
  auto end_it = global_submap_poses.EndOfTrajectory(trajectory_id);
  if (begin_it == end_it) {
    const auto it = data_.initial_trajectory_poses.find(trajectory_id);
    if (it != data_.initial_trajectory_poses.end()) {
      return GetInterpolatedGlobalTrajectoryPose(it->second.to_trajectory_id,
                                                 it->second.time) *
             it->second.relative_pose;
    } else {
      return transform::Rigid3d::Identity();
    }
  }

  const SubmapId last_optimized_submap_id = std::prev(end_it)->id;
  // Accessing 'local_pose' in Submap is okay, since the member is const.
  //LOG(INFO) << "okagv data_.submap_data 21";
  //LOG(INFO) << "last_optimized_submap_id "
  //          << last_optimized_submap_id.trajectory_id << " , "
  //          << last_optimized_submap_id.submap_index;
  return transform::Embed3D(
             global_submap_poses.at(last_optimized_submap_id).global_pose) *
         data_.submap_data.at(last_optimized_submap_id)
             .submap->local_pose()
             .inverse();
}

PoseGraphInterface::SubmapData PoseGraph2D::GetSubmapDataUnderLock(
    const SubmapId& submap_id) const {
  const auto it = data_.submap_data.find(submap_id);
  if (it == data_.submap_data.end()) {
    return {};
  }

  auto submap = it->data.submap;
  if (data_.global_submap_poses_2d.Contains(submap_id)) {
    // We already have an optimized pose.
    return {submap,
            transform::Embed3D(
                data_.global_submap_poses_2d.at(submap_id).global_pose)};
  }
  // We have to extrapolate.
  //  LOG(INFO) << "okagv GetLocalToGlobalTransform 5";
  return {submap, ComputeLocalToGlobalTransform(data_.global_submap_poses_2d,
                                                submap_id.trajectory_id) *
                      submap->local_pose()};
}

PoseGraph2D::TrimmingHandle::TrimmingHandle(PoseGraph2D* const parent)
    : parent_(parent) {}

int PoseGraph2D::TrimmingHandle::num_submaps(const int trajectory_id) const {
  const auto& submap_data = parent_->optimization_problem_->submap_data();
  return submap_data.SizeOfTrajectoryOrZero(trajectory_id);
}

MapById<SubmapId, PoseGraphInterface::SubmapData>
PoseGraph2D::TrimmingHandle::GetOptimizedSubmapData() const {
  MapById<SubmapId, PoseGraphInterface::SubmapData> submaps;
  for (const auto& submap_id_data : parent_->data_.submap_data) {
    if (submap_id_data.data.state != SubmapState::kFinished ||
        !parent_->data_.global_submap_poses_2d.Contains(submap_id_data.id)) {
      continue;
    }
    submaps.Insert(
        submap_id_data.id,
        SubmapData{submap_id_data.data.submap,
                   transform::Embed3D(parent_->data_.global_submap_poses_2d
                                          .at(submap_id_data.id)
                                          .global_pose)});
  }
  return submaps;
}

std::vector<SubmapId> PoseGraph2D::TrimmingHandle::GetSubmapIds(
    int trajectory_id) const {
  std::vector<SubmapId> submap_ids;
  const auto& submap_data = parent_->optimization_problem_->submap_data();
  for (const auto& it : submap_data.trajectory(trajectory_id)) {
    submap_ids.push_back(it.id);
  }
  return submap_ids;
}

//okagv
std::vector<NodeId> PoseGraph2D::TrimmingHandle::GetNodeIds(
  int trajectory_id) const{
    std::vector<NodeId> node_ids;
    const auto& node_data = parent_->optimization_problem_->node_data();
    for(const auto& it : node_data.trajectory(trajectory_id)){
      node_ids.push_back(it.id);
    }
    return node_ids;
  }

const MapById<NodeId, TrajectoryNode>&
PoseGraph2D::TrimmingHandle::GetTrajectoryNodes() const {
  return parent_->data_.trajectory_nodes;
}

const std::vector<PoseGraphInterface::Constraint>&
PoseGraph2D::TrimmingHandle::GetConstraints() const {
  return parent_->data_.constraints;
}

bool PoseGraph2D::TrimmingHandle::IsFinished(const int trajectory_id) const {
  return parent_->IsTrajectoryFinished(trajectory_id);
}

void PoseGraph2D::TrimmingHandle::SetTrajectoryState(int trajectory_id,
                                                     TrajectoryState state) {
  parent_->data_.trajectories_state[trajectory_id].state = state;
}

void PoseGraph2D::TrimmingHandle::TrimSubmap(const SubmapId& submap_id) {

  // okagv
  /*
  if (parent_->current_trajectory_type_ == TrajectoryType::SLAM) {
    CHECK(parent_->data_.submap_data.at(submap_id).state ==
          SubmapState::kFinished);

    std::vector<Constraint> constraints;
    for (const Constraint& constraint : parent_->data_.constraints) {
      if (constraint.submap_id == submap_id &&
          constraint.tag == Constraint::Tag::INTRA_SUBMAP) {
        continue;
      } else {
        constraints.push_back(constraint);
      }
    }
    parent_->data_.constraints = std::move(constraints);

    return;
  }
  */

   /**********************************************************************
    * old way --only keep 3 submap
    * ********************************************************************/

  // TODO(hrapp): We have to make sure that the trajectory has been finished
  // if we want to delete the last submaps.
  CHECK(parent_->data_.submap_data.at(submap_id).state ==
        SubmapState::kFinished);

  // Compile all nodes that are still INTRA_SUBMAP constrained once the submap
  // with 'submap_id' is gone.
  //LOG(INFO) << "okagv TrimSubmap 0";
  std::set<NodeId> nodes_to_retain;
  for (const Constraint& constraint : parent_->data_.constraints) {
    if (constraint.tag == Constraint::Tag::INTRA_SUBMAP &&
        constraint.submap_id != submap_id) {
      nodes_to_retain.insert(constraint.node_id);
    }
  }

  // Remove all 'data_.constraints' related to 'submap_id'.
  //LOG(INFO) << "okagv TrimSubmap 1";
  std::set<NodeId> nodes_to_remove;
  {
    std::vector<Constraint> constraints;
    for (const Constraint& constraint : parent_->data_.constraints) {

      if (constraint.submap_id == submap_id) {
              
        if (constraint.tag == Constraint::Tag::INTRA_SUBMAP &&
            nodes_to_retain.count(constraint.node_id) == 0) {
          // This node will no longer be INTRA_SUBMAP contrained and has to be
          // removed.
          nodes_to_remove.insert(constraint.node_id);
        }
      } else {
        constraints.push_back(constraint);
      }
    }
    parent_->data_.constraints = std::move(constraints);
  }

  // Remove all 'data_.constraints' related to 'nodes_to_remove'.
  //LOG(INFO) << "okagv TrimSubmap 2";
  {
    std::vector<Constraint> constraints;
    for (const Constraint& constraint : parent_->data_.constraints) {
      if (nodes_to_remove.count(constraint.node_id) == 0) {
        constraints.push_back(constraint);
      }
    }
    parent_->data_.constraints = std::move(constraints);
  }

  // Mark the submap with 'submap_id' as trimmed and remove its data.
  //LOG(INFO) << "okagv TrimSubmap 3";
  CHECK(parent_->data_.submap_data.at(submap_id).state ==
        SubmapState::kFinished);
            //LOG(INFO) << "okagv TrimSubmap " << submap_id;
  parent_->data_.submap_data.Trim(submap_id);
  //LOG(INFO) << "okagv TrimSubmap 3-2";
  parent_->constraint_builder_.DeleteScanMatcher(submap_id);
  //LOG(INFO) << "okagv TrimSubmap 3-3";
  parent_->optimization_problem_->TrimSubmap(submap_id);

  //okagv
  // LOG(INFO) << "okagv TrimSubmap 4";

   for (const auto& it : parent_->data_.global_submap_poses_2d) {
     if (it.id == submap_id) {
       parent_->data_.global_submap_poses_2d.Trim(submap_id);
     }
   }

  // We have one submap less, update the gauge metrics.
  kDeletedSubmapsMetric->Increment();
  if (parent_->IsTrajectoryFrozen(submap_id.trajectory_id)) {
    kFrozenSubmapsMetric->Decrement();
  } else {
    kActiveSubmapsMetric->Decrement();
  }

  // Remove the 'nodes_to_remove' from the pose graph and the optimization
  // problem.
  //LOG(INFO) << "okagv TrimSubmap 5";
  for (const NodeId& node_id : nodes_to_remove) {
    parent_->data_.trajectory_nodes.Trim(node_id);

    parent_->optimization_problem_->TrimTrajectoryNode(node_id);
  }
  //LOG(INFO) << "okagv TrimSubmap 6";

}

void PoseGraph2D::TrimmingHandle::TrimNode(const NodeId& node_id) {

}

MapById<SubmapId, PoseGraphInterface::SubmapData>
PoseGraph2D::GetSubmapDataUnderLock() const {
  MapById<SubmapId, PoseGraphInterface::SubmapData> submaps;
  for (const auto& submap_id_data : data_.submap_data) {
    submaps.Insert(submap_id_data.id,
                   GetSubmapDataUnderLock(submap_id_data.id));
  }
  return submaps;
}

MapById<SubmapId, PoseGraphInterface::SubmapData>
PoseGraph2D::GetSubmapDataAfterUpdateUnderLock() const {
  MapById<SubmapId, PoseGraphInterface::SubmapData> submaps;
  int old_map_last_submap_index = 0;

  for (const auto& submap_id_data : data_.submap_data) {
    if (submap_id_data.id.trajectory_id == 0) {
      submaps.Insert(submap_id_data.id,
                     GetSubmapDataUnderLock(submap_id_data.id));
      old_map_last_submap_index = submap_id_data.id.submap_index;
    } else if (submap_id_data.id.trajectory_id == 1001) {
      // int loaded_map_submap_count =
      //    data_.submap_data.EndOfTrajectory(0)->id.submap_index;
      //LOG(INFO) << "okagv old_map_last_submap_index "
      //          << old_map_last_submap_index;
      //LOG(INFO) << "okagv submap_id_data "
      //          << submap_id_data.id.submap_index +  old_map_last_submap_index +
      //                 1;
      SubmapId id(
          0, submap_id_data.id.submap_index + old_map_last_submap_index + 1);
      submaps.Insert(id, GetSubmapDataUnderLock(submap_id_data.id));
    }
  }
  return submaps;
}

void PoseGraph2D::SetGlobalSlamOptimizationCallback(
    PoseGraphInterface::GlobalSlamOptimizationCallback callback) {
  global_slam_optimization_callback_ = callback;
}

void PoseGraph2D::RegisterMetrics(metrics::FamilyFactory* family_factory) {
  auto* latency = family_factory->NewGaugeFamily(
      "mapping_2d_pose_graph_work_queue_delay",
      "Age of the oldest entry in the work queue in seconds");
  kWorkQueueDelayMetric = latency->Add({});
  auto* queue_size =
      family_factory->NewGaugeFamily("mapping_2d_pose_graph_work_queue_size",
                                     "Number of items in the work queue");
  kWorkQueueSizeMetric = queue_size->Add({});
  auto* constraints = family_factory->NewGaugeFamily(
      "mapping_2d_pose_graph_constraints",
      "Current number of constraints in the pose graph");
  kConstraintsDifferentTrajectoryMetric =
      constraints->Add({{"tag", "inter_submap"}, {"trajectory", "different"}});
  kConstraintsSameTrajectoryMetric =
      constraints->Add({{"tag", "inter_submap"}, {"trajectory", "same"}});
  auto* submaps = family_factory->NewGaugeFamily(
      "mapping_2d_pose_graph_submaps", "Number of submaps in the pose graph.");
  kActiveSubmapsMetric = submaps->Add({{"state", "active"}});
  kFrozenSubmapsMetric = submaps->Add({{"state", "frozen"}});
  kDeletedSubmapsMetric = submaps->Add({{"state", "deleted"}});
}

// okagv
bool PoseGraph2D::IsTrajectoryExist(int trajectory_id) const {
  absl::MutexLock locker(&mutex_);
  auto it = data_.trajectories_state.find(trajectory_id);
  if (it == data_.trajectories_state.end()) {
    LOG(INFO) << "Find non-existing trajectory_id: "
                 << trajectory_id;
    return false;
  }
  else
  {
    return true;
  }
  
}
// okagv
void PoseGraph2D::GetCovarianceScore(double& score, bool& is_update) {
  absl::MutexLock locker(&mutex_);  // okagv test
  // LOG(INFO) << "okagv GetCovarianceScore";
  // LOG(INFO) << "okagv poseGraph2D covariance_score_ " <<
  // covariance_score_;
  
  switch (current_trajectory_type_) {
    case TrajectoryType::RELOCALIZAION:
      //LOG(INFO) << "okagv current type is relocalization";
      covariance_score_ = constraint_builder_.GetRelocalizationMatchScore();
      //LOG(INFO) << "okagv GetRelocalizationMatchScore " << covariance_score_;
      if(process_work_queue == false) covariance_score_ = 0.0;
      break;
    case TrajectoryType::NAVIGATION:
      //LOG(INFO) << "okagv current type is navigation";
      constraint_builder_.GetMatchScore(covariance_score_,is_update_score);
      //LOG(INFO) << "okagv GetMatchScore " << covariance_score_;
      break;
    default:
      //LOG(INFO) << "okagv current type is others";
      covariance_score_ = 0.0;
      //LOG(INFO) << "okagv covariance_score_ " << covariance_score_;
      break;
  }

  const auto wall_time = std::chrono::steady_clock::now();

  if (covariance_score_ == 0.0) {
    last_covariance_score_ = 0.0;
    last_decrease_covariance_score_ = 0.0;
    // absl::SleepFor(absl::Milliseconds(1000));
  } else if (last_covariance_score_ != covariance_score_) {
    last_covariance_score_ = covariance_score_;
    last_decrease_covariance_score_ = covariance_score_;
    // LOG(INFO) << "okagv covariance_score_ update, value is "
    //          << covariance_score_;
    last_wall_time_ = wall_time;
  } else if (last_wall_time_.has_value() &&
             common::ToSeconds(wall_time - last_wall_time_.value()) > 1.0) {
    last_wall_time_ = wall_time;

    if (!last_record_node_global_pose.IsValid()) {
      last_record_node_global_pose = current_node_global_pose;
    } else {
      double distance = (current_node_global_pose.translation() -
                         last_record_node_global_pose.translation())
                            .norm();
      //LOG(INFO) << "okagv GetCovarianceScore distance " << distance;
      if (distance > 0.1) {
        last_decrease_covariance_score_ -=
            0.1 / options_.optimize_every_n_nodes();
      }
    }

    last_record_node_global_pose = current_node_global_pose;
    // LOG(INFO) << "okagv miss match time " <<
    // last_decrease_covariance_score_;
  }

  // use for justify if can be used for updating map
  if (covariance_score_ > 0) {
    if (!last_stable_wall_time_.has_value()) {
      stable_localization_count++;
      last_stable_wall_time_ = wall_time;
      last_stable_covariance_score_ = covariance_score_;
    }

    //
    // LOG(INFO) << "okagv last_stable_wall_time_ " <<
    // last_stable_wall_time_; LOG(INFO) << "okagv intercal " <<
    // common::ToSeconds(wall_time - last_stable_wall_time_.value()); LOG(INFO)
    // << "last_covariance_score_ " << last_covariance_score_; LOG(INFO) <<
    // "covariance_score_ " << covariance_score_;

    if (last_stable_wall_time_.has_value() &&
        common::ToSeconds(wall_time - last_stable_wall_time_.value()) >
            options_.local_constraint_search_after_n_seconds() &&
        last_stable_covariance_score_ != covariance_score_) {
      stable_localization_count++;
      last_stable_wall_time_ = wall_time;
      last_stable_covariance_score_ = covariance_score_;
    }
  }
  // LOG(INFO) << "okagv stable_localization_count " <<
  // stable_localization_count;
  score = last_decrease_covariance_score_;
  is_update = is_update_score;
  constraint_builder_.SetUpdateState(false);

  return;
}

// okagv
  void PoseGraph2D::SetCovarianceScore(double score) {
    
    constraint_builder_.SetMatchScore(score);

    /*
      if (score == 0.0) {
        TrimmingHandle trimming_handle(this);
        // TODO(gaschler): Consider directly deleting from data_, which may be
        // more complete.

        auto submap_ids =
            trimming_handle.GetSubmapIds(current_localization_trajectory_id_);

        if(submap_ids.size() == 0) return;

        for (auto &submap_id : submap_ids) {
          InternalSubmapData& finished_submap_data =
              data_.submap_data.at(submap_id);
          finished_submap_data.state = SubmapState::kFinished;

          trimming_handle.TrimSubmap(submap_id);
          // okagv
          if(data_.global_submap_poses_2d.Contains(submap_id))
          {
             data_.global_submap_poses_2d.Trim(submap_id);
          }

        }

        absl::SleepFor(absl::Milliseconds(200));

        AddWorkItem([=]() LOCKS_EXCLUDED(mutex_) {
          absl::MutexLock locker(&mutex_);
          return WorkItem::Result::kRunOptimization;
        });

        is_robot_hijack = true;
        LOG(INFO) << "Robot is hijack";
        covariance_score_ = score;
      }
      */
  }

// okagv
void PoseGraph2D::SetTrajectoryState(int trajectory_id,
                                      TrajectoryState state) {
  data_.trajectories_state[trajectory_id].state = state;
}

// okagv
void PoseGraph2D::SetWorkingTrajectoryType(uint8_t type) {
  current_trajectory_type_ = TrajectoryType(type);

  double init_value = 0;

  switch (current_trajectory_type_) {
    case TrajectoryType::RELOCALIZAION:
      constraint_builder_.SetRelocalizationMatchScore(init_value);
      break;
    case TrajectoryType::NAVIGATION:
      constraint_builder_.SetMatchScore(init_value);
      break;
    default:
      break;
  }
}

// okagv
uint8_t PoseGraph2D::GetWorkingTrajectoryType()
{
  return uint8_t(current_trajectory_type_);
}

void PoseGraph2D::SetConstraintBuilderMinScore(double score)
{
  options_.mutable_constraint_builder_options()->set_min_score(score);
}

void PoseGraph2D::TrimSubmapTimer() {
  // const auto predicate = [this]() EXCLUSIVE_LOCKS_REQUIRED(trimmer_mutex_) {
  //  return start_check_and_trim_submap_;
  //};
  for (;;) {
    // LOG(INFO) << "okagv TrimSubmapTimer 1";
    // absl::MutexLock locker(&trimmer_mutex_);
    // LOG(INFO) << "okagv TrimSubmapTimer 2";
    // trimmer_mutex_.Await(absl::Condition(&predicate));
    //LOG(INFO) << "okagv check and Trim the overlaping submaps";
    // LOG(INFO) << "okagv CheckRemovedSubmapId";
    {
      absl::MutexLock locker(&mutex_);

      TrimmingHandle trimming_handle(this);
      for (auto& trimmer : trimmers_) {
        // okagv
        
        if (current_trajectory_type_ == TrajectoryType::NAVIGATION &&
            stable_localization_count < 5)
          continue;  // okagv
          

        // LOG(INFO) << "okagv CheckRemovedSubmapId";
        trimmer->CheckRemovedSubmapId(&trimming_handle);
      }
    }
    // trimmers_.erase(
    //     std::remove_if(trimmers_.begin(), trimmers_.end(),
    //                    [](std::unique_ptr<PoseGraphTrimmer>& trimmer) {
    //                      return trimmer->IsFinished();
    //                    }),
    //     trimmers_.end());

    // start_check_and_trim_submap_ = false;
    absl::SleepFor(absl::Seconds(5));
  }

  return;
}

// okagv
void PoseGraph2D::StopDrainWorkQueue() {
    is_start_clear_work_queue = true;
}
// okagv
void PoseGraph2D::StartDrainWorkQueue() {
    is_start_clear_work_queue = false;
}
// okagv
void PoseGraph2D::SetThreadPoolState(mapping::PoseGraphInterface::ThreadPoolState type) {
   current_thread_pool_state_ = type;
}
// okagv
mapping::PoseGraphInterface::ThreadPoolState PoseGraph2D::GetThreadPoolState() {
  return current_thread_pool_state_;
}


//okagv
void PoseGraph2D::SetPoseGraphOption(proto::PoseGraphOptions& option_reset)
{
  //LOG(INFO) << "okagv SetPoseGraphOption PoseGraph2D";
  //LOG(INFO) << "okagv option_reset.optimize_every_n_nodes() " << option_reset.optimize_every_n_nodes();
  options_ = option_reset;
  //LOG(INFO) << "okagv options_.optimize_every_n_nodes() " << options_.optimize_every_n_nodes();
  //LOG(INFO) << "okagv " << options_.constraint_builder_options().min_score();

  constraint_builder_.SetConstraintBuilderOptions(*options_.mutable_constraint_builder_options());

}

}  // namespace mapping
}  // namespace cartographer
