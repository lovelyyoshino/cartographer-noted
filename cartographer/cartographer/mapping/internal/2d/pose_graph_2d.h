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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_POSE_GRAPH_2D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_POSE_GRAPH_2D_H_

#include <deque>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <vector>
#include <queue>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "absl/container/flat_hash_map.h"
#include "absl/synchronization/mutex.h"
#include "cartographer/common/fixed_ratio_sampler.h"
#include "cartographer/common/thread_pool.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/internal/constraints/constraint_builder_2d.h"
#include "cartographer/mapping/internal/optimization/optimization_problem_2d.h"
#include "cartographer/mapping/internal/trajectory_connectivity_state.h"
#include "cartographer/mapping/internal/work_queue.h"
#include "cartographer/mapping/pose_graph.h"
#include "cartographer/mapping/pose_graph_data.h"
#include "cartographer/mapping/pose_graph_trimmer.h"
#include "cartographer/mapping/value_conversion_tables.h"
#include "cartographer/metrics/family_factory.h"
#include "cartographer/sensor/fixed_frame_pose_data.h"
#include "cartographer/sensor/landmark_data.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"

#include "cartographer/mapping/map_builder_interface.h"

// okagv
#include <thread>

namespace cartographer {
namespace mapping {

// Implements the loop closure method called Sparse Pose Adjustment (SPA) from
// Konolige, Kurt, et al. "Efficient sparse pose adjustment for 2d mapping."
// Intelligent Robots and Systems (IROS), 2010 IEEE/RSJ International Conference
// on (pp. 22--29). IEEE, 2010.
//
// It is extended for submapping:
// Each node has been matched against one or more submaps (adding a constraint
// for each match), both poses of nodes and of submaps are to be optimized.
// All constraints are between a submap i and a node j.

using TrajectoryType = cartographer::mapping::MapBuilderInterface::TrajectoryType; 

struct submap_sort
{   
      double distance;
      int trajectory_id;
      int index; //submap or node

      submap_sort(double distance_value, int trajectory_id_value, int index_value)
      {
              distance = distance_value;
              trajectory_id = trajectory_id_value;
              index = index_value;
      }

      bool operator<(const submap_sort& other) const
      {
              return distance > other.distance;
      }
};


class PoseGraph2D : public PoseGraph {
public:
    PoseGraph2D(
            proto::PoseGraphOptions& options,
            std::unique_ptr<optimization::OptimizationProblem2D> optimization_problem,
            common::ThreadPool* thread_pool);
    ~PoseGraph2D() override;

    PoseGraph2D(const PoseGraph2D&) = delete;
    PoseGraph2D& operator=(const PoseGraph2D&) = delete;

    // Adds a new node with 'constant_data'. Its 'constant_data->local_pose' was
    // determined by scan matching against 'insertion_submaps.front()' and the
    // node data was inserted into the 'insertion_submaps'. If
    // 'insertion_submaps.front().finished()' is 'true', data was inserted into
    // this submap for the last time.
    NodeId AddNode(
            std::shared_ptr<const TrajectoryNode::Data> constant_data,
            int trajectory_id,
            const std::vector<std::shared_ptr<const Submap2D>>& insertion_submaps)
    LOCKS_EXCLUDED(mutex_);

    void AddImuData(int trajectory_id, const sensor::ImuData& imu_data) override
    LOCKS_EXCLUDED(mutex_);
    void AddOdometryData(int trajectory_id,
                         const sensor::OdometryData& odometry_data) override
    LOCKS_EXCLUDED(mutex_);
    void AddFixedFramePoseData(
            int trajectory_id,
            const sensor::FixedFramePoseData& fixed_frame_pose_data) override
    LOCKS_EXCLUDED(mutex_);
    void AddLandmarkData(int trajectory_id,
                         const sensor::LandmarkData& landmark_data) override
    LOCKS_EXCLUDED(mutex_);

    void DeleteTrajectory(int trajectory_id) override;
    void FinishTrajectory(int trajectory_id) override;
    bool IsTrajectoryFinished(int trajectory_id) const override
    EXCLUSIVE_LOCKS_REQUIRED(mutex_);
    void FreezeTrajectory(int trajectory_id) override;
    bool IsTrajectoryFrozen(int trajectory_id) const override
    EXCLUSIVE_LOCKS_REQUIRED(mutex_);
    void AddSubmapFromProto(const transform::Rigid3d& global_submap_pose,
                            const proto::Submap& submap) override;
    void AddNodeFromProto(const transform::Rigid3d& global_pose,
                          const proto::Node& node) override;
    void SetTrajectoryDataFromProto(const proto::TrajectoryData& data) override;
    void AddNodeToSubmap(const NodeId& node_id,
                         const SubmapId& submap_id) override;
    void AddSerializedConstraints(
            const std::vector<Constraint>& constraints) override;
    void AddTrimmer(std::unique_ptr<PoseGraphTrimmer> trimmer) override;
    void RunFinalOptimization() override;
    std::vector<std::vector<int>> GetConnectedTrajectories() const override
    LOCKS_EXCLUDED(mutex_);
    PoseGraphInterface::SubmapData GetSubmapData(const SubmapId& submap_id) const
    LOCKS_EXCLUDED(mutex_) override;
    MapById<SubmapId, PoseGraphInterface::SubmapData> GetAllSubmapData() const
    LOCKS_EXCLUDED(mutex_) override;
    MapById<SubmapId, SubmapPose> GetAllSubmapPoses() const
    LOCKS_EXCLUDED(mutex_) override;
    transform::Rigid3d GetLocalToGlobalTransform(int trajectory_id) const
    LOCKS_EXCLUDED(mutex_) override;
    MapById<NodeId, TrajectoryNode> GetTrajectoryNodes() const override
    LOCKS_EXCLUDED(mutex_);
    MapById<NodeId, TrajectoryNodePose> GetTrajectoryNodePoses() const override
    LOCKS_EXCLUDED(mutex_);
    std::map<int, TrajectoryState> GetTrajectoryStates() const override
    LOCKS_EXCLUDED(mutex_);
    std::map<std::string, transform::Rigid3d> GetLandmarkPoses() const override
    LOCKS_EXCLUDED(mutex_);
    //okagv
    std::map<std::string, transform::Rigid3d> GetLandmarkPosesWithId(int trajectory_id) const override
    LOCKS_EXCLUDED(mutex_);
    //okagv
    MapById<SubmapId, PoseGraphInterface::SubmapData> GetAllSubmapDataAfterUpdate() const
    LOCKS_EXCLUDED(mutex_) override;
    //okagv
    MapById<NodeId, TrajectoryNode> GetTrajectoryNodesAfterUpdate() const override
    LOCKS_EXCLUDED(mutex_); 
    //okagv
    std::map<std::string, transform::Rigid3d> GetLandmarkPosesAfterUpdate() const override
    LOCKS_EXCLUDED(mutex_);

    void SetLandmarkPose(const std::string& landmark_id,
                         const transform::Rigid3d& global_pose,
                         const bool frozen = false) override
    LOCKS_EXCLUDED(mutex_);
    sensor::MapByTime<sensor::ImuData> GetImuData() const override
    LOCKS_EXCLUDED(mutex_);
    sensor::MapByTime<sensor::OdometryData> GetOdometryData() const override
    LOCKS_EXCLUDED(mutex_);
    sensor::MapByTime<sensor::FixedFramePoseData> GetFixedFramePoseData()
    const override LOCKS_EXCLUDED(mutex_);
    std::map<std::string /* landmark ID */, PoseGraph::LandmarkNode>
    GetLandmarkNodes() const override LOCKS_EXCLUDED(mutex_);
    std::map<int, TrajectoryData> GetTrajectoryData() const override
    LOCKS_EXCLUDED(mutex_);
    std::vector<Constraint> constraints() const override LOCKS_EXCLUDED(mutex_);
    //okagv
    std::vector<Constraint> constraintsWithId(int trajectory_id) const override LOCKS_EXCLUDED(mutex_);
    //okagv
        std::map<int, TrajectoryData> GetTrajectoryDataAfterUpdate() const override
    LOCKS_EXCLUDED(mutex_);
    //okagv
    std::vector<Constraint> constraintsAfterUpdate() const override LOCKS_EXCLUDED(mutex_);

    void SetInitialTrajectoryPose(int from_trajectory_id, int to_trajectory_id,
                                  const transform::Rigid3d& pose,
                                  const common::Time time) override
    LOCKS_EXCLUDED(mutex_);
    void SetGlobalSlamOptimizationCallback(
            PoseGraphInterface::GlobalSlamOptimizationCallback callback) override;
    transform::Rigid3d GetInterpolatedGlobalTrajectoryPose(
            int trajectory_id, const common::Time time) const
    EXCLUSIVE_LOCKS_REQUIRED(mutex_);

    static void RegisterMetrics(metrics::FamilyFactory* family_factory);

    //okagv
    OKagvOrder GetOKagv_Order() const override
    LOCKS_EXCLUDED(mutex_);

    void SetOKagv_Order(const OKagvOrder& order) override
    LOCKS_EXCLUDED(mutex_);

    //okagv
    bool LocalizeOKagvPoses(const bool use_initial_pose,
                            const int tarjectory_id,
                            const transform::Rigid3d initial_pose) override;

     OKagvFeedback GetOKagv_Feedback() const override
    LOCKS_EXCLUDED(mutex_);

     void SetOKagv_Feedback(const OKagvFeedback& feedback) override
    LOCKS_EXCLUDED(mutex_);

    //okagv
    std::vector<SubmapId> FindPoseOfSubmapInTrajectory(
            const transform::Rigid3d& initial_pose) const;
 
    //okagv
    bool IsTrajectoryExist(int trajectory_id) const override;

    //okagv
    void GetCovarianceScore(double& score, bool& is_update) override;

    //okagv 
    void SetCovarianceScore(double score) override;

    //okagv
    void SetTrajectoryState(int trajectory_id, TrajectoryState state) override;  

    //okagv
    void SetWorkingTrajectoryType(uint8_t type) override;

      //okagv
    uint8_t GetWorkingTrajectoryType() override;

    //okagv
    void SetConstraintBuilderMinScore(double score) override;

    //okagv
    void TrimSubmapTimer();

    //okagv
    sensor::MapByTime<sensor::ImuData> GetImuDataAfterUpdate() const override
    LOCKS_EXCLUDED(mutex_);

    // okagv
    void StopDrainWorkQueue() override;
    // okagv
    void StartDrainWorkQueue() override;
    // okagv
    void SetThreadPoolState(mapping::PoseGraphInterface::ThreadPoolState type) override;
    // okagv
    mapping::PoseGraphInterface::ThreadPoolState GetThreadPoolState() override;

    //okagv
    TrajectoryType current_trajectory_type_ = TrajectoryType::IDLE;

    //okagv
    void SetPoseGraphOption(proto::PoseGraphOptions& option_reset) override;

   private:
    MapById<SubmapId, PoseGraphInterface::SubmapData> GetSubmapDataUnderLock()
    const EXCLUSIVE_LOCKS_REQUIRED(mutex_);

    //okagv
    MapById<SubmapId, PoseGraphInterface::SubmapData> GetSubmapDataAfterUpdateUnderLock()
    const EXCLUSIVE_LOCKS_REQUIRED(mutex_);

    // Handles a new work item.
    void AddWorkItem(const std::function<WorkItem::Result()>& work_item)
    LOCKS_EXCLUDED(mutex_) LOCKS_EXCLUDED(work_queue_mutex_);

    // Adds connectivity and sampler for a trajectory if it does not exist.
    void AddTrajectoryIfNeeded(int trajectory_id)
    EXCLUSIVE_LOCKS_REQUIRED(mutex_);

    // Appends the new node and submap (if needed) to the internal data
    // structures.
    NodeId AppendNode(
            std::shared_ptr<const TrajectoryNode::Data> constant_data,
            int trajectory_id,
            const std::vector<std::shared_ptr<const Submap2D>>& insertion_submaps,
            const transform::Rigid3d& optimized_pose) LOCKS_EXCLUDED(mutex_);

    // Grows the optimization problem to have an entry for every element of
    // 'insertion_submaps'. Returns the IDs for the 'insertion_submaps'.
    std::vector<SubmapId> InitializeGlobalSubmapPoses(
            int trajectory_id, const common::Time time,
            const std::vector<std::shared_ptr<const Submap2D>>& insertion_submaps)
    EXCLUSIVE_LOCKS_REQUIRED(mutex_);

    // Adds constraints for a node, and starts scan matching in the background.
    WorkItem::Result ComputeConstraintsForNode(
            const NodeId& node_id,
            std::vector<std::shared_ptr<const Submap2D>> insertion_submaps,
            bool newly_finished_submap) LOCKS_EXCLUDED(mutex_);

    // Computes constraints for a node and submap pair.
    void ComputeConstraint(const NodeId& node_id, const SubmapId& submap_id)
    LOCKS_EXCLUDED(mutex_);

    //okagv
    void ComputeConstraintForRelocalization(const NodeId& node_id, 
                                            const SubmapId& submap_id,
                                            const transform::Rigid2d initial_relative_pose)
    LOCKS_EXCLUDED(mutex_);

    // Deletes trajectories waiting for deletion. Must not be called during
    // constraint search.
    void DeleteTrajectoriesIfNeeded() EXCLUSIVE_LOCKS_REQUIRED(mutex_);

    //okagv
    void DeleteTrajectoriesIfNeeded(int trajectory_id) EXCLUSIVE_LOCKS_REQUIRED(mutex_);

    // Runs the optimization, executes the trimmers and processes the work queue.
    void HandleWorkQueue(const constraints::ConstraintBuilder2D::Result& result)
    LOCKS_EXCLUDED(mutex_) LOCKS_EXCLUDED(work_queue_mutex_) LOCKS_EXCLUDED(trimmer_mutex_);

    // Process pending tasks in the work queue on the calling thread, until the
    // queue is either empty or an optimization is required.
    void DrainWorkQueue() LOCKS_EXCLUDED(mutex_)
    LOCKS_EXCLUDED(work_queue_mutex_);

    // Waits until we caught up (i.e. nothing is waiting to be scheduled), and
    // all computations have finished.
    void WaitForAllComputations() LOCKS_EXCLUDED(mutex_)
    LOCKS_EXCLUDED(work_queue_mutex_);

    void WaitForAllComputationsThenFinishTrajectory(int trajectory_id) LOCKS_EXCLUDED(mutex_)
    LOCKS_EXCLUDED(work_queue_mutex_);
 
    void WaitForAllComputationsThenDeleteTrajectory(int trajectory_id) LOCKS_EXCLUDED(mutex_)
    LOCKS_EXCLUDED(work_queue_mutex_);

    // Runs the optimization. Callers have to make sure, that there is only one
    // optimization being run at a time.
    void RunOptimization() LOCKS_EXCLUDED(mutex_);

    bool CanAddWorkItemModifying(int trajectory_id)
    EXCLUSIVE_LOCKS_REQUIRED(mutex_);

    // Computes the local to global map frame transform based on the given
    // 'global_submap_poses'.
    transform::Rigid3d ComputeLocalToGlobalTransform(
            const MapById<SubmapId, optimization::SubmapSpec2D>& global_submap_poses,
            int trajectory_id) const EXCLUSIVE_LOCKS_REQUIRED(mutex_);

    SubmapData GetSubmapDataUnderLock(const SubmapId& submap_id) const
    EXCLUSIVE_LOCKS_REQUIRED(mutex_);

    common::Time GetLatestNodeTime(const NodeId& node_id,
                                   const SubmapId& submap_id) const
    EXCLUSIVE_LOCKS_REQUIRED(mutex_);

    // Updates the trajectory connectivity structure with a new constraint.
    void UpdateTrajectoryConnectivity(const Constraint& constraint)
    EXCLUSIVE_LOCKS_REQUIRED(mutex_);

    proto::PoseGraphOptions options_; //okagv const to non-const
    GlobalSlamOptimizationCallback global_slam_optimization_callback_;
    mutable absl::Mutex mutex_;
    absl::Mutex work_queue_mutex_;

    //okagv
    //mutable absl::Mutex delete_mutex_;
 
    WorkItem::Result LocalizeOKagvPosesWorkItem(const bool use_initial_pose,
                            const int tarjectory_id,
                            const transform::Rigid3d initial_pose) LOCKS_EXCLUDED(mutex_);

    //okagv
    void ClearWorkQueue() LOCKS_EXCLUDED(mutex_)
    LOCKS_EXCLUDED(work_queue_mutex_);

    //okagv
    //absl::Mutex delete_mutex_;

    // If it exists, further work items must be added to this queue, and will be
    // considered later.
    std::unique_ptr<WorkQueue> work_queue_ GUARDED_BY(work_queue_mutex_);

    // We globally localize a fraction of the nodes from each trajectory.
    absl::flat_hash_map<int, std::unique_ptr<common::FixedRatioSampler>>
    global_localization_samplers_ GUARDED_BY(mutex_);

    // Number of nodes added since last loop closure.
    int num_nodes_since_last_loop_closure_ GUARDED_BY(mutex_) = 0;

    // Current optimization problem.
    std::unique_ptr<optimization::OptimizationProblem2D> optimization_problem_;
    constraints::ConstraintBuilder2D constraint_builder_;

    // Thread pool used for handling the work queue.
    common::ThreadPool* const thread_pool_;

    // List of all trimmers to consult when optimizations finish.
    std::vector<std::unique_ptr<PoseGraphTrimmer>> trimmers_ GUARDED_BY(mutex_);

    PoseGraphData data_ GUARDED_BY(mutex_);

    ValueConversionTables conversion_tables_;

    //okagv
    mapping::PoseGraphInterface::OKagvOrder current_okagv_order_ =
            mapping::PoseGraphInterface::OKagvOrder::WAIT;
    //okagv
    mapping::PoseGraphInterface::OKagvFeedback current_okagv_feedback_;
    //mapping::PoseGraphInterface::OKagvFeedback::PROCESSING;
            
    //okagv
    bool add_relocalization_constraint;

    //okagv
    double covariance_score_ = 0.0;
    double last_covariance_score_ = 0.0;
    double last_decrease_covariance_score_ = 0.0;
    absl::optional<std::chrono::steady_clock::time_point> last_wall_time_;
    bool is_update_score = false;

    //use for justify if the robot in right navigation;
    int stable_localization_count = 0; 
    double last_stable_covariance_score_ = 0.0;
    absl::optional<std::chrono::steady_clock::time_point> last_stable_wall_time_; 

    //okagv
    int current_map_trajectory_id_ = 1000;
    int current_localization_trajectory_id_ = 1001;
    std::vector<int> working_trajectory_ids;

    //okagv
    transform::Rigid3d last_node_global_pose;
    common::Time last_time;

    //CovarianceScore
    transform::Rigid3d current_node_global_pose;
    transform::Rigid3d last_record_node_global_pose;

    //okagv
    bool is_robot_hijack = false;
    transform::Rigid3d last_hijack_pose;
    
    //okagv
    ThreadPoolState current_thread_pool_state_ = ThreadPoolState::ACTIVE;

    //okagv,use for trimmer the submap
    std::thread* trimmers_thread_;
    //okagv
    bool start_check_and_trim_submap_ = false;

    //okagv
    bool process_work_queue = true;

    //okagv
    bool is_node_exist = false;

    //okagv
    bool is_start_clear_work_queue = false;
    bool is_end_clear_work_queue = false;

    // Allows querying and manipulating the pose graph by the 'trimmers_'. The
    // 'mutex_' of the pose graph is held while this class is used.
    class TrimmingHandle : public Trimmable {
    public:
        TrimmingHandle(PoseGraph2D* parent);
        ~TrimmingHandle() override {}

        int num_submaps(int trajectory_id) const override;
        std::vector<SubmapId> GetSubmapIds(int trajectory_id) const override;
        MapById<SubmapId, SubmapData> GetOptimizedSubmapData() const override
        EXCLUSIVE_LOCKS_REQUIRED(parent_->mutex_);
        const MapById<NodeId, TrajectoryNode>& GetTrajectoryNodes() const override
        EXCLUSIVE_LOCKS_REQUIRED(parent_->mutex_);
        const std::vector<Constraint>& GetConstraints() const override
        EXCLUSIVE_LOCKS_REQUIRED(parent_->mutex_);
        void TrimSubmap(const SubmapId& submap_id)
        EXCLUSIVE_LOCKS_REQUIRED(parent_->mutex_) override;
        bool IsFinished(int trajectory_id) const override
        EXCLUSIVE_LOCKS_REQUIRED(parent_->mutex_);
        void SetTrajectoryState(int trajectory_id, TrajectoryState state) override
        EXCLUSIVE_LOCKS_REQUIRED(parent_->mutex_);

        //okagv
        std::vector<NodeId> GetNodeIds(int trajectory_id) const override;

        //okagv
        void TrimNode(const NodeId& node_id)
        EXCLUSIVE_LOCKS_REQUIRED(parent_->mutex_) override;

    private:
        PoseGraph2D* const parent_;

    };
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_2D_POSE_GRAPH_2D_H_
