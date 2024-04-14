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

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_H

#include <map>
#include <memory>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "absl/synchronization/mutex.h"
#include "cartographer/common/fixed_ratio_sampler.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/mapping/pose_extrapolator.h"
#include "cartographer_ros/map_builder_bridge.h"
#include "cartographer_ros/metrics/family_factory.h"
#include "cartographer_ros/node_constants.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/trajectory_options.h"
#include "cartographer_ros_msgs/FinishTrajectory.h"
#include "cartographer_ros_msgs/GetTrajectoryStates.h"
#include "cartographer_ros_msgs/ReadMetrics.h"
#include "cartographer_ros_msgs/StartTrajectory.h"
#include "cartographer_ros_msgs/StatusResponse.h"
#include "cartographer_ros_msgs/SubmapEntry.h"
#include "cartographer_ros_msgs/SubmapList.h"
#include "cartographer_ros_msgs/SubmapQuery.h"
#include "cartographer_ros_msgs/WriteState.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/MultiEchoLaserScan.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf2_ros/transform_broadcaster.h"

// okagv
#include <dirent.h>
#include <iostream>
#include "cartographer_ros_msgs/DeleteTrajectory.h"
#include "cartographer_ros_msgs/LoadTrajectory.h"
#include "cartographer_ros_msgs/LocalizeTrajectory.h"
#include "cartographer_ros_msgs/SetTrajectoryStates.h"
#include "cartographer_ros_msgs/LaserScanStates.h"
#include "cartographer_ros_msgs/ScanQualityQuery.h"

// okagv
#include <thread>
#include "boost/thread.hpp"
#include "boost/thread/condition_variable.hpp"
#include "boost/thread/mutex.hpp"

//okagv
#include "async_grpc/execution_context.h"
#include "async_grpc/server.h"

//okagv
#include "cartographer_ros/map_builder_server_options.h"

//okagv
#include<stdlib.h>

namespace cartographer_ros {

// Wires up ROS topics to SLAM.
class Node {
 public:
  Node(const NodeOptions& node_options,
       std::shared_ptr<cartographer::mapping::MapBuilderInterface>
           map_builder,  // okagv default is unique_ptr
       tf2_ros::Buffer* tf_buffer, bool collect_metrics);
  ~Node();

  Node(const Node&) = delete;
  Node& operator=(const Node&) = delete;

  // Finishes all yet active trajectories.
  void FinishAllTrajectories();
  // Finishes a single given trajectory. Returns false if the trajectory did not
  // exist or was already finished.
  bool FinishTrajectory(int trajectory_id);

  // Runs final optimization. All trajectories have to be finished when calling.
  void RunFinalOptimization();

  // Starts the first trajectory with the default topics.
  void StartTrajectoryWithDefaultTopics(const TrajectoryOptions& options);

  // Returns unique SensorIds for multiple input bag files based on
  // their TrajectoryOptions.
  // 'SensorId::id' is the expected ROS topic name.
  std::vector<
      std::set<::cartographer::mapping::TrajectoryBuilderInterface::SensorId>>
  ComputeDefaultSensorIdsForMultipleBags(
      const std::vector<TrajectoryOptions>& bags_options) const;

  // Adds a trajectory for offline processing, i.e. not listening to topics.
  int AddOfflineTrajectory(
      const std::set<
          cartographer::mapping::TrajectoryBuilderInterface::SensorId>&
          expected_sensor_ids,
      const TrajectoryOptions& options);

  // The following functions handle adding sensor data to a trajectory.
  void HandleOdometryMessage(int trajectory_id, const std::string& sensor_id,
                             const nav_msgs::Odometry::ConstPtr& msg);
  void HandleNavSatFixMessage(int trajectory_id, const std::string& sensor_id,
                              const sensor_msgs::NavSatFix::ConstPtr& msg);
  void HandleLandmarkMessage(
      int trajectory_id, const std::string& sensor_id,
      const cartographer_ros_msgs::LandmarkList::ConstPtr& msg);

  /*--------------------------------------------------------------------/
  /        IMU             HandleImuMessage                             /                    
  /--------------------------------------------------------------------*/        
  void HandleImuMessage(int trajectory_id, const std::string& sensor_id,
                        const sensor_msgs::Imu::ConstPtr& msg);

  /*--------------------------------------------------------------------/
  /        LASER             HandleLaserScanMessage                     /                    
  /--------------------------------------------------------------------*/      
  void HandleLaserScanMessage(int trajectory_id, const std::string& sensor_id,
                              const sensor_msgs::LaserScan::ConstPtr& msg);
  void HandleMultiEchoLaserScanMessage(
      int trajectory_id, const std::string& sensor_id,
      const sensor_msgs::MultiEchoLaserScan::ConstPtr& msg);
  void HandlePointCloud2Message(int trajectory_id, const std::string& sensor_id,
                                const sensor_msgs::PointCloud2::ConstPtr& msg);

  void HandleInitialPoseMessage(
      const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

  // Serializes the complete Node state.
  void SerializeState(const std::string& filename,
                      const bool include_unfinished_submaps);

  // Loads a serialized SLAM state from a .pbstream file.
  void LoadState(const std::string& state_filename, bool load_frozen_state);

  // okagv
  void SetConfigurationParam(std::string directory, std::string slam_basename, std::string navigation_basename);

  // okagv
  void LoadMaps(const std::string& file_directory);

  // okagv
  void RecordMapBuilderOption(
      cartographer::cloud::proto::MapBuilderServerOptions& slam_option,
      cartographer::cloud::proto::MapBuilderServerOptions& navigation_option);

  //okagv
  void RecordQrCodePosition(const cartographer_ros_msgs::LandmarkList::ConstPtr& msg);

  ::ros::NodeHandle* node_handle();

  // okagv
  /*--------------------------------------------------------------------/
  /        START             HandleStartTrajectory                      /                    
  /--------------------------------------------------------------------*/
  bool HandleStartTrajectory(
      cartographer_ros_msgs::StartTrajectory::Request& request,
      cartographer_ros_msgs::StartTrajectory::Response& response);
  bool HandleStartTrajectoryDetail(
      cartographer_ros_msgs::StartTrajectory::Request& request,
      cartographer_ros_msgs::StartTrajectory::Response& response);

  //okagv
   void init_grpc_server();

 private:
  struct Subscriber {
    ::ros::Subscriber subscriber;

    // ::ros::Subscriber::getTopic() does not necessarily return the same
    // std::string
    // it was given in its constructor. Since we rely on the topic name as the
    // unique identifier of a subscriber, we remember it ourselves.
    std::string topic;
  };

  bool HandleSubmapQuery(
      cartographer_ros_msgs::SubmapQuery::Request& request,
      cartographer_ros_msgs::SubmapQuery::Response& response);
  bool HandleTrajectoryQuery(
      ::cartographer_ros_msgs::TrajectoryQuery::Request& request,
      ::cartographer_ros_msgs::TrajectoryQuery::Response& response);
  // bool HandleStartTrajectory(
  //    cartographer_ros_msgs::StartTrajectory::Request& request,
  //    cartographer_ros_msgs::StartTrajectory::Response& response);

  /*--------------------------------------------------------------------/
  /       FINSIH              HandleFinishTrajectory                    /                    
  /--------------------------------------------------------------------*/
  bool HandleFinishTrajectory(
      cartographer_ros_msgs::FinishTrajectory::Request& request,
      cartographer_ros_msgs::FinishTrajectory::Response& response);
  bool HandleFinishTrajectoryDetail(
      cartographer_ros_msgs::FinishTrajectory::Request& request,
      cartographer_ros_msgs::FinishTrajectory::Response& response);

  /*---------------------------------------------------------------------/
  /       SAVE              HandleWriteState                             /                    
  /---------------------------------------------------------------------*/
  bool HandleWriteState(cartographer_ros_msgs::WriteState::Request& request,
                        cartographer_ros_msgs::WriteState::Response& response);
  bool HandleWriteStateDetail(cartographer_ros_msgs::WriteState::Request& request,
                        cartographer_ros_msgs::WriteState::Response& response);

  /*---------------------------------------------------------------------/
  /       UPDATE              HandleWriteState                             /                    
  /---------------------------------------------------------------------*/
  bool HandleUpdateState(cartographer_ros_msgs::WriteState::Request& request,
                         cartographer_ros_msgs::WriteState::Response& response);
  bool HandleUpdateStateDetail(
      cartographer_ros_msgs::WriteState::Request& request,
      cartographer_ros_msgs::WriteState::Response& response);

  bool HandleGetTrajectoryStates(
      ::cartographer_ros_msgs::GetTrajectoryStates::Request& request,
      ::cartographer_ros_msgs::GetTrajectoryStates::Response& response);
  bool HandleReadMetrics(
      cartographer_ros_msgs::ReadMetrics::Request& request,
      cartographer_ros_msgs::ReadMetrics::Response& response);

  // okagv

  /*---------------------------------------------------------------------/
  /       LOAD              HandleLoadTrajectory                         /                    
  /---------------------------------------------------------------------*/
  bool HandleLoadTrajectory(
      cartographer_ros_msgs::LoadTrajectory::Request& request,
      cartographer_ros_msgs::LoadTrajectory::Response& response);
  bool HandleLoadTrajectoryDetail(
      cartographer_ros_msgs::LoadTrajectory::Request& request,
      cartographer_ros_msgs::LoadTrajectory::Response& response);

  /*---------------------------------------------------------------------/
  /       Delete              HandleDeleteTrajectory                     /                    
  /---------------------------------------------------------------------*/
  bool HandleDeleteTrajectory(
      cartographer_ros_msgs::DeleteTrajectory::Request& request,
      cartographer_ros_msgs::DeleteTrajectory::Response& response);
  bool HandleDeleteTrajectoryDetail(
      cartographer_ros_msgs::DeleteTrajectory::Request& request,
      cartographer_ros_msgs::DeleteTrajectory::Response& response);

  /*---------------------------------------------------------------------/
  /       Localization             HandleLocalizeTrajectory              /                    
  /---------------------------------------------------------------------*/
  bool HandleLocalizeTrajectory(
      cartographer_ros_msgs::LocalizeTrajectory::Request& request,
      cartographer_ros_msgs::LocalizeTrajectory::Response& response);
  bool HandleLocalizeTrajectoryDetail(
      cartographer_ros_msgs::LocalizeTrajectory::Request& request,
      cartographer_ros_msgs::LocalizeTrajectory::Response& response);

  bool HandleSetTrajectoryStates(
      ::cartographer_ros_msgs::SetTrajectoryStates::Request& request,
      ::cartographer_ros_msgs::SetTrajectoryStates::Response& response);

  /*---------------------------------------------------------------------/
  /       QualityQuery             HandleScanQualityQuery              /                    
  /---------------------------------------------------------------------*/
  bool HandleScanQualityQuery(
      ::cartographer_ros_msgs::ScanQualityQuery::Request& request,
      ::cartographer_ros_msgs::ScanQualityQuery::Response& response);

  /*---------------------------------------------------------------------/
  /       Remove(finish and delete)    FinishAndDeleteTrajectory         /                    
  /---------------------------------------------------------------------*/
  bool FinishAndDeleteTrajectory(std::string currentTrajectoryId);

  /*---------------------------------------------------------------------/
  /       Remove(finish and save)    FinishAndSaveTrajectory             /                    
  /---------------------------------------------------------------------*/
  bool FinishAndSaveTrajectory(std::string currentTrajectoryId);

  bool LoadTrajectory(std::string mapTrajectoryId);

  bool StartNavigation(std::string trajectory_id, bool use_initial_pose,
    const geometry_msgs::Pose initial_pose);

  bool FinishNavigation(std::string trajectory_id);

  /*---------------------------------------------------------------------/
  /       UPDATE AND SAVE    UpdateAndSaveTrajectory                     /                    
  /---------------------------------------------------------------------*/
  bool UpdateAndSaveTrajectory(std::string trajectory_id);

  // Returns the set of SensorIds expected for a trajectory.
  // 'SensorId::id' is the expected ROS topic name.
  std::set<::cartographer::mapping::TrajectoryBuilderInterface::SensorId>
  ComputeExpectedSensorIds(const TrajectoryOptions& options) const;
  int AddTrajectory(const TrajectoryOptions& options);
  void LaunchSubscribers(const TrajectoryOptions& options, int trajectory_id);
  void PublishSubmapList(const ::ros::WallTimerEvent& timer_event);
  void AddExtrapolator(int trajectory_id, const TrajectoryOptions& options);
  void AddSensorSamplers(int trajectory_id, const TrajectoryOptions& options);
  void PublishLocalTrajectoryData(const ::ros::TimerEvent& timer_event);
  void PublishTrajectoryNodeList(const ::ros::WallTimerEvent& timer_event);
  void PublishLandmarkPosesList(const ::ros::WallTimerEvent& timer_event);
  void PublishConstraintList(const ::ros::WallTimerEvent& timer_event);
  bool ValidateTrajectoryOptions(const TrajectoryOptions& options);
  bool ValidateTopicNames(const TrajectoryOptions& options);
  cartographer_ros_msgs::StatusResponse FinishTrajectoryUnderLock(
      int trajectory_id) EXCLUSIVE_LOCKS_REQUIRED(mutex_);
  void MaybeWarnAboutTopicMismatch(const ::ros::WallTimerEvent&);

  // okagv
  int AddTrajectoryWithId(const int trajectory_id,
                          const TrajectoryOptions& options);

  // okagv
  cartographer_ros_msgs::StatusResponse LoadTrajectoryUnderLock(
      int trajectory_id, std::string state_filename, uint8_t state)
      EXCLUSIVE_LOCKS_REQUIRED(mutex_);
  cartographer_ros_msgs::StatusResponse DeleteTrajectoryUnderLock(
      int trajectory_id) EXCLUSIVE_LOCKS_REQUIRED(mutex_);
  cartographer_ros_msgs::StatusResponse LocalizeTrajectoryUnderLock(
      int trajectory_id, bool use_initial_pose,
      const geometry_msgs::Pose initial_pose) EXCLUSIVE_LOCKS_REQUIRED(mutex_);

  // okagv
  void CheckAndRunOKagvOrder(const ::ros::WallTimerEvent& timer_event);

  // okagv
  void SetTrajectoryTypeWithId(std::string type, int trajectory_id);

  //okagv
  void RegisterClientIdForTrajectory(int trajectory_id, std::string trajectory_name);

  // okagv
  void GetFileNames(std::string path, std::vector<std::string>& filenames);

  //okagv
  void StartRelocalizeRobot();

  // Helper function for service handlers that need to check trajectory states.
  cartographer_ros_msgs::StatusResponse TrajectoryStateToStatus(
      int trajectory_id,
      const std::set<
          cartographer::mapping::PoseGraphInterface::TrajectoryState>&
          valid_states);
  const NodeOptions node_options_;

  tf2_ros::TransformBroadcaster tf_broadcaster_;

  absl::Mutex mutex_;
  std::unique_ptr<cartographer_ros::metrics::FamilyFactory> metrics_registry_;
  MapBuilderBridge map_builder_bridge_ GUARDED_BY(mutex_);

  ::ros::NodeHandle node_handle_;
  ::ros::Publisher submap_list_publisher_;
  ::ros::Publisher trajectory_node_list_publisher_;
  ::ros::Publisher landmark_poses_list_publisher_;
  ::ros::Publisher constraint_list_publisher_;
  // These ros::ServiceServers need to live for the lifetime of the node.
  std::vector<::ros::ServiceServer> service_servers_;
  ::ros::Publisher scan_matched_point_cloud_publisher_;

  //okagv
  ::ros::Publisher current_pose_publisher_;
  ::ros::Subscriber initial_pose_subscriber_;
  ::ros::Publisher rosbag_start_recorder_;
  ::ros::Publisher rosbag_stop_recorder_;

  //okagv test position
  ::ros::Subscriber qr_code_position_subscriber_;

  struct TrajectorySensorSamplers {
    TrajectorySensorSamplers(const double rangefinder_sampling_ratio,
                             const double odometry_sampling_ratio,
                             const double fixed_frame_pose_sampling_ratio,
                             const double imu_sampling_ratio,
                             const double landmark_sampling_ratio)
        : rangefinder_sampler(rangefinder_sampling_ratio),
          odometry_sampler(odometry_sampling_ratio),
          fixed_frame_pose_sampler(fixed_frame_pose_sampling_ratio),
          imu_sampler(imu_sampling_ratio),
          landmark_sampler(landmark_sampling_ratio) {}

    ::cartographer::common::FixedRatioSampler rangefinder_sampler;
    ::cartographer::common::FixedRatioSampler odometry_sampler;
    ::cartographer::common::FixedRatioSampler fixed_frame_pose_sampler;
    ::cartographer::common::FixedRatioSampler imu_sampler;
    ::cartographer::common::FixedRatioSampler landmark_sampler;
  };

  // These are keyed with 'trajectory_id'.
  std::map<int, ::cartographer::mapping::PoseExtrapolator> extrapolators_;
  std::unordered_map<int, TrajectorySensorSamplers> sensor_samplers_;
  std::unordered_map<int, std::vector<Subscriber>> subscribers_;
  std::unordered_set<std::string> subscribed_topics_;
  std::unordered_set<int> trajectories_scheduled_for_finish_;

  // okgav, maybe remove later version
  std::unordered_set<int> trajectories_scheduled_for_load_;
  std::unordered_set<int> trajectories_scheduled_for_delete_;
  std::unordered_set<int> trajectories_scheduled_for_localize_;

  // We have to keep the timer handles of ::ros::WallTimers around, otherwise
  // they do not fire.
  std::vector<::ros::WallTimer> wall_timers_;

  // The timer for publishing local trajectory data (i.e. pose transforms and
  // range data point clouds) is a regular timer which is not triggered when
  // simulation time is standing still. This prevents overflowing the transform
  // listener buffer by publishing the same transforms over and over again.
  ::ros::Timer publish_local_trajectory_data_timer_;

  TrajectoryType current_trajectory_type = TrajectoryType::IDLE;
  TrajectoryType scheduled_trajectory_type = TrajectoryType::IDLE;

  // okagv
  ::cartographer::transform::Rigid3d current_tracking_to_map;
  ::cartographer::transform::Rigid3d smooth_tracking_to_map;
  ::cartographer::transform::Rigid3d last_tracking_to_map;
  std::unordered_map<int, ::cartographer::transform::Rigid3d>
      trajectories_last_tracking_to_map;

  std::string configuration_directory_;
  std::string configuration_slam_basename_;
  std::string configuration_navigation_basename_;

  // okagv
  std::map<std::string, int> trajectory_id_toint;
    // okagv
  int last_map_trajectory_id = 0;
  std::string last_map_trajectory_id_name;

  int last_trajectory_id = 0;  // slam or navigation
  std::string last_trajectory_id_name;

  // okagv
  bool is_service_done = true;

  // okagv
  std::thread* timer_thread_;
  // okagv localization
  bool is_start_robot_relocalizaton = false;
  cartographer_ros_msgs::LocalizeTrajectory::Request relocalization_request;

  //okagv
  ros::Time last_scan_time = ros::Time(0);
  bool is_scan_recived = false;

  int laser_scan_state_log = cartographer_ros_msgs::LaserScanStates::OK;

  //okagv test
  int map_count = 0;

  //okagv
  ros::Time last_localization_time = ros::Time(0);

  //okagv
  double last_covariance_score = 0.0;
  ros::Time last_update_time = ros::Time(0);
  geometry_msgs::Pose last_update_pose;

  //okagv
  cartographer::cloud::proto::MapBuilderServerOptions slam_option_;
  cartographer::cloud::proto::MapBuilderServerOptions navigation_option_;

};

}  // namespace cartographer_ros

//#include "cartographer_ros/node_context_impl.h"

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_H
