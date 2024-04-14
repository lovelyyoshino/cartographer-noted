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

#include "cartographer_ros/node.h"

#include <chrono>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "absl/memory/memory.h"
#include "absl/strings/str_cat.h"
#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/pose_graph_interface.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/metrics/register.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/metrics/family_factory.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/sensor_bridge.h"
#include "cartographer_ros/tf_bridge.h"
#include "cartographer_ros/time_conversion.h"
#include "cartographer_ros_msgs/StatusCode.h"
#include "cartographer_ros_msgs/StatusResponse.h"
#include "glog/logging.h"
#include "nav_msgs/Odometry.h"
#include "ros/serialization.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf2_eigen/tf2_eigen.h"
#include "visualization_msgs/MarkerArray.h"

// okagv
#include "cartographer_ros_msgs/RobotPose.h"
#include "tf/tf.h"

//okagv
#include <unistd.h>
#include <sys/types.h>  
#include <sys/stat.h>  

//okagv
#include "cartographer_ros_msgs/Rosbag.h"
#include "std_msgs/String.h";

namespace cartographer_ros {

namespace carto = ::cartographer;

using carto::transform::Rigid3d;
using TrajectoryState =
    ::cartographer::mapping::PoseGraphInterface::TrajectoryState;

using OKagv_Feedback = ::cartographer::mapping::PoseGraphInterface::OKagvFeedback;
using OKagv_State = ::cartographer::mapping::PoseGraphInterface::OKagvState;

namespace {
// Subscribes to the 'topic' for 'trajectory_id' using the 'node_handle' and
// calls 'handler' on the 'node' to handle messages. Returns the subscriber.
template <typename MessageType>
::ros::Subscriber SubscribeWithHandler(
    void (Node::*handler)(int, const std::string&,
                          const typename MessageType::ConstPtr&),
    const int trajectory_id, const std::string& topic,
    ::ros::NodeHandle* const node_handle, Node* const node) {
  return node_handle->subscribe<MessageType>(
      topic, kInfiniteSubscriberQueueSize,
      boost::function<void(const typename MessageType::ConstPtr&)>(
          [node, handler, trajectory_id,
           topic](const typename MessageType::ConstPtr& msg) {
            (node->*handler)(trajectory_id, topic, msg);
          }));
}

std::string TrajectoryStateToString(const TrajectoryState trajectory_state) {
  switch (trajectory_state) {
    case TrajectoryState::ACTIVE:
      return "ACTIVE";
    case TrajectoryState::FINISHED:
      return "FINISHED";
    case TrajectoryState::FROZEN:
      return "FROZEN";
    case TrajectoryState::DELETED:
      return "DELETED";
    case TrajectoryState::IDLE:
      return "IDLE";
  }
  return "";
}

}  // namespace

bool createFolder(const std::string path) {
  LOG(INFO) << "path : " << path;

  if (!access(path.c_str(), F_OK) || path == "") {
    return true;
  }

  //从字符串末尾开始查找‘/’
  size_t pos = path.rfind("/");
  if (pos == std::string::npos) {
    LOG(INFO) << "no find '/'";
    return false;
  }

  std::string upper_path = path.substr(0, pos);
  if (createFolder(upper_path)) {
    // S_IRWXU|S_IRWXG|S_IRWXO目录访问权限
    if (mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IRWXO)) {
      // EEXIST表示目录已经存在
      if (errno != EEXIST) {
        LOG(INFO) << "failed to create folder : " << path;
        return false;
      }
    }
    return true;
  }
  return false;
}

Node::Node(
    const NodeOptions& node_options,
    std::shared_ptr<cartographer::mapping::MapBuilderInterface> map_builder,
    tf2_ros::Buffer* const tf_buffer, const bool collect_metrics)
    : node_options_(node_options),
      map_builder_bridge_(node_options_, 
      map_builder, //std::move(map_builder),
      tf_buffer) {
  absl::MutexLock lock(&mutex_);
  if (collect_metrics) {
    metrics_registry_ = absl::make_unique<metrics::FamilyFactory>();
    carto::metrics::RegisterAllMetrics(metrics_registry_.get());
  }

  submap_list_publisher_ =
      node_handle_.advertise<::cartographer_ros_msgs::SubmapList>(
          kSubmapListTopic, kLatestOnlyPublisherQueueSize);
  trajectory_node_list_publisher_ =
      node_handle_.advertise<::visualization_msgs::MarkerArray>(
          kTrajectoryNodeListTopic, kLatestOnlyPublisherQueueSize);
  landmark_poses_list_publisher_ =
      node_handle_.advertise<::visualization_msgs::MarkerArray>(
          kLandmarkPosesListTopic, kLatestOnlyPublisherQueueSize);
  constraint_list_publisher_ =
      node_handle_.advertise<::visualization_msgs::MarkerArray>(
          kConstraintListTopic, kLatestOnlyPublisherQueueSize);
  service_servers_.push_back(node_handle_.advertiseService(
      kSubmapQueryServiceName, &Node::HandleSubmapQuery, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kTrajectoryQueryServiceName, &Node::HandleTrajectoryQuery, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kStartTrajectoryServiceName, &Node::HandleStartTrajectory, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kFinishTrajectoryServiceName, &Node::HandleFinishTrajectory, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kWriteStateServiceName, &Node::HandleWriteState, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kGetTrajectoryStatesServiceName, &Node::HandleGetTrajectoryStates, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kReadMetricsServiceName, &Node::HandleReadMetrics, this));

  // okagv
  service_servers_.push_back(node_handle_.advertiseService(
      kLoadTrajectoryServiceName, &Node::HandleLoadTrajectory, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kDeleteTrajectoryServiceName, &Node::HandleDeleteTrajectory, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kLocalizeTrajectoryServiceName, &Node::HandleLocalizeTrajectory, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kSetTrajectoryStatesServiceName, &Node::HandleSetTrajectoryStates, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kUpdateStateServiceName, &Node::HandleUpdateState, this));


  scan_matched_point_cloud_publisher_ =
      node_handle_.advertise<sensor_msgs::PointCloud2>(
          kScanMatchedPointCloudTopic, kLatestOnlyPublisherQueueSize);

  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(node_options_.submap_publish_period_sec),
      &Node::PublishSubmapList, this));
  if (node_options_.pose_publish_period_sec > 0) {
    publish_local_trajectory_data_timer_ = node_handle_.createTimer(
        ::ros::Duration(node_options_.pose_publish_period_sec),
        &Node::PublishLocalTrajectoryData, this);
  }
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(node_options_.trajectory_publish_period_sec),
      &Node::PublishTrajectoryNodeList, this));
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(node_options_.trajectory_publish_period_sec),
      &Node::PublishLandmarkPosesList, this));
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(kConstraintPublishPeriodSec),
      &Node::PublishConstraintList, this));

  // okagv
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(kConstraintPublishPeriodSec),
       &Node::CheckAndRunOKagvOrder, this));
 
  // okagv
  initial_pose_subscriber_ = node_handle_.subscribe(
      "/initialpose", 1, &Node::HandleInitialPoseMessage, this);

  // okagv
  current_pose_publisher_ =
      node_handle_.advertise<::cartographer_ros_msgs::RobotPose>(
          kRobotPoseTopic, kLatestOnlyPublisherQueueSize);

  timer_thread_ = new std::thread(boost::bind(&Node::StartRelocalizeRobot, this));

  //okagv
  service_servers_.push_back(node_handle_.advertiseService(
      kLaserScanQualityServiceName, &Node::HandleScanQualityQuery, this));

  // okagv
  rosbag_start_recorder_ =
      node_handle_.advertise<::cartographer_ros_msgs::Rosbag>(
          kRosbagStartRecordTopic, kLatestOnlyPublisherQueueSize);

  // okagv
  rosbag_stop_recorder_ =
      node_handle_.advertise<std_msgs::String>(
          kRosbagStopRecordTopic, kLatestOnlyPublisherQueueSize);

  //okagv
  qr_code_position_subscriber_ = node_handle_.subscribe(
      "/QRCodePositionOffset", 1, &Node::RecordQrCodePosition, this);
}

Node::~Node() { FinishAllTrajectories(); }

::ros::NodeHandle* Node::node_handle() { return &node_handle_; }

bool Node::HandleSubmapQuery(
    ::cartographer_ros_msgs::SubmapQuery::Request& request,
    ::cartographer_ros_msgs::SubmapQuery::Response& response) {
  absl::MutexLock lock(&mutex_);
  map_builder_bridge_.HandleSubmapQuery(request, response);
  return true;
}

bool Node::HandleTrajectoryQuery(
    ::cartographer_ros_msgs::TrajectoryQuery::Request& request,
    ::cartographer_ros_msgs::TrajectoryQuery::Response& response) {
  absl::MutexLock lock(&mutex_);
  response.status = TrajectoryStateToStatus(
      request.trajectory_id,
      {TrajectoryState::ACTIVE, TrajectoryState::FINISHED,
       TrajectoryState::FROZEN} /* valid states */);
  if (response.status.code != cartographer_ros_msgs::StatusCode::OK) {
    LOG(ERROR) << "Can't query trajectory from pose graph: "
               << response.status.message;
    return true;
  }
  map_builder_bridge_.HandleTrajectoryQuery(request, response);
  return true;
}

void Node::PublishSubmapList(const ::ros::WallTimerEvent& unused_timer_event) {
  absl::MutexLock lock(&mutex_);
  submap_list_publisher_.publish(map_builder_bridge_.GetSubmapList());
}

void Node::AddExtrapolator(const int trajectory_id,
                           const TrajectoryOptions& options) {
  constexpr double kExtrapolationEstimationTimeSec = 0.001;  // 1 ms
  CHECK(extrapolators_.count(trajectory_id) == 0);
  const double gravity_time_constant =
      node_options_.map_builder_options.use_trajectory_builder_3d()
          ? options.trajectory_builder_options.trajectory_builder_3d_options()
                .imu_gravity_time_constant()
          : options.trajectory_builder_options.trajectory_builder_2d_options()
                .imu_gravity_time_constant();
  extrapolators_.emplace(
      std::piecewise_construct, std::forward_as_tuple(trajectory_id),
      std::forward_as_tuple(
          ::cartographer::common::FromSeconds(kExtrapolationEstimationTimeSec),
          gravity_time_constant));
}

void Node::AddSensorSamplers(const int trajectory_id,
                             const TrajectoryOptions& options) {
  CHECK(sensor_samplers_.count(trajectory_id) == 0);
  sensor_samplers_.emplace(
      std::piecewise_construct, std::forward_as_tuple(trajectory_id),
      std::forward_as_tuple(
          options.rangefinder_sampling_ratio, options.odometry_sampling_ratio,
          options.fixed_frame_pose_sampling_ratio, options.imu_sampling_ratio,
          options.landmarks_sampling_ratio));
}

void Node::PublishLocalTrajectoryData(const ::ros::TimerEvent& timer_event) {
  absl::MutexLock lock(&mutex_);

  for (const auto& entry : map_builder_bridge_.GetLocalTrajectoryData()) {
    const auto& trajectory_data = entry.second;

    auto& extrapolator = extrapolators_.at(entry.first);
    // We only publish a point cloud if it has changed. It is not needed at high
    // frequency, and republishing it would be computationally wasteful.
    if (trajectory_data.local_slam_data->time !=
        extrapolator.GetLastPoseTime()) {
      if (scan_matched_point_cloud_publisher_.getNumSubscribers() > 0) {
        // TODO(gaschler): Consider using other message without time
        // information.
        carto::sensor::TimedPointCloud point_cloud;
        point_cloud.reserve(trajectory_data.local_slam_data->range_data_in_local
                                .returns.size());
        for (const cartographer::sensor::RangefinderPoint point :
             trajectory_data.local_slam_data->range_data_in_local.returns) {
          point_cloud.push_back(cartographer::sensor::ToTimedRangefinderPoint(
              point, 0.f /* time */));
        }
        scan_matched_point_cloud_publisher_.publish(ToPointCloud2Message(
            carto::common::ToUniversal(trajectory_data.local_slam_data->time),
            node_options_.map_frame,
            carto::sensor::TransformTimedPointCloud(
                point_cloud, trajectory_data.local_to_map.cast<float>())));
      }
      extrapolator.AddPose(trajectory_data.local_slam_data->time,
                           trajectory_data.local_slam_data->local_pose);
    }

    geometry_msgs::TransformStamped stamped_transform;
    // If we do not publish a new point cloud, we still allow time of the
    // published poses to advance. If we already know a newer pose, we use its
    // time instead. Since tf knows how to interpolate, providing newer
    // information is better.
    const ::cartographer::common::Time now = std::max(
        FromRos(ros::Time::now()), extrapolator.GetLastExtrapolatedTime());
    stamped_transform.header.stamp =
        node_options_.use_pose_extrapolator
            ? ToRos(now)
            : ToRos(trajectory_data.local_slam_data->time);
    const Rigid3d tracking_to_local_3d =
        node_options_.use_pose_extrapolator
            ? extrapolator.ExtrapolatePose(now)
            : trajectory_data.local_slam_data->local_pose;
    const Rigid3d tracking_to_local = [&] {
      if (trajectory_data.trajectory_options.publish_frame_projected_to_2d) {
        return carto::transform::Embed3D(
            carto::transform::Project2D(tracking_to_local_3d));
      }
      return tracking_to_local_3d;
    }();

    // default is const Rigid3d tracking_to_map
    Rigid3d tracking_to_map = trajectory_data.local_to_map * tracking_to_local;

    // okagv
    current_tracking_to_map = tracking_to_map;

    // okagv , smooth pose for navigation
    if (last_tracking_to_map.IsValid()) {
      double distance =
          (current_tracking_to_map.inverse() * last_tracking_to_map)
              .translation()
              .norm();
      /*
      auto yaw_quaternion =
          (current_tracking_to_map.inverse() * last_tracking_to_map).rotation();

      double yaw, pitch, roll;
      tf::Matrix3x3(tf::Quaternion(yaw_quaternion.x(), yaw_quaternion.y(),
                                   yaw_quaternion.z(), yaw_quaternion.w()))
          .getEulerYPR(yaw, pitch, roll);

      LOG(INFO) << "Peak.ding yaw_varity " << yaw * 180 /3.1415926;
      */

      /*
      current_trajectory_type =
      map_builder_bridge_.GetTrajectoryTypeWithId(current_trajectory_id);

      if(current_trajectory_type ==  TrajectoryType::NAVIGATION)
      {
         LOG(INFO) << "Peak.ding current_trajectory_type NAVIGATION";
      }
      else if(current_trajectory_type ==  TrajectoryType::RELOCALIZAION)
      {
         LOG(INFO) << "Peak.ding current_trajectory_type RELOCALIZAION";
      }
      */

      // if(current_trajectory_type == TrajectoryType::RELOCALIZAION)
      // {
      //     LOG(INFO) << "Relocalizating...";
      //     absl::SleepFor(absl::Milliseconds(1500));

      //     current_trajectory_type =
      //     map_builder_bridge_.GetTrajectoryTypeWithId(current_trajectory_id);
      // }

      if (node_options_.use_pose_smoother == true &&
          distance > node_options_.smoother_variety_distance &&
          distance < node_options_.relocalization_variety_distance) {
        // LOG(INFO) << "Peak.ding update distance up to 0.05 and value is " <<
        // distance;
        const double factor = node_options_.pose_publish_period_sec /
                              node_options_.imu_publish_period_sec;
        // LOG(INFO) << "Peak.ding update factor is " << factor;
        const Eigen::Vector3d origin = last_tracking_to_map.translation() +
                                       (current_tracking_to_map.translation() -
                                        last_tracking_to_map.translation()) *
                                           factor;
        const Eigen::Quaterniond rotation = current_tracking_to_map.rotation();
        Eigen::Quaterniond(last_tracking_to_map.rotation())
            .slerp(factor,
                   Eigen::Quaterniond(current_tracking_to_map.rotation()));

        smooth_tracking_to_map =
            ::cartographer::transform::Rigid3d(origin, rotation);

        tracking_to_map = smooth_tracking_to_map;
        current_tracking_to_map = smooth_tracking_to_map;
      }
    }

    if (trajectory_data.published_to_tracking != nullptr) {
      if (trajectory_data.trajectory_options.provide_odom_frame) {
        std::vector<geometry_msgs::TransformStamped> stamped_transforms;

        stamped_transform.header.frame_id = node_options_.map_frame;
        stamped_transform.child_frame_id =
            trajectory_data.trajectory_options.odom_frame;
        stamped_transform.transform =
            ToGeometryMsgTransform(trajectory_data.local_to_map);
        stamped_transforms.push_back(stamped_transform);

        stamped_transform.header.frame_id =
            trajectory_data.trajectory_options.odom_frame;
        stamped_transform.child_frame_id =
            trajectory_data.trajectory_options.published_frame;
        stamped_transform.transform = ToGeometryMsgTransform(
            tracking_to_local * (*trajectory_data.published_to_tracking));
        stamped_transforms.push_back(stamped_transform);

        tf_broadcaster_.sendTransform(stamped_transforms);
      } else {
        stamped_transform.header.frame_id = node_options_.map_frame;
        stamped_transform.child_frame_id =
            trajectory_data.trajectory_options.published_frame;
        stamped_transform.transform = ToGeometryMsgTransform(
            tracking_to_map * (*trajectory_data.published_to_tracking));
        tf_broadcaster_.sendTransform(stamped_transform);

        // okagv
        ::geometry_msgs::Transform temp = stamped_transform.transform;
        //okagv
        cartographer_ros_msgs::RobotPose current_pose;

        current_pose.robot_pose.position.x = temp.translation.x;
        current_pose.robot_pose.position.y = temp.translation.y;
        current_pose.robot_pose.position.z = temp.translation.z;
        current_pose.robot_pose.orientation = temp.rotation;
        current_pose.covariance_score = trajectory_data.covariance_score;

        if (last_scan_time != ros::Time(0)) {
          ros::Duration duration = ros::Time::now() - last_scan_time;
          if (duration.toSec() > 0.5) {
            current_pose.covariance_score = 0;
            //LOG(INFO) << "Peak.ding scan missing";
          } else if (laser_scan_state_log !=
                     cartographer_ros_msgs::LaserScanStates::OK) {
            current_pose.covariance_score = 0;
            //LOG(INFO) << "Peak.ding scan have missed";           
          }
        }

        if (current_pose.covariance_score > 0) {
          current_pose.current_trajectory = last_map_trajectory_id_name;
        } else {
          // LOG(WARNING) << "Peak.ding Check Robot need relocalization";
        }

        /***********************************************************/

        if (last_covariance_score == 0.0) {
          //LOG(INFO) << "Peak.ding init last_covariance_score";
          last_update_time = ros::Time::now();
          last_update_pose = current_pose.robot_pose;
        } else {
          if (trajectory_data.is_update_score) {
            //LOG(INFO) << "Peak.ding update last_update_time";
            last_update_time = ros::Time::now();
            last_update_pose = current_pose.robot_pose;
          }

          if (last_update_time != ros::Time(0)) {
            ros::Duration duration = ros::Time::now() - last_update_time;
            current_pose.last_update_duration = duration.toSec();
          }
        }

        current_pose.last_update_pose = last_update_pose;

        /***********************************************************/

        /*
        double yaw, pitch, roll;
        tf::Matrix3x3(tf::Quaternion(temp.rotation.x, temp.rotation.y,
        temp.rotation.z, temp.rotation.w)).getEulerYPR(yaw,pitch,roll);
        current_pose.robot_pose.position.z = yaw;
        */
    

        current_pose_publisher_.publish(current_pose);
        
        //update last score
        last_covariance_score = current_pose.covariance_score;
      }
    }
  }

  if (last_tracking_to_map.IsValid()) {
    double distance = (current_tracking_to_map.inverse() * last_tracking_to_map)
                          .translation()
                          .norm();

    // LOG(INFO) << "Peak.ding distance is " << distance;
    if (distance > 0.05) {
      //LOG(WARNING) << "Peak.ding CHECK AAAAA distance up to 0.05 and value is"
      //          << distance;
      last_tracking_to_map = current_tracking_to_map;
      return;
    }
    
    // if(distance < 0.02)
    // {
    //     LOG(INFO) << "Peak.ding update distance up to 0.01 and value is " <<
    //     distance; last_tracking_to_map = current_tracking_to_map; return;
    // }
    // else if (distance < 0.03)
    // {
    //     LOG(INFO) << "Peak.ding update distance up to 0.02 and value is " <<
    //     distance; last_tracking_to_map = current_tracking_to_map; return;
    // }
    // else if (distance < 0.04)
    // {
    //     LOG(INFO) << "Peak.ding update distance up to 0.03 and value is " <<
    //     distance; last_tracking_to_map = current_tracking_to_map; return;
    // }
    // else if (distance < 0.05)
    // {
    //     LOG(INFO) << "Peak.ding update distance up to 0.04 and value is " <<
    //     distance; last_tracking_to_map = current_tracking_to_map; return;
    // }
    // else
    // {
    //     LOG(INFO) << "Peak.ding update distance up to 0.05 and value is " <<
    //     distance;
    //   }
  }

  last_tracking_to_map = current_tracking_to_map;

  if (!trajectories_last_tracking_to_map.count(last_trajectory_id)) {
    trajectories_last_tracking_to_map.emplace(last_trajectory_id,
                                              last_tracking_to_map);
  } else {
    trajectories_last_tracking_to_map.at(last_trajectory_id) =
        last_tracking_to_map;
  }
}

void Node::PublishTrajectoryNodeList(
    const ::ros::WallTimerEvent& unused_timer_event) {
  if (trajectory_node_list_publisher_.getNumSubscribers() > 0) {
    absl::MutexLock lock(&mutex_);
    trajectory_node_list_publisher_.publish(
        map_builder_bridge_.GetTrajectoryNodeList());
  }
}

void Node::PublishLandmarkPosesList(
    const ::ros::WallTimerEvent& unused_timer_event) {
  if (landmark_poses_list_publisher_.getNumSubscribers() > 0) {
    absl::MutexLock lock(&mutex_);
    landmark_poses_list_publisher_.publish(
        map_builder_bridge_.GetLandmarkPosesList());
  }
}

void Node::PublishConstraintList(
    const ::ros::WallTimerEvent& unused_timer_event) {
  if (constraint_list_publisher_.getNumSubscribers() > 0) {
    absl::MutexLock lock(&mutex_);
    constraint_list_publisher_.publish(map_builder_bridge_.GetConstraintList());
  }
}

// okagv
void Node::CheckAndRunOKagvOrder(const ::ros::WallTimerEvent& timer_event) {

  using OKagv_Order = ::cartographer::mapping::PoseGraphInterface::OKagvOrder;
  absl::MutexLock lock(&mutex_);

  auto order = map_builder_bridge_.GetOKagv_Order();
  
  ::cartographer_ros_msgs::StartTrajectory::Request start_trajectory_request;
  ::cartographer_ros_msgs::StartTrajectory::Response start_trajectory_response;

  ::cartographer_ros_msgs::WriteState::Request save_trajectory_request;
  ::cartographer_ros_msgs::WriteState::Response save_trajectory_response;

  //request.trajectory_id = 1001;

  //return;

  switch (order) {
    case OKagv_Order::FINISH:
      LOG(INFO) << "Current OKagv Order is FINISH";
      map_builder_bridge_.SetOKagv_Order(OKagv_Order::WAIT);
      break;
    case OKagv_Order::START:
      LOG(INFO) << "Current OKagv Order is START";
      map_builder_bridge_.GetOkagvOrderStartTrajectoryRequest(start_trajectory_request);
      HandleStartTrajectory(start_trajectory_request, start_trajectory_response);
      map_builder_bridge_.SetOKagv_Order(OKagv_Order::WAIT);
      break;
    case OKagv_Order::SAVE:
      LOG(INFO) << "Current OKagv Order is SAVE";
      map_builder_bridge_.GetOkagvOrderSaveTrajectoryRequest(save_trajectory_request);
      HandleWriteState(save_trajectory_request,save_trajectory_response);
      map_builder_bridge_.SetOKagv_Order(OKagv_Order::WAIT);
      break;
    case OKagv_Order::WAIT:
       //LOG(INFO) << "Current OKagv Order is WAIT";
       //map_builder_bridge_.SetOKagv_Order(OKagv_Order::WAIT);
       //map_builder_bridge_.GetThreadPoolState();
      break;
    case OKagv_Order::LOCALIZE:
      break;
    default:
      break;

  }

  return;
}

void Node::StartRelocalizeRobot() {
  while (ros::ok()) {
    if (is_start_robot_relocalizaton == true) {
        //LOG(WARNING) << "peak.ding Wait time_delay_for_relocalization";
        absl::SleepFor(absl::Seconds(node_options_.time_delay_for_relocalization));
        //LOG(WARNING) << "peak.ding StartRelocalizeRobot";
      cartographer_ros_msgs::LocalizeTrajectory::Response
          relocalization_response;
      HandleLocalizeTrajectoryDetail(relocalization_request, relocalization_response);
      is_start_robot_relocalizaton = false;
    }

    absl::SleepFor(absl::Seconds(0.1));
  }
}

std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>
Node::ComputeExpectedSensorIds(const TrajectoryOptions& options) const {
  using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;
  using SensorType = SensorId::SensorType;
  std::set<SensorId> expected_topics;
  // Subscribe to all laser scan, multi echo laser scan, and point cloud topics.
  for (const std::string& topic :
       ComputeRepeatedTopicNames(kLaserScanTopic, options.num_laser_scans)) {
    expected_topics.insert(SensorId{SensorType::RANGE, topic});
  }
  for (const std::string& topic : ComputeRepeatedTopicNames(
           kMultiEchoLaserScanTopic, options.num_multi_echo_laser_scans)) {
    expected_topics.insert(SensorId{SensorType::RANGE, topic});
  }
  for (const std::string& topic :
       ComputeRepeatedTopicNames(kPointCloud2Topic, options.num_point_clouds)) {
    expected_topics.insert(SensorId{SensorType::RANGE, topic});
  }
  // For 2D SLAM, subscribe to the IMU if we expect it. For 3D SLAM, the IMU is
  // required.
  if (node_options_.map_builder_options.use_trajectory_builder_3d() ||
      (node_options_.map_builder_options.use_trajectory_builder_2d() &&
       options.trajectory_builder_options.trajectory_builder_2d_options()
           .use_imu_data())) {
    expected_topics.insert(SensorId{SensorType::IMU, kImuTopic});
  }
  // Odometry is optional.
  if (options.use_odometry) {
    expected_topics.insert(SensorId{SensorType::ODOMETRY, kOdometryTopic});
  }
  // NavSatFix is optional.
  if (options.use_nav_sat) {
    expected_topics.insert(
        SensorId{SensorType::FIXED_FRAME_POSE, kNavSatFixTopic});
  }
  // Landmark is optional.
  if (options.use_landmarks) {
    expected_topics.insert(SensorId{SensorType::LANDMARK, kLandmarkTopic});
  }
  return expected_topics;
}

int Node::AddTrajectory(const TrajectoryOptions& options) {
  const std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>
      expected_sensor_ids = ComputeExpectedSensorIds(options);

  const int trajectory_id =
      map_builder_bridge_.AddTrajectory(expected_sensor_ids, options);

  AddExtrapolator(trajectory_id, options);
  AddSensorSamplers(trajectory_id, options);
  LaunchSubscribers(options, trajectory_id);
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(kTopicMismatchCheckDelaySec),
      &Node::MaybeWarnAboutTopicMismatch, this, /*oneshot=*/true));
  for (const auto& sensor_id : expected_sensor_ids) {
    subscribed_topics_.insert(sensor_id.id);
  }
  return trajectory_id;
}

int Node::AddTrajectoryWithId(const int trajectory_id,
                              const TrajectoryOptions& options) {
  const std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>
      expected_sensor_ids = ComputeExpectedSensorIds(options);

  map_builder_bridge_.AddTrajectoryWithId(trajectory_id, expected_sensor_ids,
                                          options);

  AddExtrapolator(trajectory_id, options);
  AddSensorSamplers(trajectory_id, options);
  LaunchSubscribers(options, trajectory_id);
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(kTopicMismatchCheckDelaySec),
      &Node::MaybeWarnAboutTopicMismatch, this, /*oneshot=*/true));
  for (const auto& sensor_id : expected_sensor_ids) {
    subscribed_topics_.insert(sensor_id.id);
  }
  return trajectory_id;
}

void Node::LaunchSubscribers(const TrajectoryOptions& options,
                             const int trajectory_id) {
  
  //okagv， reflag the laser state;
  is_scan_recived = false;
  laser_scan_state_log = cartographer_ros_msgs::LaserScanStates::OK;

  for (const std::string& topic :
       ComputeRepeatedTopicNames(kLaserScanTopic, options.num_laser_scans)) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::LaserScan>(
             &Node::HandleLaserScanMessage, trajectory_id, topic, &node_handle_,
             this),
         topic});
  }
  for (const std::string& topic : ComputeRepeatedTopicNames(
           kMultiEchoLaserScanTopic, options.num_multi_echo_laser_scans)) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::MultiEchoLaserScan>(
             &Node::HandleMultiEchoLaserScanMessage, trajectory_id, topic,
             &node_handle_, this),
         topic});
  }
  for (const std::string& topic :
       ComputeRepeatedTopicNames(kPointCloud2Topic, options.num_point_clouds)) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::PointCloud2>(
             &Node::HandlePointCloud2Message, trajectory_id, topic,
             &node_handle_, this),
         topic});
  }

  // For 2D SLAM, subscribe to the IMU if we expect it. For 3D SLAM, the IMU is
  // required.
  if (node_options_.map_builder_options.use_trajectory_builder_3d() ||
      (node_options_.map_builder_options.use_trajectory_builder_2d() &&
       options.trajectory_builder_options.trajectory_builder_2d_options()
           .use_imu_data())) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::Imu>(&Node::HandleImuMessage,
                                                trajectory_id, kImuTopic,
                                                &node_handle_, this),
         kImuTopic});
  }

  if (options.use_odometry) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<nav_msgs::Odometry>(&Node::HandleOdometryMessage,
                                                  trajectory_id, kOdometryTopic,
                                                  &node_handle_, this),
         kOdometryTopic});
  }
  if (options.use_nav_sat) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::NavSatFix>(
             &Node::HandleNavSatFixMessage, trajectory_id, kNavSatFixTopic,
             &node_handle_, this),
         kNavSatFixTopic});
  }
  if (options.use_landmarks) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<cartographer_ros_msgs::LandmarkList>(
             &Node::HandleLandmarkMessage, trajectory_id, kLandmarkTopic,
             &node_handle_, this),
         kLandmarkTopic});
  }
}

bool Node::ValidateTrajectoryOptions(const TrajectoryOptions& options) {
  if (node_options_.map_builder_options.use_trajectory_builder_2d()) {
    return options.trajectory_builder_options
        .has_trajectory_builder_2d_options();
  }
  if (node_options_.map_builder_options.use_trajectory_builder_3d()) {
    return options.trajectory_builder_options
        .has_trajectory_builder_3d_options();
  }
  return false;
}

bool Node::ValidateTopicNames(const TrajectoryOptions& options) {
  for (const auto& sensor_id : ComputeExpectedSensorIds(options)) {
    const std::string& topic = sensor_id.id;
    if (subscribed_topics_.count(topic) > 0) {
      LOG(ERROR) << "Topic name [" << topic << "] is already used.";
      return false;
    }
  }
  return true;
}

cartographer_ros_msgs::StatusResponse Node::TrajectoryStateToStatus(
    const int trajectory_id, const std::set<TrajectoryState>& valid_states) {
  const auto trajectory_states = map_builder_bridge_.GetTrajectoryStates();
  cartographer_ros_msgs::StatusResponse status_response;

  const auto it = trajectory_states.find(trajectory_id);
  if (it == trajectory_states.end()) {
    status_response.message =
        absl::StrCat("Trajectory ", trajectory_id, " doesn't exist.");
    status_response.code = cartographer_ros_msgs::StatusCode::NOT_FOUND;
    return status_response;
  }

  status_response.message =
      absl::StrCat("Trajectory ", trajectory_id, " is in '",
                   TrajectoryStateToString(it->second), "' state.");
  status_response.code =
      valid_states.count(it->second)
          ? cartographer_ros_msgs::StatusCode::OK
          : cartographer_ros_msgs::StatusCode::INVALID_ARGUMENT;
  return status_response;
}

cartographer_ros_msgs::StatusResponse Node::FinishTrajectoryUnderLock(
    const int trajectory_id) {
  cartographer_ros_msgs::StatusResponse status_response;
  if (trajectories_scheduled_for_finish_.count(trajectory_id)) {
    status_response.message = absl::StrCat("Trajectory ", trajectory_id,
                                           " already pending to finish.");
    status_response.code = cartographer_ros_msgs::StatusCode::OK;
    LOG(INFO) << status_response.message;
    return status_response;
  }

  // okagv
  
  status_response = TrajectoryStateToStatus(
      trajectory_id, {TrajectoryState::FINISHED, TrajectoryState::FROZEN, TrajectoryState::IDLE} /* valid states */);
  if (status_response.code == cartographer_ros_msgs::StatusCode::OK) {
    status_response.code = cartographer_ros_msgs::StatusCode::OK;
    return status_response;
  }
  

  // First, check if we can actually finish the trajectory.
  status_response = TrajectoryStateToStatus(
      trajectory_id, {TrajectoryState::ACTIVE} /* valid states */);
  if (status_response.code != cartographer_ros_msgs::StatusCode::OK) {
    LOG(WARNING) << "Can't finish trajectory: " << status_response.message;
    return status_response;
  }

  // Shutdown the subscribers of this trajectory.
  // A valid case with no subscribers is e.g. if we just visualize states.
  if (subscribers_.count(trajectory_id)) {
    for (auto& entry : subscribers_[trajectory_id]) {
      entry.subscriber.shutdown();
      subscribed_topics_.erase(entry.topic);
      LOG(INFO) << "Shutdown the subscriber of [" << entry.topic << "]";
    }
    CHECK_EQ(subscribers_.erase(trajectory_id), 1);
  }

  trajectories_scheduled_for_finish_.emplace(trajectory_id);
  map_builder_bridge_.FinishTrajectory(trajectory_id);
  trajectories_scheduled_for_finish_.erase(trajectory_id);
  status_response.message = "Success";
  // absl::StrCat("Finished trajectory ", trajectory_id, ".");
  status_response.code = cartographer_ros_msgs::StatusCode::OK;
  return status_response;
}

bool Node::HandleStartTrajectory(
    ::cartographer_ros_msgs::StartTrajectory::Request& request,
    ::cartographer_ros_msgs::StartTrajectory::Response& response) {
    //absl::MutexLock lock(&mutex_);
  if (is_service_done == true) {
    is_service_done = false;
    //
    absl::SleepFor(absl::Seconds(0.1));
  } else {
    response.status.code = cartographer_ros_msgs::StatusCode::UNIMPLEMENTED;
    response.status.message = "another service is running, please wait";
    // is_service_done = f;
    // current_trajectory_type = TrajectoryType::IDLE;
    return true;
  }

  switch (current_trajectory_type) {
    case TrajectoryType::SLAM:
      LOG(INFO) << "HandleStartTrajectory SLAM";

        // finish and delete current trajectory_id;
        if (!FinishAndDeleteTrajectory(last_trajectory_id_name)) {
          response.status.code = cartographer_ros_msgs::StatusCode::ABORTED;
          response.status.message =
              "last trajectory is in slam mode, and remove it fail";
          return true;
        }
      /*
      if (request.use_initial_pose == true) {
        // finish and delete current trajectory_id;
        if (!FinishAndDeleteTrajectory(last_trajectory_id_name)) {
          response.status.code = cartographer_ros_msgs::StatusCode::ABORTED;
          response.status.message =
              "last trajectory is in slam mode, and remove it fail";
          return true;
        }
      } else {
        if (!trajectory_id_toint.count(last_trajectory_id_name)) {
          response.status.code = cartographer_ros_msgs::StatusCode::ABORTED;
          response.status.message = "no trajectory found, and finish it fail";
          return true;
        }
        if (!FinishTrajectory(trajectory_id_toint[last_trajectory_id_name])) {
          response.status.code = cartographer_ros_msgs::StatusCode::ABORTED;
          response.status.message =
              "last trajectory is in slam mode, and finish it fail";
          return true;
        }
      }
      */
      break;
    case TrajectoryType::NAVIGATION:
          LOG(INFO) << "HandleStartTrajectory NAVIGATION";
      // finish and delete current navigation trajectory_id;
      if (!FinishAndDeleteTrajectory(last_trajectory_id_name)) {
        response.status.code = cartographer_ros_msgs::StatusCode::ABORTED;
        response.status.message =
            "last trajectory is in navigation mode, and remove it fail";
        return true;
      }

      // finsih and delete current map trajectory_id
      if (!FinishAndDeleteTrajectory(last_map_trajectory_id_name)) {
        response.status.code = cartographer_ros_msgs::StatusCode::ABORTED;
        response.status.message =
            "last trajectory is in navigation mode, and remove it fail";
        return true;
      }
      break;
    case TrajectoryType::IDLE:
      LOG(INFO) << "HandleStartTrajectory IDLE";
      // finsih and delete current map trajectory_id
      LOG(INFO) << "last_map_trajectory_id_name " << last_map_trajectory_id_name;

      if (request.use_initial_pose == true) break;

      if (!last_trajectory_id_name.empty()) {
        if (!FinishAndDeleteTrajectory(last_map_trajectory_id_name)) {
          response.status.code = cartographer_ros_msgs::StatusCode::ABORTED;
          response.status.message = "has last trajectory but remove it fail";
          return true;
        }
      }

      break;
    default:
      break;
  }


  if(request.use_initial_pose == true)
  {
    request.relative_to_trajectory_id = last_trajectory_id_name;
    request.initial_pose = ToGeometryMsgPose(current_tracking_to_map);
  }

  // start new slam
  HandleStartTrajectoryDetail(request, response);

  if (response.status.code == cartographer_ros_msgs::StatusCode::OK) {
    current_trajectory_type = TrajectoryType::SLAM;
    LOG(INFO) << "Success to start Trajectory";

    cartographer_ros_msgs::Rosbag order;
    order.config = "standard";
    order.bag_name = request.trajectory_id;
    rosbag_start_recorder_.publish(order);

    OKagv_Feedback feedback;
    feedback.code = 0;
    feedback.state = OKagv_State::SUCCESS;
    map_builder_bridge_.SetOKagv_Feedback(feedback);

  } else {
    current_trajectory_type = TrajectoryType::ABORTION;
    LOG(INFO) << "Fail to start Trajectory";
    
    OKagv_Feedback feedback;
    feedback.code = 1;
    feedback.state = OKagv_State::SUCCESS;
    feedback.message = "Fail to start Trajectory";

    map_builder_bridge_.SetOKagv_Feedback(feedback);
    
  }

  absl::SleepFor(absl::Seconds(node_options_.time_delay_for_start_trajectory));
  is_service_done = true;
  return true;
}

bool Node::HandleStartTrajectoryDetail(
    ::cartographer_ros_msgs::StartTrajectory::Request& request,
    ::cartographer_ros_msgs::StartTrajectory::Response& response) {
  TrajectoryOptions trajectory_options;
  std::string configuration_basename_;

  if (request.trajectory_type == "slam") {
    configuration_basename_ = configuration_slam_basename_;
  } else if (request.trajectory_type == "navigation") {
    configuration_basename_ = configuration_navigation_basename_;
  } else {
    response.status.message = "Failed. trajectory_type don't exist";
    return true;
  }

  std::tie(std::ignore, trajectory_options) =
      LoadOptions(configuration_directory_, configuration_basename_);

  if (configuration_directory_.empty() || configuration_basename_.empty()) {
    response.status.message = "configuration file do not find";
    return true;
  } else {
    // LOG(INFO) << "Peak.ding check configuration_directory_ "
    //          << configuration_directory_;
    // LOG(INFO) << "Peak.ding check configuration_basename_ "
    //          << configuration_basename_;
  }

    //
  int scheduled_trajectory_id = 0;
  std::string scheduled_trajectory_id_name;

  auto it = trajectory_id_toint.find(request.trajectory_id);
  if (it == trajectory_id_toint.end()) {
    // okagv
    if (request.trajectory_type == "slam") {
      scheduled_trajectory_id = trajectory_id_toint.size();
      scheduled_trajectory_id_name = request.trajectory_id;
      if (scheduled_trajectory_id_name.empty())
        scheduled_trajectory_id_name = std::to_string(scheduled_trajectory_id);
      scheduled_trajectory_type = TrajectoryType::SLAM;
      // set slam param
      trajectory_options.trajectory_builder_options.set_pure_localization(
          false);
      
      //okagv
      LOG(INFO) << "Peak.ding SetMapBuilderOptions slam_option_";
      map_builder_bridge_.SetMapBuilderOptions(slam_option_);

    } else if (request.trajectory_type == "navigation") {
      scheduled_trajectory_id = 1001;
      scheduled_trajectory_id_name = "navigation";
      scheduled_trajectory_type = TrajectoryType::NAVIGATION;
      // set navigation param
      trajectory_options.trajectory_builder_options.set_pure_localization(true);

      // okagv
      map_builder_bridge_.SetMapBuilderOptions(navigation_option_);

    } else {
      response.status.message = "Failed. trajectory_type don't exist";
      return true;
    }

  } else {
    scheduled_trajectory_id = it->second;
  }

  if (request.use_initial_pose) {
    const auto pose = ToRigid3d(request.initial_pose);
    if (!pose.IsValid()) {
      response.status.message =
          "Invalid pose argument. Orientation quaternion must be normalized.";
      LOG(ERROR) << response.status.message;
      response.status.code =
          cartographer_ros_msgs::StatusCode::INVALID_ARGUMENT;
          LOG(ERROR) << "intial pose is wrong";
      return true;
    }

    // Check if the requested trajectory for the relative initial pose exists.
    if (!trajectory_id_toint.count(request.relative_to_trajectory_id)) {
      response.status.code =
          cartographer_ros_msgs::StatusCode::INVALID_ARGUMENT;
      LOG(ERROR) << "relative_to_trajectory_id is "
                 << request.relative_to_trajectory_id
                 << ", do not exist map trajectory_id";
      return true;
    }

    int relative_to_trajectory_id =
        trajectory_id_toint.at(request.relative_to_trajectory_id);
        //LOG(INFO) << "Peak.ding relative_to_trajectory_id " << relative_to_trajectory_id; 
    response.status = TrajectoryStateToStatus(
        relative_to_trajectory_id,
        {TrajectoryState::ACTIVE, TrajectoryState::FROZEN,
         TrajectoryState::FINISHED, TrajectoryState::IDLE} /* valid states */);
    if (response.status.code != cartographer_ros_msgs::StatusCode::OK) {
      LOG(ERROR) << "Can't start a trajectory with initial pose: "
                 << response.status.message;
      return true;
    }

    ::cartographer::mapping::proto::InitialTrajectoryPose
        initial_trajectory_pose;
    initial_trajectory_pose.set_to_trajectory_id(relative_to_trajectory_id);
    *initial_trajectory_pose.mutable_relative_pose() =
        cartographer::transform::ToProto(pose);
    initial_trajectory_pose.set_timestamp(cartographer::common::ToUniversal(
        ::cartographer_ros::FromRos(ros::Time(0))));
    *trajectory_options.trajectory_builder_options
         .mutable_initial_trajectory_pose() = initial_trajectory_pose;
  }

  // First, check if we can actually start the trajectory.
  cartographer_ros_msgs::StatusResponse status_response =
      TrajectoryStateToStatus(scheduled_trajectory_id, {} /* valid states */);
  if (status_response.code != cartographer_ros_msgs::StatusCode::NOT_FOUND) {
    LOG(INFO) << "Peak.ding scheduled_trajectory_id " << scheduled_trajectory_id;
    LOG(ERROR) << "Can't start trajectory that is exist ";

    response.status.code = cartographer_ros_msgs::StatusCode::ALREADY_EXISTS;
    response.status.message = "trajectory id already exist";
    return true;
  }

  if (!ValidateTrajectoryOptions(trajectory_options)) {
    response.status.message = "Invalid trajectory options.";
    LOG(ERROR) << response.status.message;
    response.status.code = cartographer_ros_msgs::StatusCode::INVALID_ARGUMENT;
  } else if (!ValidateTopicNames(trajectory_options)) {
    response.status.message = "Topics are already used by another trajectory.";
    LOG(ERROR) << response.status.message;
    response.status.code = cartographer_ros_msgs::StatusCode::INVALID_ARGUMENT;
  } else {
    response.status.message = "Success";
    // okagv
    SetTrajectoryTypeWithId(request.trajectory_type, scheduled_trajectory_id);
    response.trajectory_id =
        AddTrajectoryWithId(scheduled_trajectory_id, trajectory_options);
    response.status.code = cartographer_ros_msgs::StatusCode::OK;

    //okagv
    RegisterClientIdForTrajectory(scheduled_trajectory_id,scheduled_trajectory_id_name);
    // okagv
    last_trajectory_id = scheduled_trajectory_id;
    last_trajectory_id_name = scheduled_trajectory_id_name;
    trajectory_id_toint.emplace(last_trajectory_id_name,
                                last_trajectory_id);
    current_trajectory_type = scheduled_trajectory_type;

  }
  return true;
}

void Node::StartTrajectoryWithDefaultTopics(const TrajectoryOptions& options) {
  absl::MutexLock lock(&mutex_);
  CHECK(ValidateTrajectoryOptions(options));
  AddTrajectory(options);
}

std::vector<
    std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>>
Node::ComputeDefaultSensorIdsForMultipleBags(
    const std::vector<TrajectoryOptions>& bags_options) const {
  using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;
  std::vector<std::set<SensorId>> bags_sensor_ids;
  for (size_t i = 0; i < bags_options.size(); ++i) {
    std::string prefix;
    if (bags_options.size() > 1) {
      prefix = "bag_" + std::to_string(i + 1) + "_";
    }
    std::set<SensorId> unique_sensor_ids;
    for (const auto& sensor_id : ComputeExpectedSensorIds(bags_options.at(i))) {
      unique_sensor_ids.insert(SensorId{sensor_id.type, prefix + sensor_id.id});
    }
    bags_sensor_ids.push_back(unique_sensor_ids);
  }
  return bags_sensor_ids;
}

int Node::AddOfflineTrajectory(
    const std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>&
        expected_sensor_ids,
    const TrajectoryOptions& options) {
  absl::MutexLock lock(&mutex_);
  const int trajectory_id =
      map_builder_bridge_.AddTrajectory(expected_sensor_ids, options);
  AddExtrapolator(trajectory_id, options);
  AddSensorSamplers(trajectory_id, options);
  return trajectory_id;
}

bool Node::HandleGetTrajectoryStates(
    ::cartographer_ros_msgs::GetTrajectoryStates::Request& request,
    ::cartographer_ros_msgs::GetTrajectoryStates::Response& response) {
  using TrajectoryState =
      ::cartographer::mapping::PoseGraphInterface::TrajectoryState;
  absl::MutexLock lock(&mutex_);
  response.status.code = ::cartographer_ros_msgs::StatusCode::OK;
  response.trajectory_states.header.stamp = ros::Time::now();
  for (const auto& entry : map_builder_bridge_.GetTrajectoryStates()) {
    response.trajectory_states.trajectory_id.push_back(entry.first);
    switch (entry.second) {
      case TrajectoryState::ACTIVE:
        response.trajectory_states.trajectory_state.push_back(
            ::cartographer_ros_msgs::TrajectoryStates::ACTIVE);
        break;
      case TrajectoryState::FINISHED:
        response.trajectory_states.trajectory_state.push_back(
            ::cartographer_ros_msgs::TrajectoryStates::FINISHED);
        break;
      case TrajectoryState::FROZEN:
        response.trajectory_states.trajectory_state.push_back(
            ::cartographer_ros_msgs::TrajectoryStates::FROZEN);
        break;
      case TrajectoryState::DELETED:
        response.trajectory_states.trajectory_state.push_back(
            ::cartographer_ros_msgs::TrajectoryStates::DELETED);
        break;
      case TrajectoryState::IDLE:
        response.trajectory_states.trajectory_state.push_back(
            ::cartographer_ros_msgs::TrajectoryStates::IDLE);
        break;
    }
  }
  return true;
}



bool Node::HandleWriteState(
    ::cartographer_ros_msgs::WriteState::Request& request,
    ::cartographer_ros_msgs::WriteState::Response& response) {
  //absl::MutexLock lock(&mutex_);

  // Step-1 check current state
  LOG(INFO) << "step-1 : check current trajectory state";
  if (is_service_done == true) {
    is_service_done = false;
    //
    absl::SleepFor(absl::Seconds(0.1));
    //ros::Duration(0.1).sleep();

  } else {
    response.status.code = cartographer_ros_msgs::StatusCode::UNIMPLEMENTED;
    response.status.message = "another service is running, please wait";
    is_service_done = true;
    current_trajectory_type = TrajectoryType::IDLE;
    return true;
  }
    OKagv_Feedback feedback;
    switch (current_trajectory_type) {
    case TrajectoryType::SLAM:
         LOG(INFO) << "HandleWriteState SLAM";
         //step-1: //finish and save it
         if(!FinishAndSaveTrajectory(last_trajectory_id_name))
         {
           LOG(INFO) << "Peak.ding error occur when save map";
           response.status.code = cartographer_ros_msgs::StatusCode::ABORTED;
           response.status.message = "error occur when save map";

           feedback.code = 1;
           feedback.message = "error occur when save map";
           feedback.state = OKagv_State::FAIL;

           map_builder_bridge_.SetOKagv_Feedback(feedback);
           //return true;
         }
         else
         {
           LOG(INFO) << "Peak.ding save map success";
           response.status.code = cartographer_ros_msgs::StatusCode::OK;
           response.status.message = "save map success";
           current_trajectory_type = TrajectoryType::IDLE;
           last_map_trajectory_id_name = last_trajectory_id_name; //request.filename;
           
           feedback.code = 0;
           feedback.state = OKagv_State::SUCCESS;

           map_builder_bridge_.SetOKagv_Feedback(feedback);
           //return true;
         }
      break;
    case TrajectoryType::NAVIGATION:
           LOG(INFO) << "HandleWriteState NAVIGATION";
           response.status.code = cartographer_ros_msgs::StatusCode::ABORTED;
           response.status.message = "current is navigation mode , please slam first";
           
           feedback.code = 1;
           feedback.message = "current is navigation mode , please slam first";
           feedback.state = OKagv_State::FAIL;
           
           map_builder_bridge_.SetOKagv_Feedback(feedback);
           //return true;
      break;
    case TrajectoryType::IDLE:
           LOG(INFO) << "HandleWriteState IDLE";
           response.status.code = cartographer_ros_msgs::StatusCode::ABORTED;
           response.status.message = "no slam trajectory_id found, please slam first";

           feedback.code = 1;
           feedback.message = "no slam trajectory_id found, please slam first";
           feedback.state = OKagv_State::FAIL;
           
           map_builder_bridge_.SetOKagv_Feedback(feedback);
           //return true;
      break;
    default:
      break;
  }

  is_service_done = true;
 
  //stop record;
  std_msgs::String order;
  order.data = "standard";
  rosbag_stop_recorder_.publish(order);

  return true;

}

bool Node::HandleWriteStateDetail(
    cartographer_ros_msgs::WriteState::Request& request,
    cartographer_ros_msgs::WriteState::Response& response) {
  //first check the filedir exist?

  DIR* pDir;
  std::string filedir =
      getenv("HOME") + node_options_.root_file_directory + "/scan";
  if (!(pDir = opendir(filedir.c_str()))) {
    LOG(INFO) << "Folder doesn't Exist! create it";

    if (!createFolder(filedir)) {
      LOG(INFO) << "create filedir failed";
      response.status.code = cartographer_ros_msgs::StatusCode::ABORTED;
      response.status.message = "create filedir failed";
      return true;
    }
    closedir(pDir);
  }


  std::string intact_name = getenv("HOME") + node_options_.root_file_directory + "/scan/" +
                            request.filename + ".pbstream";
  LOG(INFO) << "intact_name: " << intact_name;                          

  auto it = trajectory_id_toint.find(request.filename);
  if(it == trajectory_id_toint.end()) 
  {
    response.status.code = cartographer_ros_msgs::StatusCode::ABORTED;
    response.status.message = "save the map do not in slam list";
    return true;
  }

  int trajectory_to_save = trajectory_id_toint[request.filename];

  if (map_builder_bridge_.SerializeStateWithId(
          intact_name, trajectory_to_save,
          request.include_unfinished_submaps)) {
    response.status.code = cartographer_ros_msgs::StatusCode::OK;
    response.status.message =
        absl::StrCat("State written to '", request.filename, "'.");
  } else {
    response.status.code = cartographer_ros_msgs::StatusCode::INVALID_ARGUMENT;
    response.status.message =
        absl::StrCat("Failed to write '", request.filename, "'.");
    return true;
  }

  // set trajectory state
  is_service_done = true;
  current_trajectory_type = TrajectoryType::IDLE;
  map_builder_bridge_.SetTrajectoryTypeWithId(TrajectoryType::IDLE,
                                              trajectory_to_save);

  response.status.code = cartographer_ros_msgs::StatusCode::OK;
  response.status.message = "save map success";

  return true;
}

bool Node::HandleReadMetrics(
    ::cartographer_ros_msgs::ReadMetrics::Request& request,
    ::cartographer_ros_msgs::ReadMetrics::Response& response) {
  absl::MutexLock lock(&mutex_);
  response.timestamp = ros::Time::now();
  if (!metrics_registry_) {
    response.status.code = cartographer_ros_msgs::StatusCode::UNAVAILABLE;
    response.status.message = "Collection of runtime metrics is not activated.";
    return true;
  }
  metrics_registry_->ReadMetrics(&response);
  response.status.code = cartographer_ros_msgs::StatusCode::OK;
  response.status.message = "Successfully read metrics.";
  return true;
}

void Node::FinishAllTrajectories() {
  absl::MutexLock lock(&mutex_);
  for (const auto& entry : map_builder_bridge_.GetTrajectoryStates()) {
    if (entry.second == TrajectoryState::ACTIVE) {
      const int trajectory_id = entry.first;
      CHECK_EQ(FinishTrajectoryUnderLock(trajectory_id).code,
               cartographer_ros_msgs::StatusCode::OK);
    }
  }
}

bool Node::FinishTrajectory(const int trajectory_id) {
  absl::MutexLock lock(&mutex_);
  return FinishTrajectoryUnderLock(trajectory_id).code ==
         cartographer_ros_msgs::StatusCode::OK;
}

void Node::RunFinalOptimization() {
  {
    for (const auto& entry : map_builder_bridge_.GetTrajectoryStates()) {
      const int trajectory_id = entry.first;
      if (entry.second == TrajectoryState::ACTIVE) {
        LOG(WARNING)
            << "Can't run final optimization if there are one or more active "
               "trajectories. Trying to finish trajectory with ID "
            << std::to_string(trajectory_id) << " now.";
        CHECK(FinishTrajectory(trajectory_id))
            << "Failed to finish trajectory with ID "
            << std::to_string(trajectory_id) << ".";
      }
    }
  }
  // Assuming we are not adding new data anymore, the final optimization
  // can be performed without holding the mutex.
  map_builder_bridge_.RunFinalOptimization();
}

void Node::HandleOdometryMessage(const int trajectory_id,
                                 const std::string& sensor_id,
                                 const nav_msgs::Odometry::ConstPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).odometry_sampler.Pulse()) {
    return;
  }
  auto sensor_bridge_ptr = map_builder_bridge_.sensor_bridge(trajectory_id);
  auto odometry_data_ptr = sensor_bridge_ptr->ToOdometryData(msg);
  if (odometry_data_ptr != nullptr) {
    extrapolators_.at(trajectory_id).AddOdometryData(*odometry_data_ptr);
  }
  sensor_bridge_ptr->HandleOdometryMessage(sensor_id, msg);
}

void Node::HandleNavSatFixMessage(const int trajectory_id,
                                  const std::string& sensor_id,
                                  const sensor_msgs::NavSatFix::ConstPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).fixed_frame_pose_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_.sensor_bridge(trajectory_id)
      ->HandleNavSatFixMessage(sensor_id, msg);
}

void Node::HandleLandmarkMessage(
    const int trajectory_id, const std::string& sensor_id,
    const cartographer_ros_msgs::LandmarkList::ConstPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).landmark_sampler.Pulse()) {
    return;
  }

  CHECK(msg->landmarks.size() != 0) << "Landmarks has no item";
  //okagv QRcode
  
  if (msg->landmarks[0].type == "qrcode") {
    map_builder_bridge_.sensor_bridge(trajectory_id)
        ->HandleQRCodeLandmarkMessage(sensor_id, msg);

    //LOG(INFO) << "Peak.ding use qrcode";
    geometry_msgs::TransformStamped stamped_transform;

    stamped_transform.header.stamp = ros::Time::now();
    stamped_transform.header.frame_id = "base_link";
    stamped_transform.child_frame_id = "QR_Code";
    stamped_transform.transform = ToGeometryMsgTransform(
        ToRigid3d(msg->landmarks[0].tracking_from_landmark_transform)
            .inverse());

    tf_broadcaster_.sendTransform(stamped_transform);
  } else if (msg->landmarks[0].type == "reflector") {
    // okagv reflector
    map_builder_bridge_.sensor_bridge(trajectory_id)
        ->HandleReflectorLandmarkMessage(sensor_id, current_tracking_to_map,
                                         msg);
  }
  else if(msg->landmarks[0].type == "reflector_combined")
  {
        map_builder_bridge_.sensor_bridge(trajectory_id)
        ->HandleReflectorCombinedLandmarkMessage(sensor_id, msg);
    
    geometry_msgs::TransformStamped stamped_transform;

    stamped_transform.header.stamp = ros::Time::now();
    stamped_transform.header.frame_id = "car_laser";
    stamped_transform.child_frame_id = "landmark_1";
    stamped_transform.transform = ToGeometryMsgTransform(
        ToRigid3d(msg->landmarks[0].tracking_from_landmark_transform));

    tf_broadcaster_.sendTransform(stamped_transform);
    
  }
  else if(msg->landmarks[0].type == "apriltag")
  {
    map_builder_bridge_.sensor_bridge(trajectory_id)
        ->HandleLandmarkMessage(sensor_id, msg);
    //LOG(INFO) << "Peak.ding use qrcode";
    geometry_msgs::TransformStamped stamped_transform;

    stamped_transform.header.stamp = ros::Time::now();
    stamped_transform.header.frame_id = "camera_rgb_optical_frame";
    stamped_transform.child_frame_id = "apriltag";
    stamped_transform.transform = ToGeometryMsgTransform(
        ToRigid3d(msg->landmarks[0].tracking_from_landmark_transform));

    tf_broadcaster_.sendTransform(stamped_transform);
  }
  return;
}

void Node::HandleImuMessage(const int trajectory_id,
                            const std::string& sensor_id,
                            const sensor_msgs::Imu::ConstPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).imu_sampler.Pulse()) {
    return;
  }
  auto sensor_bridge_ptr = map_builder_bridge_.sensor_bridge(trajectory_id);
  auto imu_data_ptr = sensor_bridge_ptr->ToImuData(msg);
  if (imu_data_ptr != nullptr) {
    extrapolators_.at(trajectory_id).AddImuData(*imu_data_ptr);
  }
  sensor_bridge_ptr->HandleImuMessage(sensor_id, msg);
}

void Node::HandleLaserScanMessage(const int trajectory_id,
                                  const std::string& sensor_id,
                                  const sensor_msgs::LaserScan::ConstPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
    return;
  }

  //Peak.ding test the laser scan data missing problem
  if (is_scan_recived == true) {
    ros::Duration duration = msg->header.stamp - last_scan_time;
    if (duration.toSec() > 1.0) {
      LOG(WARNING) << "Peak.ding last time " << last_scan_time;
      LOG(WARNING) << "Peak.ding current time " << msg->header.stamp;
      laser_scan_state_log = cartographer_ros_msgs::LaserScanStates::MISSING;
    }
    
  }

  last_scan_time = msg->header.stamp;
  is_scan_recived = true;

  map_builder_bridge_.sensor_bridge(trajectory_id)
      ->HandleLaserScanMessage(sensor_id, msg);
}

void Node::HandleMultiEchoLaserScanMessage(
    const int trajectory_id, const std::string& sensor_id,
    const sensor_msgs::MultiEchoLaserScan::ConstPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_.sensor_bridge(trajectory_id)
      ->HandleMultiEchoLaserScanMessage(sensor_id, msg);
}

void Node::HandlePointCloud2Message(
    const int trajectory_id, const std::string& sensor_id,
    const sensor_msgs::PointCloud2::ConstPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_.sensor_bridge(trajectory_id)
      ->HandlePointCloud2Message(sensor_id, msg);
}

void Node::SerializeState(const std::string& filename,
                          const bool include_unfinished_submaps) {
  absl::MutexLock lock(&mutex_);
  CHECK(
      map_builder_bridge_.SerializeState(filename, include_unfinished_submaps))
      << "Could not write state.";
}

void Node::LoadState(const std::string& state_filename,
                     const bool load_frozen_state) {
  absl::MutexLock lock(&mutex_);
  map_builder_bridge_.LoadState(state_filename, load_frozen_state);
}

// okagv
void Node::LoadMaps(const std::string& file_directory) {
  std::vector<std::string> map_files;
  GetFileNames(file_directory, map_files);

  for (uint32_t i = 0; i < map_files.size(); i++) {
    //LOG(INFO) << "Peak.ding map_files " << map_files[i];
    trajectory_id_toint.emplace(map_files[i], i);

    std::string whole_map_files = getenv("HOME") + node_options_.root_file_directory + "/map/" +
                                  map_files[i] + ".pbstream";
    map_builder_bridge_.LoadTrajectory(i, whole_map_files,
                                       TrajectoryState::ACTIVE);
  }

  //return;

  if (map_files.size() != 0) {
    ::cartographer_ros_msgs::StartTrajectory::Request request_navigation_;
    ::cartographer_ros_msgs::StartTrajectory::Response response_navigation_;

    request_navigation_.trajectory_type = "navigation";
    HandleStartTrajectory(request_navigation_, response_navigation_);
  }
}

void Node::MaybeWarnAboutTopicMismatch(
    const ::ros::WallTimerEvent& unused_timer_event) {
  ::ros::master::V_TopicInfo ros_topics;
  ::ros::master::getTopics(ros_topics);
  std::set<std::string> published_topics;
  std::stringstream published_topics_string;
  for (const auto& it : ros_topics) {
    std::string resolved_topic = node_handle_.resolveName(it.name, false);
    published_topics.insert(resolved_topic);
    published_topics_string << resolved_topic << ",";
  }
  bool print_topics = false;
  for (const auto& entry : subscribers_) {
    int trajectory_id = entry.first;
    for (const auto& subscriber : entry.second) {
      std::string resolved_topic = node_handle_.resolveName(subscriber.topic);
      if (published_topics.count(resolved_topic) == 0) {
        LOG(WARNING) << "Expected topic \"" << subscriber.topic
                     << "\" (trajectory " << trajectory_id << ")"
                     << " (resolved topic \"" << resolved_topic << "\")"
                     << " but no publisher is currently active.";
        print_topics = true;
      }
    }
  }
  if (print_topics) {
    LOG(WARNING) << "Currently available topics are: "
                 << published_topics_string.str();
  }
}

void Node::HandleInitialPoseMessage(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  ::cartographer::transform::Rigid3d initial_pose(
      {msg->pose.pose.position.x, msg->pose.pose.position.y,
       msg->pose.pose.position.z},
      ToEigen(msg->pose.pose.orientation));

  bool use_initial_pose = true;
  int tarjectory_id = last_map_trajectory_id;
  map_builder_bridge_.SetInitialPose(use_initial_pose, tarjectory_id,
                                     initial_pose);
}

  //okagv
  void Node::RecordQrCodePosition(const cartographer_ros_msgs::LandmarkList::ConstPtr& msg)
  {

    LOG(INFO) << "Peak.ding RecordQrCodePosition";

    for(auto item : msg->landmarks)
    {
       std::string outcomeAds =  getenv("HOME") + node_options_.root_file_directory + "/checkpreicse.txt";
       std::ofstream outfile;
       outfile.open(outcomeAds,std::ios::app);

       double yaw, pitch, roll;
       tf::Matrix3x3(
           tf::Quaternion(item.tracking_from_landmark_transform.orientation.x,
                          item.tracking_from_landmark_transform.orientation.y,
                          item.tracking_from_landmark_transform.orientation.z,
                          item.tracking_from_landmark_transform.orientation.w))
           .getEulerYPR(yaw, pitch, roll);

       outfile << item.id << " "
                          << item.tracking_from_landmark_transform.position.x << " "
                          << item.tracking_from_landmark_transform.position.y << " "
                          //<< roll*180/M_PI << " "
                          //<< pitch*180/M_PI << " "
                          << yaw*180/M_PI << " "
                          << std::endl;
    }
  }




cartographer_ros_msgs::StatusResponse Node::DeleteTrajectoryUnderLock(
    int trajectory_id) {
  cartographer_ros_msgs::StatusResponse status_response;
  if (trajectories_scheduled_for_delete_.count(trajectory_id)) {
    status_response.message = absl::StrCat("Trajectory ", trajectory_id,
                                           " already pending to delete.");
    status_response.code = cartographer_ros_msgs::StatusCode::OK;
    LOG(INFO) << status_response.message;
    return status_response;
  }

  // First, check if we can actually finish the trajectory.
  status_response = TrajectoryStateToStatus(
      trajectory_id, {TrajectoryState::FINISHED, TrajectoryState::FROZEN,
                      TrajectoryState::DELETED, TrajectoryState::IDLE} /* valid states */);
  if (status_response.code != cartographer_ros_msgs::StatusCode::OK) {
    LOG(WARNING) << "Can't delete trajectory: " << status_response.message;
    //return status_response;
  }

  trajectories_scheduled_for_delete_.emplace(trajectory_id);
  map_builder_bridge_.DeleteTrajectory(trajectory_id);
  trajectories_scheduled_for_delete_.erase(trajectory_id);
  status_response.message = "Success";
  // absl::StrCat("Deleted trajectory ", trajectory_id, ".");
  status_response.code = cartographer_ros_msgs::StatusCode::OK;
  return status_response;
}

cartographer_ros_msgs::StatusResponse Node::LocalizeTrajectoryUnderLock(
    int trajectory_id, bool use_initial_pose,
    const geometry_msgs::Pose initial_pose) {
  cartographer_ros_msgs::StatusResponse status_response;
  if (trajectories_scheduled_for_localize_.count(trajectory_id)) {
    status_response.message = absl::StrCat("Trajectory ", trajectory_id,
                                           " already pending to localize.");
    status_response.code = cartographer_ros_msgs::StatusCode::OK;
    LOG(INFO) << status_response.message;
    return status_response;
  }

  // First, check if we can actually localize the trajectory.
  status_response = TrajectoryStateToStatus(
      trajectory_id,
      {TrajectoryState::ACTIVE, TrajectoryState::FINISHED,
       TrajectoryState::FROZEN, TrajectoryState::IDLE} /* valid states */);
  if (status_response.code != cartographer_ros_msgs::StatusCode::OK) {
    LOG(ERROR) << "Can't localize trajectory: " << status_response.message;
    return status_response;
  }

  trajectories_scheduled_for_localize_.emplace(trajectory_id);
  map_builder_bridge_.LocalizeTrajectory(trajectory_id, use_initial_pose,
                                         initial_pose);
  trajectories_scheduled_for_localize_.erase(trajectory_id);
  status_response.message = "Success";
  // absl::StrCat("Localized trajectory ", trajectory_id, ".");
  status_response.code = cartographer_ros_msgs::StatusCode::OK;
  return status_response;
}

void Node::SetConfigurationParam(std::string directory,
                                 std::string slam_basename,
                                 std::string navigation_basename) {
  configuration_directory_ = directory;
  configuration_slam_basename_ = slam_basename;
  configuration_navigation_basename_ = navigation_basename;
}

// okagv
void Node::SetTrajectoryTypeWithId(std::string type, int trajectory_id) {
  if (type == "slam") {
    map_builder_bridge_.SetTrajectoryTypeWithId(TrajectoryType::SLAM,
                                                trajectory_id);
  } else if (type == "navigation") {
    map_builder_bridge_.SetTrajectoryTypeWithId(TrajectoryType::NAVIGATION,
                                                trajectory_id);
  } else if (type == "relocalization") {
    map_builder_bridge_.SetTrajectoryTypeWithId(TrajectoryType::RELOCALIZAION,
                                                trajectory_id);
  } else {
    return;
  }
}

// okagv
void Node::GetFileNames(std::string path, std::vector<std::string>& filenames) {
  DIR* pDir;
  struct dirent* ptr;
  if (!(pDir = opendir(path.c_str()))) {
    std::cout << "Folder doesn't Exist!" << std::endl;
    return;
  }
  while ((ptr = readdir(pDir)) != 0) {
    if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0) {
      // okagv check the last suffix
      const std::string suffix = ".pbstream";
      std::string state_filename = ptr->d_name;
      if (state_filename.substr(std::max<int>(
              state_filename.size() - suffix.size(), 0)) != suffix)
        continue;

      std::string name_no_suffix =
          state_filename.substr(0, state_filename.size() - suffix.size());
      filenames.push_back(name_no_suffix);
    }
  }
  closedir(pDir);
}

void Node::init_grpc_server() {
  //async_grpc::Server::Builder server_builder;
  // server_builder.SetServerAddress(map_builder_bridge_.map_builder())

  //grpc_server_ = server_builder.Build();
  //grpc_server_->SetExecutionContext(std::make_unique<NodeContext>(this));

}

void Node::RegisterClientIdForTrajectory(int trajectory_id, std::string trajectory_name)
{
  map_builder_bridge_.RegisterClientIdForTrajectory(trajectory_id, trajectory_name);
}

// okagv
bool Node::HandleScanQualityQuery(
    ::cartographer_ros_msgs::ScanQualityQuery::Request& request,
    ::cartographer_ros_msgs::ScanQualityQuery::Response& response) {
  if (laser_scan_state_log != cartographer_ros_msgs::LaserScanStates::OK) {
    response.status.code = cartographer_ros_msgs::LaserScanStates::MISSING;
    response.status.message = "missing scan data occur!";

  } else {
    response.status.code = cartographer_ros_msgs::LaserScanStates::OK;
    response.status.message = "scan data is OK";
  }

  return true;
}

bool Node::FinishAndSaveTrajectory(std::string currentTrajectoryId) {
  // step-1 finish current active slam trajectory first
  cartographer_ros_msgs::FinishTrajectory::Request finish_request;
  cartographer_ros_msgs::FinishTrajectory::Response finish_response;
  finish_request.trajectory_id = currentTrajectoryId;
  HandleFinishTrajectoryDetail(finish_request, finish_response);
  if (finish_response.status.code != cartographer_ros_msgs::StatusCode::OK) {
    LOG(INFO) << finish_response.status.message;
    return false;
  }

  // step-2 wait time
  absl::SleepFor(absl::Seconds(node_options_.time_delay_for_finish_trajectory));

  // step-3 save it
  cartographer_ros_msgs::WriteState::Request save_request;
  cartographer_ros_msgs::WriteState::Response save_response;

  save_request.filename = currentTrajectoryId;
  save_request.include_unfinished_submaps = true;

  HandleWriteStateDetail(save_request, save_response);

  if(save_response.status.code == cartographer_ros_msgs::StatusCode::OK)
  {
        LOG(INFO) << save_response.status.message;
    return true;
  }
  else
  {
        LOG(INFO) << save_response.status.message;
    return false;
  }

}





// okagv
void Node::RecordMapBuilderOption(
    cartographer::cloud::proto::MapBuilderServerOptions& slam_option,
    cartographer::cloud::proto::MapBuilderServerOptions& navigation_option) {
  slam_option_ = slam_option;
  navigation_option_ = navigation_option;
}

}  // namespace cartographer_ros
