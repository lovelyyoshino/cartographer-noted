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

#include "cartographer/io/internal/mapping_state_serialization.h"

#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace io {
namespace {
using mapping::MapById;
using mapping::NodeId;
using mapping::PoseGraphInterface;
using mapping::SubmapId;
using mapping::TrajectoryNode;
using mapping::proto::SerializedData;

mapping::proto::AllTrajectoryBuilderOptions
CreateAllTrajectoryBuilderOptionsProto(
    const std::map<int, mapping::proto::TrajectoryBuilderOptionsWithSensorIds>&
        all_options_with_sensor_ids,
    const std::vector<int>& trajectory_ids_to_serialize) {
  mapping::proto::AllTrajectoryBuilderOptions all_options_proto;
  for (auto id : trajectory_ids_to_serialize) {
    *all_options_proto.add_options_with_sensor_ids() =
        all_options_with_sensor_ids.at(id);
  }
  return all_options_proto;
}

// Will return all trajectory ids, that have `state != DELETED`.
std::vector<int> GetValidTrajectoryIds(
    const std::map<int, PoseGraphInterface::TrajectoryState>&
        trajectory_states) {
  std::vector<int> valid_trajectories;
  for (const auto& t : trajectory_states) {
    if (t.second != PoseGraphInterface::TrajectoryState::DELETED) {
      valid_trajectories.push_back(t.first);
    }
  }
  return valid_trajectories;
}

mapping::proto::SerializationHeader CreateHeader() {
  mapping::proto::SerializationHeader header;
  header.set_format_version(kMappingStateSerializationFormatVersion);
  return header;
}

SerializedData SerializePoseGraph(const mapping::PoseGraph& pose_graph,
                                  bool include_unfinished_submaps) {
  SerializedData proto;
  *proto.mutable_pose_graph() = pose_graph.ToProto(include_unfinished_submaps);
  return proto;
}

//okagv
SerializedData SerializePoseGraphWithId(int trajectory_id,
                                  const mapping::PoseGraph& pose_graph,
                                  bool include_unfinished_submaps) {
  SerializedData proto;
  *proto.mutable_pose_graph() = pose_graph.ToProtoWithId(trajectory_id, include_unfinished_submaps);
  return proto;
}

//okagv
SerializedData UpdateAndSerializePoseGraph(const mapping::PoseGraph& pose_graph,
                                  bool include_unfinished_submaps) {
  SerializedData proto;
  *proto.mutable_pose_graph() = pose_graph.ToProtoWithUpdate(include_unfinished_submaps);
  return proto;
}


SerializedData SerializeTrajectoryBuilderOptions(
    const std::map<int, mapping::proto::TrajectoryBuilderOptionsWithSensorIds>&
        trajectory_builder_options,
    const std::vector<int>& trajectory_ids_to_serialize) {
  SerializedData proto;
  *proto.mutable_all_trajectory_builder_options() =
      CreateAllTrajectoryBuilderOptionsProto(trajectory_builder_options,
                                             trajectory_ids_to_serialize);
  return proto;
}

//okagv ,need to check the trajectory_id first
SerializedData SerializeTrajectoryBuilderOptionsWithId(int trajectory_id,
    const std::map<int, mapping::proto::TrajectoryBuilderOptionsWithSensorIds>&
        trajectory_builder_options,
    const std::vector<int>& trajectory_ids_to_serialize) {

      std::map<int, mapping::proto::TrajectoryBuilderOptionsWithSensorIds>
        trajectory_builder_option_temp;
      std::vector<int> trajectory_ids_to_serialize_temp;

     trajectory_ids_to_serialize_temp.push_back(trajectory_id);
     trajectory_builder_option_temp.emplace(trajectory_id,trajectory_builder_options.at(trajectory_id));

  SerializedData proto;
  *proto.mutable_all_trajectory_builder_options() =
      CreateAllTrajectoryBuilderOptionsProto(trajectory_builder_option_temp,
                                             trajectory_ids_to_serialize_temp);
  return proto;
}

//okagv
SerializedData UpdateAndSerializeTrajectoryBuilderOptions(
    const std::map<int, mapping::proto::TrajectoryBuilderOptionsWithSensorIds>&
        trajectory_builder_options,
    const std::vector<int>& trajectory_ids_to_serialize) {

      std::map<int, mapping::proto::TrajectoryBuilderOptionsWithSensorIds>
        trajectory_builder_option_temp;
      std::vector<int> trajectory_ids_to_serialize_temp;

     trajectory_ids_to_serialize_temp.push_back(0);
     trajectory_builder_option_temp.emplace(0,trajectory_builder_options.at(0));

  SerializedData proto;
  *proto.mutable_all_trajectory_builder_options() =
      CreateAllTrajectoryBuilderOptionsProto(trajectory_builder_option_temp,
                                             trajectory_ids_to_serialize_temp);
  return proto;
}

void SerializeSubmaps(
    const MapById<SubmapId, PoseGraphInterface::SubmapData>& submap_data,
    bool include_unfinished_submaps, ProtoStreamWriterInterface* const writer) {
  // Next serialize all submaps.
  for (const auto& submap_id_data : submap_data) {
    if (!include_unfinished_submaps &&
        !submap_id_data.data.submap->insertion_finished()) {
      continue;
    }
    SerializedData proto;
    auto* const submap_proto = proto.mutable_submap();
    *submap_proto = submap_id_data.data.submap->ToProto(
        /*include_probability_grid_data=*/true);
    submap_proto->mutable_submap_id()->set_trajectory_id(
        submap_id_data.id.trajectory_id);
    submap_proto->mutable_submap_id()->set_submap_index(
        submap_id_data.id.submap_index);
    writer->WriteProto(proto);
  }
}

//okagv
void SerializeSubmapsWithId(int trajectory_id,
    const MapById<SubmapId, PoseGraphInterface::SubmapData>& submap_data,
    bool include_unfinished_submaps, ProtoStreamWriterInterface* const writer) {
  // Next serialize all submaps.
  for (const auto& submap_id_data : submap_data) {

    if(submap_id_data.id.trajectory_id != trajectory_id) continue;

    if (!include_unfinished_submaps &&
        !submap_id_data.data.submap->insertion_finished()) {
      continue;
    }
    SerializedData proto;
    auto* const submap_proto = proto.mutable_submap();
    *submap_proto = submap_id_data.data.submap->ToProto(
        /*include_probability_grid_data=*/true);
    submap_proto->mutable_submap_id()->set_trajectory_id(
        submap_id_data.id.trajectory_id);
    submap_proto->mutable_submap_id()->set_submap_index(
        submap_id_data.id.submap_index);
    writer->WriteProto(proto);
  }
}

//okagv
void UpdateAndSerializeSubmaps(
    const MapById<SubmapId, PoseGraphInterface::SubmapData>& submap_data,
    bool include_unfinished_submaps, ProtoStreamWriterInterface* const writer) {
  // Next serialize all submaps.
  for (const auto& submap_id_data : submap_data) {
    if (!include_unfinished_submaps &&
        !submap_id_data.data.submap->insertion_finished()) {
      continue;
    }
    SerializedData proto;
    auto* const submap_proto = proto.mutable_submap();
    *submap_proto = submap_id_data.data.submap->ToProto(
        /*include_probability_grid_data=*/true);
    submap_proto->mutable_submap_id()->set_trajectory_id(
        submap_id_data.id.trajectory_id);
    submap_proto->mutable_submap_id()->set_submap_index(
        submap_id_data.id.submap_index);
    writer->WriteProto(proto);
  }
}

void SerializeTrajectoryNodes(
    const MapById<NodeId, TrajectoryNode>& trajectory_nodes,
    ProtoStreamWriterInterface* const writer) {
  for (const auto& node_id_data : trajectory_nodes) {
    SerializedData proto;
    auto* const node_proto = proto.mutable_node();
    node_proto->mutable_node_id()->set_trajectory_id(
        node_id_data.id.trajectory_id);
    node_proto->mutable_node_id()->set_node_index(node_id_data.id.node_index);
    *node_proto->mutable_node_data() =
        ToProto(*node_id_data.data.constant_data);
    writer->WriteProto(proto);
  }
}

//okagv
void SerializeTrajectoryNodesWithId(int trajectory_id,
    const MapById<NodeId, TrajectoryNode>& trajectory_nodes,
    ProtoStreamWriterInterface* const writer) {
  for (const auto& node_id_data : trajectory_nodes) {
    
    if(node_id_data.id.trajectory_id != trajectory_id) continue;

    SerializedData proto;
    auto* const node_proto = proto.mutable_node();
    node_proto->mutable_node_id()->set_trajectory_id(
        node_id_data.id.trajectory_id);
    node_proto->mutable_node_id()->set_node_index(node_id_data.id.node_index);
    *node_proto->mutable_node_data() =
        ToProto(*node_id_data.data.constant_data);
    writer->WriteProto(proto);
  }
}

//okagv
void UpdateAndSerializeTrajectoryNodes(
    const MapById<NodeId, TrajectoryNode>& trajectory_nodes,
    ProtoStreamWriterInterface* const writer) {
  for (const auto& node_id_data : trajectory_nodes) {
    SerializedData proto;
    auto* const node_proto = proto.mutable_node();
    node_proto->mutable_node_id()->set_trajectory_id(
        node_id_data.id.trajectory_id);
    node_proto->mutable_node_id()->set_node_index(node_id_data.id.node_index);
    *node_proto->mutable_node_data() =
        ToProto(*node_id_data.data.constant_data);
    writer->WriteProto(proto);
  }
}


void SerializeTrajectoryData(
    const std::map<int, PoseGraphInterface::TrajectoryData>&
        all_trajectory_data,
    ProtoStreamWriterInterface* const writer) {
  for (const auto& trajectory_data : all_trajectory_data) {
    SerializedData proto;
    auto* const trajectory_data_proto = proto.mutable_trajectory_data();
    trajectory_data_proto->set_trajectory_id(trajectory_data.first);
    trajectory_data_proto->set_gravity_constant(
        trajectory_data.second.gravity_constant);
    *trajectory_data_proto->mutable_imu_calibration() = transform::ToProto(
        Eigen::Quaterniond(trajectory_data.second.imu_calibration[0],
                           trajectory_data.second.imu_calibration[1],
                           trajectory_data.second.imu_calibration[2],
                           trajectory_data.second.imu_calibration[3]));
    if (trajectory_data.second.fixed_frame_origin_in_map.has_value()) {
      *trajectory_data_proto->mutable_fixed_frame_origin_in_map() =
          transform::ToProto(
              trajectory_data.second.fixed_frame_origin_in_map.value());
    }
    writer->WriteProto(proto);
  }
}

//okagv
void SerializeTrajectoryDataWithId(int trajectory_id,
    const std::map<int, PoseGraphInterface::TrajectoryData>&
        all_trajectory_data,
    ProtoStreamWriterInterface* const writer) {
  for (const auto& trajectory_data : all_trajectory_data) {
    //okagv
    if(trajectory_data.first != trajectory_id) continue;

    SerializedData proto;
    auto* const trajectory_data_proto = proto.mutable_trajectory_data();
    trajectory_data_proto->set_trajectory_id(trajectory_data.first);
    trajectory_data_proto->set_gravity_constant(
        trajectory_data.second.gravity_constant);
    *trajectory_data_proto->mutable_imu_calibration() = transform::ToProto(
        Eigen::Quaterniond(trajectory_data.second.imu_calibration[0],
                           trajectory_data.second.imu_calibration[1],
                           trajectory_data.second.imu_calibration[2],
                           trajectory_data.second.imu_calibration[3]));
    if (trajectory_data.second.fixed_frame_origin_in_map.has_value()) {
      *trajectory_data_proto->mutable_fixed_frame_origin_in_map() =
          transform::ToProto(
              trajectory_data.second.fixed_frame_origin_in_map.value());
    }
    writer->WriteProto(proto);
  }
}

//okagv
void UpdateAndSerializeTrajectoryData(
    const std::map<int, PoseGraphInterface::TrajectoryData>&
        all_trajectory_data,
    ProtoStreamWriterInterface* const writer) {
  for (const auto& trajectory_data : all_trajectory_data) {
    SerializedData proto;
    auto* const trajectory_data_proto = proto.mutable_trajectory_data();
    trajectory_data_proto->set_trajectory_id(trajectory_data.first);
    trajectory_data_proto->set_gravity_constant(
        trajectory_data.second.gravity_constant);
    *trajectory_data_proto->mutable_imu_calibration() = transform::ToProto(
        Eigen::Quaterniond(trajectory_data.second.imu_calibration[0],
                           trajectory_data.second.imu_calibration[1],
                           trajectory_data.second.imu_calibration[2],
                           trajectory_data.second.imu_calibration[3]));
    if (trajectory_data.second.fixed_frame_origin_in_map.has_value()) {
      *trajectory_data_proto->mutable_fixed_frame_origin_in_map() =
          transform::ToProto(
              trajectory_data.second.fixed_frame_origin_in_map.value());
    }
    writer->WriteProto(proto);
  }
}

void SerializeImuData(const sensor::MapByTime<sensor::ImuData>& all_imu_data,
                      ProtoStreamWriterInterface* const writer) {
  for (const int trajectory_id : all_imu_data.trajectory_ids()) {
    for (const auto& imu_data : all_imu_data.trajectory(trajectory_id)) {
      SerializedData proto;
      auto* const imu_data_proto = proto.mutable_imu_data();
      imu_data_proto->set_trajectory_id(trajectory_id);
      *imu_data_proto->mutable_imu_data() = sensor::ToProto(imu_data);
      writer->WriteProto(proto);
    }
  }
}

//okagv
void SerializeImuDataWithId(int trajectory_id,
  const sensor::MapByTime<sensor::ImuData>& all_imu_data,
                      ProtoStreamWriterInterface* const writer) {
  for (const int entry_trajectory_id : all_imu_data.trajectory_ids()) {
     
     if(entry_trajectory_id != trajectory_id) continue;

    for (const auto& imu_data : all_imu_data.trajectory(entry_trajectory_id)) {
      SerializedData proto;
      auto* const imu_data_proto = proto.mutable_imu_data();
      imu_data_proto->set_trajectory_id(trajectory_id);
      *imu_data_proto->mutable_imu_data() = sensor::ToProto(imu_data);
      writer->WriteProto(proto);
    }
  }
}

// okagv
void UpdateAndSerializeImuData(
    const sensor::MapByTime<sensor::ImuData>& all_imu_data,
    ProtoStreamWriterInterface* const writer) {
  for (const int trajectory_id : all_imu_data.trajectory_ids()) {
    for (const auto& imu_data : all_imu_data.trajectory(trajectory_id)) {
      SerializedData proto;
      auto* const imu_data_proto = proto.mutable_imu_data();

      if (trajectory_id == 1001) {
        imu_data_proto->set_trajectory_id(0);
      } else if (trajectory_id == 0) {
        imu_data_proto->set_trajectory_id(trajectory_id);
      }
      *imu_data_proto->mutable_imu_data() = sensor::ToProto(imu_data);
      writer->WriteProto(proto);
    }
  }
}

void SerializeOdometryData(
    const sensor::MapByTime<sensor::OdometryData>& all_odometry_data,
    ProtoStreamWriterInterface* const writer) {
  for (const int trajectory_id : all_odometry_data.trajectory_ids()) {
    for (const auto& odometry_data :
         all_odometry_data.trajectory(trajectory_id)) {
      SerializedData proto;
      auto* const odometry_data_proto = proto.mutable_odometry_data();
      odometry_data_proto->set_trajectory_id(trajectory_id);
      *odometry_data_proto->mutable_odometry_data() =
          sensor::ToProto(odometry_data);
      writer->WriteProto(proto);
    }
  }
}

//okagv
void SerializeOdometryDataWithId(int trajectory_id,
    const sensor::MapByTime<sensor::OdometryData>& all_odometry_data,
    ProtoStreamWriterInterface* const writer) {
  for (const int entry_trajectory_id : all_odometry_data.trajectory_ids()) {

     if(entry_trajectory_id != trajectory_id) continue;

    for (const auto& odometry_data :
         all_odometry_data.trajectory(entry_trajectory_id)) {
      SerializedData proto;
      auto* const odometry_data_proto = proto.mutable_odometry_data();
      odometry_data_proto->set_trajectory_id(entry_trajectory_id);
      *odometry_data_proto->mutable_odometry_data() =
          sensor::ToProto(odometry_data);
      writer->WriteProto(proto);
    }
  }
}

//okagv
void UpdateAndSerializeOdometryData(
    const sensor::MapByTime<sensor::OdometryData>& all_odometry_data,
    ProtoStreamWriterInterface* const writer) {
  for (const int trajectory_id : all_odometry_data.trajectory_ids()) {
    for (const auto& odometry_data :
         all_odometry_data.trajectory(trajectory_id)) {
      SerializedData proto;
      auto* const odometry_data_proto = proto.mutable_odometry_data();
      odometry_data_proto->set_trajectory_id(trajectory_id);
      *odometry_data_proto->mutable_odometry_data() =
          sensor::ToProto(odometry_data);
      writer->WriteProto(proto);
    }
  }
}

void SerializeFixedFramePoseData(
    const sensor::MapByTime<sensor::FixedFramePoseData>&
        all_fixed_frame_pose_data,
    ProtoStreamWriterInterface* const writer) {
  for (const int trajectory_id : all_fixed_frame_pose_data.trajectory_ids()) {
    for (const auto& fixed_frame_pose_data :
         all_fixed_frame_pose_data.trajectory(trajectory_id)) {
      SerializedData proto;
      auto* const fixed_frame_pose_data_proto =
          proto.mutable_fixed_frame_pose_data();
      fixed_frame_pose_data_proto->set_trajectory_id(trajectory_id);
      *fixed_frame_pose_data_proto->mutable_fixed_frame_pose_data() =
          sensor::ToProto(fixed_frame_pose_data);
      writer->WriteProto(proto);
    }
  }
}

void SerializeFixedFramePoseDataWithId(int trajectory_id,
    const sensor::MapByTime<sensor::FixedFramePoseData>&
        all_fixed_frame_pose_data,
    ProtoStreamWriterInterface* const writer) {
  for (const int entry_trajectory_id : all_fixed_frame_pose_data.trajectory_ids()) {

     if(entry_trajectory_id == trajectory_id) continue;

    for (const auto& fixed_frame_pose_data :
         all_fixed_frame_pose_data.trajectory(entry_trajectory_id)) {
      SerializedData proto;
      auto* const fixed_frame_pose_data_proto =
          proto.mutable_fixed_frame_pose_data();
      fixed_frame_pose_data_proto->set_trajectory_id(entry_trajectory_id);
      *fixed_frame_pose_data_proto->mutable_fixed_frame_pose_data() =
          sensor::ToProto(fixed_frame_pose_data);
      writer->WriteProto(proto);
    }
  }
}

//okagv
void UpdateAndSerializeFixedFramePoseData(
    const sensor::MapByTime<sensor::FixedFramePoseData>&
        all_fixed_frame_pose_data,
    ProtoStreamWriterInterface* const writer) {
  for (const int trajectory_id : all_fixed_frame_pose_data.trajectory_ids()) {
    for (const auto& fixed_frame_pose_data :
         all_fixed_frame_pose_data.trajectory(trajectory_id)) {
      SerializedData proto;
      auto* const fixed_frame_pose_data_proto =
          proto.mutable_fixed_frame_pose_data();
      fixed_frame_pose_data_proto->set_trajectory_id(trajectory_id);
      *fixed_frame_pose_data_proto->mutable_fixed_frame_pose_data() =
          sensor::ToProto(fixed_frame_pose_data);
      writer->WriteProto(proto);
    }
  }
}

void SerializeLandmarkNodes(
    const std::map<std::string, PoseGraphInterface::LandmarkNode>&
        all_landmark_nodes,
    ProtoStreamWriterInterface* const writer) {
  for (const auto& node : all_landmark_nodes) {
    for (const auto& observation : node.second.landmark_observations) {
      SerializedData proto;
      auto* landmark_data_proto = proto.mutable_landmark_data();
      landmark_data_proto->set_trajectory_id(observation.trajectory_id);
      landmark_data_proto->mutable_landmark_data()->set_timestamp(
          common::ToUniversal(observation.time));

      auto* observation_proto = landmark_data_proto->mutable_landmark_data()
                                    ->add_landmark_observations();
      observation_proto->set_id(node.first);
      *observation_proto->mutable_landmark_to_tracking_transform() =
          transform::ToProto(observation.landmark_to_tracking_transform);
      observation_proto->set_translation_weight(observation.translation_weight);
      observation_proto->set_rotation_weight(observation.rotation_weight);

      //*landmark_data_proto->mutable_landmark_data()
      //     ->mutable_global_landmark_pose() =
      //    transform::ToProto(node.second.global_landmark_pose.value());

      writer->WriteProto(proto);
    }
  }
}

//okagv
void SerializeLandmarkNodesWithId(int trajectory_id,
    const std::map<std::string, PoseGraphInterface::LandmarkNode>&
        all_landmark_nodes,
    ProtoStreamWriterInterface* const writer) {
  for (const auto& node : all_landmark_nodes) {
    for (const auto& observation : node.second.landmark_observations) {

      if(observation.trajectory_id != trajectory_id) continue;

      SerializedData proto;
      auto* landmark_data_proto = proto.mutable_landmark_data();
      landmark_data_proto->set_trajectory_id(observation.trajectory_id);
      landmark_data_proto->mutable_landmark_data()->set_timestamp(
          common::ToUniversal(observation.time));
      auto* observation_proto = landmark_data_proto->mutable_landmark_data()
                                    ->add_landmark_observations();
      observation_proto->set_id(node.first);
      *observation_proto->mutable_landmark_to_tracking_transform() =
          transform::ToProto(observation.landmark_to_tracking_transform);
      observation_proto->set_translation_weight(observation.translation_weight);
      observation_proto->set_rotation_weight(observation.rotation_weight);

      //okagv
      //*landmark_data_proto->mutable_landmark_data()
      //     ->mutable_global_landmark_pose() =
      //    transform::ToProto(node.second.global_landmark_pose.value());

      writer->WriteProto(proto);
    }
  }
}

//okagv
void UpdateAndSerializeLandmarkNodes(
    const std::map<std::string, PoseGraphInterface::LandmarkNode>&
        all_landmark_nodes,
    ProtoStreamWriterInterface* const writer) {
  for (const auto& node : all_landmark_nodes) {
    for (const auto& observation : node.second.landmark_observations) {
      SerializedData proto;
      auto* landmark_data_proto = proto.mutable_landmark_data();
      landmark_data_proto->set_trajectory_id(observation.trajectory_id);
      landmark_data_proto->mutable_landmark_data()->set_timestamp(
          common::ToUniversal(observation.time));

      auto* observation_proto = landmark_data_proto->mutable_landmark_data()
                                    ->add_landmark_observations();
      observation_proto->set_id(node.first);
      *observation_proto->mutable_landmark_to_tracking_transform() =
          transform::ToProto(observation.landmark_to_tracking_transform);
      observation_proto->set_translation_weight(observation.translation_weight);
      observation_proto->set_rotation_weight(observation.rotation_weight);

      //*landmark_data_proto->mutable_landmark_data()
      //     ->mutable_global_landmark_pose() =
      //    transform::ToProto(node.second.global_landmark_pose.value());

      writer->WriteProto(proto);
    }
  }
}

}  // namespace

void WritePbStream(
    const mapping::PoseGraph& pose_graph,
    const std::map<int, mapping::proto::TrajectoryBuilderOptionsWithSensorIds>&
        trajectory_builder_options,
    ProtoStreamWriterInterface* const writer, bool include_unfinished_submaps) {
  writer->WriteProto(CreateHeader());
  writer->WriteProto(
      SerializePoseGraph(pose_graph, include_unfinished_submaps));
  writer->WriteProto(SerializeTrajectoryBuilderOptions(
      trajectory_builder_options,
      GetValidTrajectoryIds(pose_graph.GetTrajectoryStates())));

  SerializeSubmaps(pose_graph.GetAllSubmapData(), include_unfinished_submaps,
                   writer);
  SerializeTrajectoryNodes(pose_graph.GetTrajectoryNodes(), writer);
  SerializeTrajectoryData(pose_graph.GetTrajectoryData(), writer);
  SerializeImuData(pose_graph.GetImuData(), writer);
  SerializeOdometryData(pose_graph.GetOdometryData(), writer);
  SerializeFixedFramePoseData(pose_graph.GetFixedFramePoseData(), writer);
  SerializeLandmarkNodes(pose_graph.GetLandmarkNodes(), writer);
}

void WritePbStreamWithId(
    int trajectory_id, const mapping::PoseGraph& pose_graph,
    const std::map<int, mapping::proto::TrajectoryBuilderOptionsWithSensorIds>&
                trajectory_builder_options,
    ProtoStreamWriterInterface* const writer, bool include_unfinished_submaps) {
  
  writer->WriteProto(CreateHeader());
  writer->WriteProto(
      SerializePoseGraphWithId(trajectory_id, pose_graph, include_unfinished_submaps));
  writer->WriteProto(SerializeTrajectoryBuilderOptionsWithId(trajectory_id,
      trajectory_builder_options,
      GetValidTrajectoryIds(pose_graph.GetTrajectoryStates())));

  SerializeSubmapsWithId(trajectory_id, pose_graph.GetAllSubmapData(), include_unfinished_submaps,
                   writer);

  SerializeTrajectoryNodesWithId(trajectory_id, pose_graph.GetTrajectoryNodes(), writer);
  SerializeTrajectoryDataWithId(trajectory_id, pose_graph.GetTrajectoryData(), writer);
  SerializeImuDataWithId(trajectory_id, pose_graph.GetImuData(), writer);
  SerializeOdometryDataWithId(trajectory_id, pose_graph.GetOdometryData(), writer);
  SerializeFixedFramePoseDataWithId(trajectory_id, pose_graph.GetFixedFramePoseData(), writer);
  SerializeLandmarkNodesWithId(trajectory_id, pose_graph.GetLandmarkNodes(), writer);
  
}

void UpdateAndWritePbStream(
    const mapping::PoseGraph& pose_graph,
    const std::map<int, mapping::proto::TrajectoryBuilderOptionsWithSensorIds>&
        trajectory_builder_options,
    ProtoStreamWriterInterface* const writer, bool include_unfinished_submaps) {
      //LOG(INFO) << "okagv UpdateAndWritePbStream 0";
  writer->WriteProto(CreateHeader());
        //LOG(INFO) << "okagv UpdateAndWritePbStream 1";
  writer->WriteProto(
      UpdateAndSerializePoseGraph(pose_graph, include_unfinished_submaps));
            //LOG(INFO) << "okagv UpdateAndWritePbStream 2";
  writer->WriteProto(UpdateAndSerializeTrajectoryBuilderOptions(
      trajectory_builder_options,
      GetValidTrajectoryIds(pose_graph.GetTrajectoryStates())));
            //LOG(INFO) << "okagv UpdateAndWritePbStream 3";

  UpdateAndSerializeSubmaps(pose_graph.GetAllSubmapDataAfterUpdate(), include_unfinished_submaps,
                   writer);
                         //LOG(INFO) << "okagv UpdateAndWritePbStream 4";
  UpdateAndSerializeTrajectoryNodes(pose_graph.GetTrajectoryNodesAfterUpdate(), writer);
        //LOG(INFO) << "okagv UpdateAndWritePbStream 5";
  UpdateAndSerializeTrajectoryData(pose_graph.GetTrajectoryDataAfterUpdate(), writer);
        //LOG(INFO) << "okagv UpdateAndWritePbStream 6";
  UpdateAndSerializeImuData(pose_graph.GetImuData(), writer);
        //LOG(INFO) << "okagv UpdateAndWritePbStream 7";
  UpdateAndSerializeOdometryData(pose_graph.GetOdometryData(), writer);
        //LOG(INFO) << "okagv UpdateAndWritePbStream 8";
  UpdateAndSerializeFixedFramePoseData(pose_graph.GetFixedFramePoseData(), writer);
        //LOG(INFO) << "okagv UpdateAndWritePbStream 9";
  UpdateAndSerializeLandmarkNodes(pose_graph.GetLandmarkNodes(), writer);
        //LOG(INFO) << "okagv UpdateAndWritePbStream 10";
}

}  // namespace io
}  // namespace cartographer
