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

#include "cartographer/cloud/internal/handlers/okagv_save_trajectory.h"

#include "absl/memory/memory.h"
#include "async_grpc/rpc_handler.h"
#include "cartographer/cloud/internal/map_builder_context_interface.h"
#include "cartographer/cloud/internal/map_builder_server.h"
#include "cartographer/cloud/proto/map_builder_service.pb.h"
#include "cartographer/io/proto_stream.h"

namespace cartographer {
namespace cloud {
namespace handlers {

void OKagvSaveTrajectoryHandler::OnRequest(
    const proto::OKagv_SaveTrajectoryRequest& request) {

        LOG(INFO) << "okagv save name " << request.filename();
  if (request.filename().empty()) {
    Finish(::grpc::Status(::grpc::INVALID_ARGUMENT, "Filename empty."));
    return;
  }

  int trajectory_id =  GetContext<MapBuilderContextInterface>()->map_builder().GetTrajectoryIdByName(request.filename());
  std::string root_file_directory = GetContext<MapBuilderContextInterface>()->GetRootFileDirectory();
  std::string intact_name = root_file_directory + "/map/" + request.filename() + ".pbstream";

  LOG(INFO) << "trajectory_id is " << trajectory_id; 
  LOG(INFO) << "intact_name is " << intact_name;

  GetContext<MapBuilderContextInterface>()
      ->map_builder()
      .okagv_order_save_trajectory_.filename = request.filename();

  GetContext<MapBuilderContextInterface>()
      ->map_builder()
      .okagv_order_save_trajectory_.include_unfinished_submaps =
      request.include_unfinished_submaps();

  GetContext<MapBuilderContextInterface>()
      ->map_builder()
      .pose_graph()
      ->SetOKagv_Order(mapping::PoseGraphInterface::OKagvOrder::SAVE);

  //then save it
  
  //bool success =
  //    GetContext<MapBuilderContextInterface>()
  //        ->map_builder()
  //        .SerializeStateToFileWithId(trajectory_id, /*include_unfinished_submaps*/true, intact_name);

  int count = 0;
  int wait_time = 100;
  auto response = absl::make_unique<proto::WriteStateToFileResponse>();

  mapping::PoseGraphInterface::OKagvFeedback feedback;
  feedback.state = mapping::PoseGraphInterface::OKagvState::PROCESSING;

  GetContext<MapBuilderContextInterface>()
      ->map_builder()
      .pose_graph()
      ->SetOKagv_Feedback(feedback);
  
  for(;;)
  {
      mapping::PoseGraphInterface::OKagvFeedback feedback =  GetContext<MapBuilderContextInterface>()
      ->map_builder()
      .pose_graph()->GetOKagv_Feedback();

      if(feedback.state == mapping::PoseGraphInterface::OKagvState::SUCCESS)
      {
        LOG(INFO) << "save success!";
        response->set_code(0);
        response->set_message("save map succuss!");
        response->set_success(true);
        break;
      }
      else if(feedback.state == mapping::PoseGraphInterface::OKagvState::FAIL)
      {
        LOG(INFO) << "Save fail";
        response->set_code(feedback.code);
        response->set_message(feedback.message);
        response->set_success(false);
        break;
      }
      else if(feedback.state == mapping::PoseGraphInterface::OKagvState::PROCESSING)
      {
        //LOG(INFO) << "Save map ...";
        continue;
      }

      absl::SleepFor(absl::Milliseconds(wait_time));
      count++;
      if(count*wait_time > 60*1000)
      {
        LOG(INFO) << "Failed to save map because of time out";
        response->set_code(1);
        response->set_message("Failed to save map because of time out");
        response->set_success(false);
        break;
      }
      
  }

  Send(std::move(response));
}

}  // namespace handlers
}  // namespace cloud
}  // namespace cartographer
