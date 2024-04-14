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

#include "cartographer/cloud/internal/handlers/okagv_start_trajectory.h"

#include "absl/memory/memory.h"
#include "async_grpc/rpc_handler.h"
#include "cartographer/cloud/internal/map_builder_context_interface.h"
#include "cartographer/cloud/proto/map_builder_service.pb.h"

#include "cartographer/mapping/pose_graph_interface.h"
#include "cartographer/mapping/map_builder_interface.h"

namespace cartographer {
namespace cloud {
namespace handlers {

using TrajectoryType = ::cartographer::mapping::MapBuilderInterface::TrajectoryType;

void OKagvStartTrajectoryHandler::OnRequest(
    const proto::OKagv_StartTrajectoryRequest& request) {
  /*      
  if (GetContext<MapBuilderContextInterface>()
          ->map_builder()
          .GetWorkingTrajectoryType() != TrajectoryType::IDLE) {
    Finish(
        ::grpc::Status(::grpc::UNAVAILABLE, "current trajectory unavailable"));
    return;
  } 
  */
  //reset param;    
  GetContext<MapBuilderContextInterface>()
      ->map_builder()
      .okagv_order_start_trajectory_.initial_pose =
      transform::ToRigid3(request.initial_pose());

  GetContext<MapBuilderContextInterface>()
      ->map_builder()
      .okagv_order_start_trajectory_.relative_to_trajectory_id =
      request.relative_to_trajectory_id();

  GetContext<MapBuilderContextInterface>()
      ->map_builder()
      .okagv_order_start_trajectory_.trajectory_id = request.trajectory_id();

  GetContext<MapBuilderContextInterface>()
      ->map_builder()
      .okagv_order_start_trajectory_.trajectory_type =
      request.trajectory_type();

  GetContext<MapBuilderContextInterface>()
      ->map_builder()
      .okagv_order_start_trajectory_.use_initial_pose =
      request.use_initial_pose();

  GetContext<MapBuilderContextInterface>()
      ->map_builder()
      .pose_graph()
      ->SetOKagv_Order(mapping::PoseGraphInterface::OKagvOrder::START);

  int count = 0;
  int wait_time = 100;
  auto response = absl::make_unique<proto::OKagv_StartTrajectoryResponse>();

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
        LOG(INFO) << "start success!";
        response->set_code(0);
        response->set_state(true);
        break;
      }
      else if(feedback.state == mapping::PoseGraphInterface::OKagvState::PROCESSING)
      {
        LOG(INFO) << "Processing ...";
      }
      else if(feedback.state == mapping::PoseGraphInterface::OKagvState::FAIL);
      {
        LOG(INFO) << "start fail";
        response->set_code(feedback.code);
        response->set_message(feedback.message);
        response->set_state(false);
        break;
      }
      
      absl::SleepFor(absl::Milliseconds(wait_time));
      count++;
      if(count*wait_time > 60*1000)
      {
        LOG(INFO) << "Failed to save map because of time out";
        response->set_code(1);
        response->set_message("Failed to save map because of time out");
        response->set_state(false);
        break;
      }
  }

  Send(std::move(response));
}

}  // namespace handlers
}  // namespace cloud
}  // namespace cartographer
