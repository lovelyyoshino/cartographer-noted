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

#include "cartographer/cloud/internal/handlers/okagv_get_trajectory_state.h"

#include "absl/memory/memory.h"
#include "async_grpc/rpc_handler.h"
#include "cartographer/cloud/internal/map_builder_context_interface.h"
#include "cartographer/cloud/internal/mapping/serialization.h"
#include "cartographer/cloud/proto/map_builder_service.pb.h"
#include "google/protobuf/empty.pb.h"

namespace cartographer {
namespace cloud {
namespace handlers {

using TrajectoryType = cartographer::mapping::MapBuilderInterface::TrajectoryType; 

void OKagvGetTrajectoryStateHandler::OnRequest(
    const google::protobuf::Empty& request) {
  uint8_t type = GetContext<MapBuilderContextInterface>()
                                ->map_builder()
                                .pose_graph()
                                ->GetWorkingTrajectoryType();

  auto response = absl::make_unique<proto::OKagv_GetTrajectoryStateResponse>();

  //response->set_order_type(int(okagv_order));

  switch (TrajectoryType(type))
  {
  case TrajectoryType::SLAM:
    response->set_message("slam");
    break;
  case TrajectoryType::LOAD:
    response->set_message("load map ...");
    break;
  case TrajectoryType::NAVIGATION:
    response->set_message("navigation");
    break;
  case TrajectoryType::RELOCALIZAION:
    response->set_message("relocalization");
    break;  
  case TrajectoryType::IDLE:
    response->set_message("idle");
    break;
  case TrajectoryType::ABORTION:
    response->set_message("abortion quit");
    break;  
  default:
    response->set_message("unknown state");
    break;
  }
  
  response->set_code(type);
  Send(std::move(response));
}

}  // namespace handlers
}  // namespace cloud
}  // namespace cartographer
