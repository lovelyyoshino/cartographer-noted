#include "cartographer/cloud/map_builder_server_interface.h"

#include "absl/memory/memory.h"
#include "cartographer/cloud/internal/map_builder_server.h"

namespace cartographer {
namespace cloud {

void RegisterMapBuilderServerMetrics(metrics::FamilyFactory* factory) {
  MapBuilderServer::RegisterMetrics(factory);
}

std::unique_ptr<MapBuilderServerInterface> CreateMapBuilderServer(  //okagv default is unique_ptr
    const proto::MapBuilderServerOptions& map_builder_server_options,
    std::shared_ptr<mapping::MapBuilderInterface> map_builder) {
  return absl::make_unique<MapBuilderServer>(map_builder_server_options,
                                             std::move(map_builder));
}

}  // namespace cloud
}  // namespace cartographer
