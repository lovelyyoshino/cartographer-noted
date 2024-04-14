// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: cartographer/mapping/proto/submap.proto

#include "cartographer/mapping/proto/submap.pb.h"

#include <algorithm>

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/wire_format_lite.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
extern PROTOBUF_INTERNAL_EXPORT_cartographer_2fmapping_2fproto_2fgrid_5f2d_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<4> scc_info_Grid2D_cartographer_2fmapping_2fproto_2fgrid_5f2d_2eproto;
extern PROTOBUF_INTERNAL_EXPORT_cartographer_2fmapping_2fproto_2fhybrid_5fgrid_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_HybridGrid_cartographer_2fmapping_2fproto_2fhybrid_5fgrid_2eproto;
extern PROTOBUF_INTERNAL_EXPORT_cartographer_2ftransform_2fproto_2ftransform_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<2> scc_info_Rigid3d_cartographer_2ftransform_2fproto_2ftransform_2eproto;
namespace cartographer {
namespace mapping {
namespace proto {
class Submap2DDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<Submap2D> _instance;
} _Submap2D_default_instance_;
class Submap3DDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<Submap3D> _instance;
} _Submap3D_default_instance_;
}  // namespace proto
}  // namespace mapping
}  // namespace cartographer
static void InitDefaultsscc_info_Submap2D_cartographer_2fmapping_2fproto_2fsubmap_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::cartographer::mapping::proto::_Submap2D_default_instance_;
    new (ptr) ::cartographer::mapping::proto::Submap2D();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::cartographer::mapping::proto::Submap2D::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<2> scc_info_Submap2D_cartographer_2fmapping_2fproto_2fsubmap_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 2, 0, InitDefaultsscc_info_Submap2D_cartographer_2fmapping_2fproto_2fsubmap_2eproto}, {
      &scc_info_Rigid3d_cartographer_2ftransform_2fproto_2ftransform_2eproto.base,
      &scc_info_Grid2D_cartographer_2fmapping_2fproto_2fgrid_5f2d_2eproto.base,}};

static void InitDefaultsscc_info_Submap3D_cartographer_2fmapping_2fproto_2fsubmap_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::cartographer::mapping::proto::_Submap3D_default_instance_;
    new (ptr) ::cartographer::mapping::proto::Submap3D();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::cartographer::mapping::proto::Submap3D::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<2> scc_info_Submap3D_cartographer_2fmapping_2fproto_2fsubmap_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 2, 0, InitDefaultsscc_info_Submap3D_cartographer_2fmapping_2fproto_2fsubmap_2eproto}, {
      &scc_info_Rigid3d_cartographer_2ftransform_2fproto_2ftransform_2eproto.base,
      &scc_info_HybridGrid_cartographer_2fmapping_2fproto_2fhybrid_5fgrid_2eproto.base,}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_cartographer_2fmapping_2fproto_2fsubmap_2eproto[2];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_cartographer_2fmapping_2fproto_2fsubmap_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_cartographer_2fmapping_2fproto_2fsubmap_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_cartographer_2fmapping_2fproto_2fsubmap_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::cartographer::mapping::proto::Submap2D, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::cartographer::mapping::proto::Submap2D, local_pose_),
  PROTOBUF_FIELD_OFFSET(::cartographer::mapping::proto::Submap2D, num_range_data_),
  PROTOBUF_FIELD_OFFSET(::cartographer::mapping::proto::Submap2D, finished_),
  PROTOBUF_FIELD_OFFSET(::cartographer::mapping::proto::Submap2D, grid_),
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::cartographer::mapping::proto::Submap3D, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::cartographer::mapping::proto::Submap3D, local_pose_),
  PROTOBUF_FIELD_OFFSET(::cartographer::mapping::proto::Submap3D, num_range_data_),
  PROTOBUF_FIELD_OFFSET(::cartographer::mapping::proto::Submap3D, finished_),
  PROTOBUF_FIELD_OFFSET(::cartographer::mapping::proto::Submap3D, high_resolution_hybrid_grid_),
  PROTOBUF_FIELD_OFFSET(::cartographer::mapping::proto::Submap3D, low_resolution_hybrid_grid_),
  PROTOBUF_FIELD_OFFSET(::cartographer::mapping::proto::Submap3D, rotational_scan_matcher_histogram_),
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, sizeof(::cartographer::mapping::proto::Submap2D)},
  { 9, -1, sizeof(::cartographer::mapping::proto::Submap3D)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::cartographer::mapping::proto::_Submap2D_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::cartographer::mapping::proto::_Submap3D_default_instance_),
};

const char descriptor_table_protodef_cartographer_2fmapping_2fproto_2fsubmap_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\'cartographer/mapping/proto/submap.prot"
  "o\022\032cartographer.mapping.proto\032(cartograp"
  "her/mapping/proto/grid_2d.proto\032,cartogr"
  "apher/mapping/proto/hybrid_grid.proto\032,c"
  "artographer/transform/proto/transform.pr"
  "oto\"\241\001\n\010Submap2D\0229\n\nlocal_pose\030\001 \001(\0132%.c"
  "artographer.transform.proto.Rigid3d\022\026\n\016n"
  "um_range_data\030\002 \001(\005\022\020\n\010finished\030\003 \001(\010\0220\n"
  "\004grid\030\004 \001(\0132\".cartographer.mapping.proto"
  ".Grid2D\"\263\002\n\010Submap3D\0229\n\nlocal_pose\030\001 \001(\013"
  "2%.cartographer.transform.proto.Rigid3d\022"
  "\026\n\016num_range_data\030\002 \001(\005\022\020\n\010finished\030\003 \001("
  "\010\022K\n\033high_resolution_hybrid_grid\030\004 \001(\0132&"
  ".cartographer.mapping.proto.HybridGrid\022J"
  "\n\032low_resolution_hybrid_grid\030\005 \001(\0132&.car"
  "tographer.mapping.proto.HybridGrid\022)\n!ro"
  "tational_scan_matcher_histogram\030\006 \003(\002b\006p"
  "roto3"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_cartographer_2fmapping_2fproto_2fsubmap_2eproto_deps[3] = {
  &::descriptor_table_cartographer_2fmapping_2fproto_2fgrid_5f2d_2eproto,
  &::descriptor_table_cartographer_2fmapping_2fproto_2fhybrid_5fgrid_2eproto,
  &::descriptor_table_cartographer_2ftransform_2fproto_2ftransform_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_cartographer_2fmapping_2fproto_2fsubmap_2eproto_sccs[2] = {
  &scc_info_Submap2D_cartographer_2fmapping_2fproto_2fsubmap_2eproto.base,
  &scc_info_Submap3D_cartographer_2fmapping_2fproto_2fsubmap_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_cartographer_2fmapping_2fproto_2fsubmap_2eproto_once;
static bool descriptor_table_cartographer_2fmapping_2fproto_2fsubmap_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_cartographer_2fmapping_2fproto_2fsubmap_2eproto = {
  &descriptor_table_cartographer_2fmapping_2fproto_2fsubmap_2eproto_initialized, descriptor_table_protodef_cartographer_2fmapping_2fproto_2fsubmap_2eproto, "cartographer/mapping/proto/submap.proto", 685,
  &descriptor_table_cartographer_2fmapping_2fproto_2fsubmap_2eproto_once, descriptor_table_cartographer_2fmapping_2fproto_2fsubmap_2eproto_sccs, descriptor_table_cartographer_2fmapping_2fproto_2fsubmap_2eproto_deps, 2, 3,
  schemas, file_default_instances, TableStruct_cartographer_2fmapping_2fproto_2fsubmap_2eproto::offsets,
  file_level_metadata_cartographer_2fmapping_2fproto_2fsubmap_2eproto, 2, file_level_enum_descriptors_cartographer_2fmapping_2fproto_2fsubmap_2eproto, file_level_service_descriptors_cartographer_2fmapping_2fproto_2fsubmap_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_cartographer_2fmapping_2fproto_2fsubmap_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_cartographer_2fmapping_2fproto_2fsubmap_2eproto), true);
namespace cartographer {
namespace mapping {
namespace proto {

// ===================================================================

void Submap2D::InitAsDefaultInstance() {
  ::cartographer::mapping::proto::_Submap2D_default_instance_._instance.get_mutable()->local_pose_ = const_cast< ::cartographer::transform::proto::Rigid3d*>(
      ::cartographer::transform::proto::Rigid3d::internal_default_instance());
  ::cartographer::mapping::proto::_Submap2D_default_instance_._instance.get_mutable()->grid_ = const_cast< ::cartographer::mapping::proto::Grid2D*>(
      ::cartographer::mapping::proto::Grid2D::internal_default_instance());
}
class Submap2D::_Internal {
 public:
  static const ::cartographer::transform::proto::Rigid3d& local_pose(const Submap2D* msg);
  static const ::cartographer::mapping::proto::Grid2D& grid(const Submap2D* msg);
};

const ::cartographer::transform::proto::Rigid3d&
Submap2D::_Internal::local_pose(const Submap2D* msg) {
  return *msg->local_pose_;
}
const ::cartographer::mapping::proto::Grid2D&
Submap2D::_Internal::grid(const Submap2D* msg) {
  return *msg->grid_;
}
void Submap2D::clear_local_pose() {
  if (GetArenaNoVirtual() == nullptr && local_pose_ != nullptr) {
    delete local_pose_;
  }
  local_pose_ = nullptr;
}
void Submap2D::clear_grid() {
  if (GetArenaNoVirtual() == nullptr && grid_ != nullptr) {
    delete grid_;
  }
  grid_ = nullptr;
}
Submap2D::Submap2D()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:cartographer.mapping.proto.Submap2D)
}
Submap2D::Submap2D(const Submap2D& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  if (from._internal_has_local_pose()) {
    local_pose_ = new ::cartographer::transform::proto::Rigid3d(*from.local_pose_);
  } else {
    local_pose_ = nullptr;
  }
  if (from._internal_has_grid()) {
    grid_ = new ::cartographer::mapping::proto::Grid2D(*from.grid_);
  } else {
    grid_ = nullptr;
  }
  ::memcpy(&num_range_data_, &from.num_range_data_,
    static_cast<size_t>(reinterpret_cast<char*>(&finished_) -
    reinterpret_cast<char*>(&num_range_data_)) + sizeof(finished_));
  // @@protoc_insertion_point(copy_constructor:cartographer.mapping.proto.Submap2D)
}

void Submap2D::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_Submap2D_cartographer_2fmapping_2fproto_2fsubmap_2eproto.base);
  ::memset(&local_pose_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&finished_) -
      reinterpret_cast<char*>(&local_pose_)) + sizeof(finished_));
}

Submap2D::~Submap2D() {
  // @@protoc_insertion_point(destructor:cartographer.mapping.proto.Submap2D)
  SharedDtor();
}

void Submap2D::SharedDtor() {
  if (this != internal_default_instance()) delete local_pose_;
  if (this != internal_default_instance()) delete grid_;
}

void Submap2D::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const Submap2D& Submap2D::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_Submap2D_cartographer_2fmapping_2fproto_2fsubmap_2eproto.base);
  return *internal_default_instance();
}


void Submap2D::Clear() {
// @@protoc_insertion_point(message_clear_start:cartographer.mapping.proto.Submap2D)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  if (GetArenaNoVirtual() == nullptr && local_pose_ != nullptr) {
    delete local_pose_;
  }
  local_pose_ = nullptr;
  if (GetArenaNoVirtual() == nullptr && grid_ != nullptr) {
    delete grid_;
  }
  grid_ = nullptr;
  ::memset(&num_range_data_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&finished_) -
      reinterpret_cast<char*>(&num_range_data_)) + sizeof(finished_));
  _internal_metadata_.Clear();
}

const char* Submap2D::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // .cartographer.transform.proto.Rigid3d local_pose = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 10)) {
          ptr = ctx->ParseMessage(_internal_mutable_local_pose(), ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // int32 num_range_data = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 16)) {
          num_range_data_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // bool finished = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 24)) {
          finished_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // .cartographer.mapping.proto.Grid2D grid = 4;
      case 4:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 34)) {
          ptr = ctx->ParseMessage(_internal_mutable_grid(), ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      default: {
      handle_unusual:
        if ((tag & 7) == 4 || tag == 0) {
          ctx->SetLastTag(tag);
          goto success;
        }
        ptr = UnknownFieldParse(tag, &_internal_metadata_, ptr, ctx);
        CHK_(ptr != nullptr);
        continue;
      }
    }  // switch
  }  // while
success:
  return ptr;
failure:
  ptr = nullptr;
  goto success;
#undef CHK_
}

::PROTOBUF_NAMESPACE_ID::uint8* Submap2D::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:cartographer.mapping.proto.Submap2D)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // .cartographer.transform.proto.Rigid3d local_pose = 1;
  if (this->has_local_pose()) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        1, _Internal::local_pose(this), target, stream);
  }

  // int32 num_range_data = 2;
  if (this->num_range_data() != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteInt32ToArray(2, this->_internal_num_range_data(), target);
  }

  // bool finished = 3;
  if (this->finished() != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(3, this->_internal_finished(), target);
  }

  // .cartographer.mapping.proto.Grid2D grid = 4;
  if (this->has_grid()) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        4, _Internal::grid(this), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:cartographer.mapping.proto.Submap2D)
  return target;
}

size_t Submap2D::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:cartographer.mapping.proto.Submap2D)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // .cartographer.transform.proto.Rigid3d local_pose = 1;
  if (this->has_local_pose()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *local_pose_);
  }

  // .cartographer.mapping.proto.Grid2D grid = 4;
  if (this->has_grid()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *grid_);
  }

  // int32 num_range_data = 2;
  if (this->num_range_data() != 0) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::Int32Size(
        this->_internal_num_range_data());
  }

  // bool finished = 3;
  if (this->finished() != 0) {
    total_size += 1 + 1;
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    return ::PROTOBUF_NAMESPACE_ID::internal::ComputeUnknownFieldsSize(
        _internal_metadata_, total_size, &_cached_size_);
  }
  int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void Submap2D::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:cartographer.mapping.proto.Submap2D)
  GOOGLE_DCHECK_NE(&from, this);
  const Submap2D* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<Submap2D>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:cartographer.mapping.proto.Submap2D)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:cartographer.mapping.proto.Submap2D)
    MergeFrom(*source);
  }
}

void Submap2D::MergeFrom(const Submap2D& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:cartographer.mapping.proto.Submap2D)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  if (from.has_local_pose()) {
    _internal_mutable_local_pose()->::cartographer::transform::proto::Rigid3d::MergeFrom(from._internal_local_pose());
  }
  if (from.has_grid()) {
    _internal_mutable_grid()->::cartographer::mapping::proto::Grid2D::MergeFrom(from._internal_grid());
  }
  if (from.num_range_data() != 0) {
    _internal_set_num_range_data(from._internal_num_range_data());
  }
  if (from.finished() != 0) {
    _internal_set_finished(from._internal_finished());
  }
}

void Submap2D::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:cartographer.mapping.proto.Submap2D)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Submap2D::CopyFrom(const Submap2D& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:cartographer.mapping.proto.Submap2D)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Submap2D::IsInitialized() const {
  return true;
}

void Submap2D::InternalSwap(Submap2D* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(local_pose_, other->local_pose_);
  swap(grid_, other->grid_);
  swap(num_range_data_, other->num_range_data_);
  swap(finished_, other->finished_);
}

::PROTOBUF_NAMESPACE_ID::Metadata Submap2D::GetMetadata() const {
  return GetMetadataStatic();
}


// ===================================================================

void Submap3D::InitAsDefaultInstance() {
  ::cartographer::mapping::proto::_Submap3D_default_instance_._instance.get_mutable()->local_pose_ = const_cast< ::cartographer::transform::proto::Rigid3d*>(
      ::cartographer::transform::proto::Rigid3d::internal_default_instance());
  ::cartographer::mapping::proto::_Submap3D_default_instance_._instance.get_mutable()->high_resolution_hybrid_grid_ = const_cast< ::cartographer::mapping::proto::HybridGrid*>(
      ::cartographer::mapping::proto::HybridGrid::internal_default_instance());
  ::cartographer::mapping::proto::_Submap3D_default_instance_._instance.get_mutable()->low_resolution_hybrid_grid_ = const_cast< ::cartographer::mapping::proto::HybridGrid*>(
      ::cartographer::mapping::proto::HybridGrid::internal_default_instance());
}
class Submap3D::_Internal {
 public:
  static const ::cartographer::transform::proto::Rigid3d& local_pose(const Submap3D* msg);
  static const ::cartographer::mapping::proto::HybridGrid& high_resolution_hybrid_grid(const Submap3D* msg);
  static const ::cartographer::mapping::proto::HybridGrid& low_resolution_hybrid_grid(const Submap3D* msg);
};

const ::cartographer::transform::proto::Rigid3d&
Submap3D::_Internal::local_pose(const Submap3D* msg) {
  return *msg->local_pose_;
}
const ::cartographer::mapping::proto::HybridGrid&
Submap3D::_Internal::high_resolution_hybrid_grid(const Submap3D* msg) {
  return *msg->high_resolution_hybrid_grid_;
}
const ::cartographer::mapping::proto::HybridGrid&
Submap3D::_Internal::low_resolution_hybrid_grid(const Submap3D* msg) {
  return *msg->low_resolution_hybrid_grid_;
}
void Submap3D::clear_local_pose() {
  if (GetArenaNoVirtual() == nullptr && local_pose_ != nullptr) {
    delete local_pose_;
  }
  local_pose_ = nullptr;
}
void Submap3D::clear_high_resolution_hybrid_grid() {
  if (GetArenaNoVirtual() == nullptr && high_resolution_hybrid_grid_ != nullptr) {
    delete high_resolution_hybrid_grid_;
  }
  high_resolution_hybrid_grid_ = nullptr;
}
void Submap3D::clear_low_resolution_hybrid_grid() {
  if (GetArenaNoVirtual() == nullptr && low_resolution_hybrid_grid_ != nullptr) {
    delete low_resolution_hybrid_grid_;
  }
  low_resolution_hybrid_grid_ = nullptr;
}
Submap3D::Submap3D()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:cartographer.mapping.proto.Submap3D)
}
Submap3D::Submap3D(const Submap3D& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      rotational_scan_matcher_histogram_(from.rotational_scan_matcher_histogram_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  if (from._internal_has_local_pose()) {
    local_pose_ = new ::cartographer::transform::proto::Rigid3d(*from.local_pose_);
  } else {
    local_pose_ = nullptr;
  }
  if (from._internal_has_high_resolution_hybrid_grid()) {
    high_resolution_hybrid_grid_ = new ::cartographer::mapping::proto::HybridGrid(*from.high_resolution_hybrid_grid_);
  } else {
    high_resolution_hybrid_grid_ = nullptr;
  }
  if (from._internal_has_low_resolution_hybrid_grid()) {
    low_resolution_hybrid_grid_ = new ::cartographer::mapping::proto::HybridGrid(*from.low_resolution_hybrid_grid_);
  } else {
    low_resolution_hybrid_grid_ = nullptr;
  }
  ::memcpy(&num_range_data_, &from.num_range_data_,
    static_cast<size_t>(reinterpret_cast<char*>(&finished_) -
    reinterpret_cast<char*>(&num_range_data_)) + sizeof(finished_));
  // @@protoc_insertion_point(copy_constructor:cartographer.mapping.proto.Submap3D)
}

void Submap3D::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_Submap3D_cartographer_2fmapping_2fproto_2fsubmap_2eproto.base);
  ::memset(&local_pose_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&finished_) -
      reinterpret_cast<char*>(&local_pose_)) + sizeof(finished_));
}

Submap3D::~Submap3D() {
  // @@protoc_insertion_point(destructor:cartographer.mapping.proto.Submap3D)
  SharedDtor();
}

void Submap3D::SharedDtor() {
  if (this != internal_default_instance()) delete local_pose_;
  if (this != internal_default_instance()) delete high_resolution_hybrid_grid_;
  if (this != internal_default_instance()) delete low_resolution_hybrid_grid_;
}

void Submap3D::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const Submap3D& Submap3D::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_Submap3D_cartographer_2fmapping_2fproto_2fsubmap_2eproto.base);
  return *internal_default_instance();
}


void Submap3D::Clear() {
// @@protoc_insertion_point(message_clear_start:cartographer.mapping.proto.Submap3D)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  rotational_scan_matcher_histogram_.Clear();
  if (GetArenaNoVirtual() == nullptr && local_pose_ != nullptr) {
    delete local_pose_;
  }
  local_pose_ = nullptr;
  if (GetArenaNoVirtual() == nullptr && high_resolution_hybrid_grid_ != nullptr) {
    delete high_resolution_hybrid_grid_;
  }
  high_resolution_hybrid_grid_ = nullptr;
  if (GetArenaNoVirtual() == nullptr && low_resolution_hybrid_grid_ != nullptr) {
    delete low_resolution_hybrid_grid_;
  }
  low_resolution_hybrid_grid_ = nullptr;
  ::memset(&num_range_data_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&finished_) -
      reinterpret_cast<char*>(&num_range_data_)) + sizeof(finished_));
  _internal_metadata_.Clear();
}

const char* Submap3D::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // .cartographer.transform.proto.Rigid3d local_pose = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 10)) {
          ptr = ctx->ParseMessage(_internal_mutable_local_pose(), ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // int32 num_range_data = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 16)) {
          num_range_data_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // bool finished = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 24)) {
          finished_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // .cartographer.mapping.proto.HybridGrid high_resolution_hybrid_grid = 4;
      case 4:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 34)) {
          ptr = ctx->ParseMessage(_internal_mutable_high_resolution_hybrid_grid(), ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // .cartographer.mapping.proto.HybridGrid low_resolution_hybrid_grid = 5;
      case 5:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 42)) {
          ptr = ctx->ParseMessage(_internal_mutable_low_resolution_hybrid_grid(), ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // repeated float rotational_scan_matcher_histogram = 6;
      case 6:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 50)) {
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::PackedFloatParser(_internal_mutable_rotational_scan_matcher_histogram(), ptr, ctx);
          CHK_(ptr);
        } else if (static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 53) {
          _internal_add_rotational_scan_matcher_histogram(::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr));
          ptr += sizeof(float);
        } else goto handle_unusual;
        continue;
      default: {
      handle_unusual:
        if ((tag & 7) == 4 || tag == 0) {
          ctx->SetLastTag(tag);
          goto success;
        }
        ptr = UnknownFieldParse(tag, &_internal_metadata_, ptr, ctx);
        CHK_(ptr != nullptr);
        continue;
      }
    }  // switch
  }  // while
success:
  return ptr;
failure:
  ptr = nullptr;
  goto success;
#undef CHK_
}

::PROTOBUF_NAMESPACE_ID::uint8* Submap3D::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:cartographer.mapping.proto.Submap3D)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // .cartographer.transform.proto.Rigid3d local_pose = 1;
  if (this->has_local_pose()) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        1, _Internal::local_pose(this), target, stream);
  }

  // int32 num_range_data = 2;
  if (this->num_range_data() != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteInt32ToArray(2, this->_internal_num_range_data(), target);
  }

  // bool finished = 3;
  if (this->finished() != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(3, this->_internal_finished(), target);
  }

  // .cartographer.mapping.proto.HybridGrid high_resolution_hybrid_grid = 4;
  if (this->has_high_resolution_hybrid_grid()) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        4, _Internal::high_resolution_hybrid_grid(this), target, stream);
  }

  // .cartographer.mapping.proto.HybridGrid low_resolution_hybrid_grid = 5;
  if (this->has_low_resolution_hybrid_grid()) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        5, _Internal::low_resolution_hybrid_grid(this), target, stream);
  }

  // repeated float rotational_scan_matcher_histogram = 6;
  if (this->_internal_rotational_scan_matcher_histogram_size() > 0) {
    target = stream->WriteFixedPacked(6, _internal_rotational_scan_matcher_histogram(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:cartographer.mapping.proto.Submap3D)
  return target;
}

size_t Submap3D::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:cartographer.mapping.proto.Submap3D)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated float rotational_scan_matcher_histogram = 6;
  {
    unsigned int count = static_cast<unsigned int>(this->_internal_rotational_scan_matcher_histogram_size());
    size_t data_size = 4UL * count;
    if (data_size > 0) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::Int32Size(
            static_cast<::PROTOBUF_NAMESPACE_ID::int32>(data_size));
    }
    int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(data_size);
    _rotational_scan_matcher_histogram_cached_byte_size_.store(cached_size,
                                    std::memory_order_relaxed);
    total_size += data_size;
  }

  // .cartographer.transform.proto.Rigid3d local_pose = 1;
  if (this->has_local_pose()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *local_pose_);
  }

  // .cartographer.mapping.proto.HybridGrid high_resolution_hybrid_grid = 4;
  if (this->has_high_resolution_hybrid_grid()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *high_resolution_hybrid_grid_);
  }

  // .cartographer.mapping.proto.HybridGrid low_resolution_hybrid_grid = 5;
  if (this->has_low_resolution_hybrid_grid()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *low_resolution_hybrid_grid_);
  }

  // int32 num_range_data = 2;
  if (this->num_range_data() != 0) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::Int32Size(
        this->_internal_num_range_data());
  }

  // bool finished = 3;
  if (this->finished() != 0) {
    total_size += 1 + 1;
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    return ::PROTOBUF_NAMESPACE_ID::internal::ComputeUnknownFieldsSize(
        _internal_metadata_, total_size, &_cached_size_);
  }
  int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void Submap3D::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:cartographer.mapping.proto.Submap3D)
  GOOGLE_DCHECK_NE(&from, this);
  const Submap3D* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<Submap3D>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:cartographer.mapping.proto.Submap3D)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:cartographer.mapping.proto.Submap3D)
    MergeFrom(*source);
  }
}

void Submap3D::MergeFrom(const Submap3D& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:cartographer.mapping.proto.Submap3D)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  rotational_scan_matcher_histogram_.MergeFrom(from.rotational_scan_matcher_histogram_);
  if (from.has_local_pose()) {
    _internal_mutable_local_pose()->::cartographer::transform::proto::Rigid3d::MergeFrom(from._internal_local_pose());
  }
  if (from.has_high_resolution_hybrid_grid()) {
    _internal_mutable_high_resolution_hybrid_grid()->::cartographer::mapping::proto::HybridGrid::MergeFrom(from._internal_high_resolution_hybrid_grid());
  }
  if (from.has_low_resolution_hybrid_grid()) {
    _internal_mutable_low_resolution_hybrid_grid()->::cartographer::mapping::proto::HybridGrid::MergeFrom(from._internal_low_resolution_hybrid_grid());
  }
  if (from.num_range_data() != 0) {
    _internal_set_num_range_data(from._internal_num_range_data());
  }
  if (from.finished() != 0) {
    _internal_set_finished(from._internal_finished());
  }
}

void Submap3D::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:cartographer.mapping.proto.Submap3D)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Submap3D::CopyFrom(const Submap3D& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:cartographer.mapping.proto.Submap3D)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Submap3D::IsInitialized() const {
  return true;
}

void Submap3D::InternalSwap(Submap3D* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  rotational_scan_matcher_histogram_.InternalSwap(&other->rotational_scan_matcher_histogram_);
  swap(local_pose_, other->local_pose_);
  swap(high_resolution_hybrid_grid_, other->high_resolution_hybrid_grid_);
  swap(low_resolution_hybrid_grid_, other->low_resolution_hybrid_grid_);
  swap(num_range_data_, other->num_range_data_);
  swap(finished_, other->finished_);
}

::PROTOBUF_NAMESPACE_ID::Metadata Submap3D::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace proto
}  // namespace mapping
}  // namespace cartographer
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::cartographer::mapping::proto::Submap2D* Arena::CreateMaybeMessage< ::cartographer::mapping::proto::Submap2D >(Arena* arena) {
  return Arena::CreateInternal< ::cartographer::mapping::proto::Submap2D >(arena);
}
template<> PROTOBUF_NOINLINE ::cartographer::mapping::proto::Submap3D* Arena::CreateMaybeMessage< ::cartographer::mapping::proto::Submap3D >(Arena* arena) {
  return Arena::CreateInternal< ::cartographer::mapping::proto::Submap3D >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>