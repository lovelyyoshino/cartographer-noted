// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: cartographer/mapping/proto/submaps_options_2d.proto

#include "cartographer/mapping/proto/submaps_options_2d.pb.h"

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
extern PROTOBUF_INTERNAL_EXPORT_cartographer_2fmapping_2fproto_2fgrid_5f2d_5foptions_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_GridOptions2D_cartographer_2fmapping_2fproto_2fgrid_5f2d_5foptions_2eproto;
extern PROTOBUF_INTERNAL_EXPORT_cartographer_2fmapping_2fproto_2frange_5fdata_5finserter_5foptions_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<2> scc_info_RangeDataInserterOptions_cartographer_2fmapping_2fproto_2frange_5fdata_5finserter_5foptions_2eproto;
namespace cartographer {
namespace mapping {
namespace proto {
class SubmapsOptions2DDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<SubmapsOptions2D> _instance;
} _SubmapsOptions2D_default_instance_;
}  // namespace proto
}  // namespace mapping
}  // namespace cartographer
static void InitDefaultsscc_info_SubmapsOptions2D_cartographer_2fmapping_2fproto_2fsubmaps_5foptions_5f2d_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::cartographer::mapping::proto::_SubmapsOptions2D_default_instance_;
    new (ptr) ::cartographer::mapping::proto::SubmapsOptions2D();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::cartographer::mapping::proto::SubmapsOptions2D::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<2> scc_info_SubmapsOptions2D_cartographer_2fmapping_2fproto_2fsubmaps_5foptions_5f2d_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 2, 0, InitDefaultsscc_info_SubmapsOptions2D_cartographer_2fmapping_2fproto_2fsubmaps_5foptions_5f2d_2eproto}, {
      &scc_info_GridOptions2D_cartographer_2fmapping_2fproto_2fgrid_5f2d_5foptions_2eproto.base,
      &scc_info_RangeDataInserterOptions_cartographer_2fmapping_2fproto_2frange_5fdata_5finserter_5foptions_2eproto.base,}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_cartographer_2fmapping_2fproto_2fsubmaps_5foptions_5f2d_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_cartographer_2fmapping_2fproto_2fsubmaps_5foptions_5f2d_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_cartographer_2fmapping_2fproto_2fsubmaps_5foptions_5f2d_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_cartographer_2fmapping_2fproto_2fsubmaps_5foptions_5f2d_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::cartographer::mapping::proto::SubmapsOptions2D, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::cartographer::mapping::proto::SubmapsOptions2D, num_range_data_),
  PROTOBUF_FIELD_OFFSET(::cartographer::mapping::proto::SubmapsOptions2D, grid_options_2d_),
  PROTOBUF_FIELD_OFFSET(::cartographer::mapping::proto::SubmapsOptions2D, range_data_inserter_options_),
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, sizeof(::cartographer::mapping::proto::SubmapsOptions2D)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::cartographer::mapping::proto::_SubmapsOptions2D_default_instance_),
};

const char descriptor_table_protodef_cartographer_2fmapping_2fproto_2fsubmaps_5foptions_5f2d_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n3cartographer/mapping/proto/submaps_opt"
  "ions_2d.proto\022\032cartographer.mapping.prot"
  "o\0320cartographer/mapping/proto/grid_2d_op"
  "tions.proto\032<cartographer/mapping/proto/"
  "range_data_inserter_options.proto\"\311\001\n\020Su"
  "bmapsOptions2D\022\026\n\016num_range_data\030\001 \001(\005\022B"
  "\n\017grid_options_2d\030\002 \001(\0132).cartographer.m"
  "apping.proto.GridOptions2D\022Y\n\033range_data"
  "_inserter_options\030\003 \001(\01324.cartographer.m"
  "apping.proto.RangeDataInserterOptionsb\006p"
  "roto3"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_cartographer_2fmapping_2fproto_2fsubmaps_5foptions_5f2d_2eproto_deps[2] = {
  &::descriptor_table_cartographer_2fmapping_2fproto_2fgrid_5f2d_5foptions_2eproto,
  &::descriptor_table_cartographer_2fmapping_2fproto_2frange_5fdata_5finserter_5foptions_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_cartographer_2fmapping_2fproto_2fsubmaps_5foptions_5f2d_2eproto_sccs[1] = {
  &scc_info_SubmapsOptions2D_cartographer_2fmapping_2fproto_2fsubmaps_5foptions_5f2d_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_cartographer_2fmapping_2fproto_2fsubmaps_5foptions_5f2d_2eproto_once;
static bool descriptor_table_cartographer_2fmapping_2fproto_2fsubmaps_5foptions_5f2d_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_cartographer_2fmapping_2fproto_2fsubmaps_5foptions_5f2d_2eproto = {
  &descriptor_table_cartographer_2fmapping_2fproto_2fsubmaps_5foptions_5f2d_2eproto_initialized, descriptor_table_protodef_cartographer_2fmapping_2fproto_2fsubmaps_5foptions_5f2d_2eproto, "cartographer/mapping/proto/submaps_options_2d.proto", 405,
  &descriptor_table_cartographer_2fmapping_2fproto_2fsubmaps_5foptions_5f2d_2eproto_once, descriptor_table_cartographer_2fmapping_2fproto_2fsubmaps_5foptions_5f2d_2eproto_sccs, descriptor_table_cartographer_2fmapping_2fproto_2fsubmaps_5foptions_5f2d_2eproto_deps, 1, 2,
  schemas, file_default_instances, TableStruct_cartographer_2fmapping_2fproto_2fsubmaps_5foptions_5f2d_2eproto::offsets,
  file_level_metadata_cartographer_2fmapping_2fproto_2fsubmaps_5foptions_5f2d_2eproto, 1, file_level_enum_descriptors_cartographer_2fmapping_2fproto_2fsubmaps_5foptions_5f2d_2eproto, file_level_service_descriptors_cartographer_2fmapping_2fproto_2fsubmaps_5foptions_5f2d_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_cartographer_2fmapping_2fproto_2fsubmaps_5foptions_5f2d_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_cartographer_2fmapping_2fproto_2fsubmaps_5foptions_5f2d_2eproto), true);
namespace cartographer {
namespace mapping {
namespace proto {

// ===================================================================

void SubmapsOptions2D::InitAsDefaultInstance() {
  ::cartographer::mapping::proto::_SubmapsOptions2D_default_instance_._instance.get_mutable()->grid_options_2d_ = const_cast< ::cartographer::mapping::proto::GridOptions2D*>(
      ::cartographer::mapping::proto::GridOptions2D::internal_default_instance());
  ::cartographer::mapping::proto::_SubmapsOptions2D_default_instance_._instance.get_mutable()->range_data_inserter_options_ = const_cast< ::cartographer::mapping::proto::RangeDataInserterOptions*>(
      ::cartographer::mapping::proto::RangeDataInserterOptions::internal_default_instance());
}
class SubmapsOptions2D::_Internal {
 public:
  static const ::cartographer::mapping::proto::GridOptions2D& grid_options_2d(const SubmapsOptions2D* msg);
  static const ::cartographer::mapping::proto::RangeDataInserterOptions& range_data_inserter_options(const SubmapsOptions2D* msg);
};

const ::cartographer::mapping::proto::GridOptions2D&
SubmapsOptions2D::_Internal::grid_options_2d(const SubmapsOptions2D* msg) {
  return *msg->grid_options_2d_;
}
const ::cartographer::mapping::proto::RangeDataInserterOptions&
SubmapsOptions2D::_Internal::range_data_inserter_options(const SubmapsOptions2D* msg) {
  return *msg->range_data_inserter_options_;
}
void SubmapsOptions2D::clear_grid_options_2d() {
  if (GetArenaNoVirtual() == nullptr && grid_options_2d_ != nullptr) {
    delete grid_options_2d_;
  }
  grid_options_2d_ = nullptr;
}
void SubmapsOptions2D::clear_range_data_inserter_options() {
  if (GetArenaNoVirtual() == nullptr && range_data_inserter_options_ != nullptr) {
    delete range_data_inserter_options_;
  }
  range_data_inserter_options_ = nullptr;
}
SubmapsOptions2D::SubmapsOptions2D()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:cartographer.mapping.proto.SubmapsOptions2D)
}
SubmapsOptions2D::SubmapsOptions2D(const SubmapsOptions2D& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  if (from._internal_has_grid_options_2d()) {
    grid_options_2d_ = new ::cartographer::mapping::proto::GridOptions2D(*from.grid_options_2d_);
  } else {
    grid_options_2d_ = nullptr;
  }
  if (from._internal_has_range_data_inserter_options()) {
    range_data_inserter_options_ = new ::cartographer::mapping::proto::RangeDataInserterOptions(*from.range_data_inserter_options_);
  } else {
    range_data_inserter_options_ = nullptr;
  }
  num_range_data_ = from.num_range_data_;
  // @@protoc_insertion_point(copy_constructor:cartographer.mapping.proto.SubmapsOptions2D)
}

void SubmapsOptions2D::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_SubmapsOptions2D_cartographer_2fmapping_2fproto_2fsubmaps_5foptions_5f2d_2eproto.base);
  ::memset(&grid_options_2d_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&num_range_data_) -
      reinterpret_cast<char*>(&grid_options_2d_)) + sizeof(num_range_data_));
}

SubmapsOptions2D::~SubmapsOptions2D() {
  // @@protoc_insertion_point(destructor:cartographer.mapping.proto.SubmapsOptions2D)
  SharedDtor();
}

void SubmapsOptions2D::SharedDtor() {
  if (this != internal_default_instance()) delete grid_options_2d_;
  if (this != internal_default_instance()) delete range_data_inserter_options_;
}

void SubmapsOptions2D::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const SubmapsOptions2D& SubmapsOptions2D::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_SubmapsOptions2D_cartographer_2fmapping_2fproto_2fsubmaps_5foptions_5f2d_2eproto.base);
  return *internal_default_instance();
}


void SubmapsOptions2D::Clear() {
// @@protoc_insertion_point(message_clear_start:cartographer.mapping.proto.SubmapsOptions2D)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  if (GetArenaNoVirtual() == nullptr && grid_options_2d_ != nullptr) {
    delete grid_options_2d_;
  }
  grid_options_2d_ = nullptr;
  if (GetArenaNoVirtual() == nullptr && range_data_inserter_options_ != nullptr) {
    delete range_data_inserter_options_;
  }
  range_data_inserter_options_ = nullptr;
  num_range_data_ = 0;
  _internal_metadata_.Clear();
}

const char* SubmapsOptions2D::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // int32 num_range_data = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 8)) {
          num_range_data_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // .cartographer.mapping.proto.GridOptions2D grid_options_2d = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 18)) {
          ptr = ctx->ParseMessage(_internal_mutable_grid_options_2d(), ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // .cartographer.mapping.proto.RangeDataInserterOptions range_data_inserter_options = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 26)) {
          ptr = ctx->ParseMessage(_internal_mutable_range_data_inserter_options(), ptr);
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

::PROTOBUF_NAMESPACE_ID::uint8* SubmapsOptions2D::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:cartographer.mapping.proto.SubmapsOptions2D)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // int32 num_range_data = 1;
  if (this->num_range_data() != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteInt32ToArray(1, this->_internal_num_range_data(), target);
  }

  // .cartographer.mapping.proto.GridOptions2D grid_options_2d = 2;
  if (this->has_grid_options_2d()) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        2, _Internal::grid_options_2d(this), target, stream);
  }

  // .cartographer.mapping.proto.RangeDataInserterOptions range_data_inserter_options = 3;
  if (this->has_range_data_inserter_options()) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        3, _Internal::range_data_inserter_options(this), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:cartographer.mapping.proto.SubmapsOptions2D)
  return target;
}

size_t SubmapsOptions2D::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:cartographer.mapping.proto.SubmapsOptions2D)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // .cartographer.mapping.proto.GridOptions2D grid_options_2d = 2;
  if (this->has_grid_options_2d()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *grid_options_2d_);
  }

  // .cartographer.mapping.proto.RangeDataInserterOptions range_data_inserter_options = 3;
  if (this->has_range_data_inserter_options()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *range_data_inserter_options_);
  }

  // int32 num_range_data = 1;
  if (this->num_range_data() != 0) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::Int32Size(
        this->_internal_num_range_data());
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    return ::PROTOBUF_NAMESPACE_ID::internal::ComputeUnknownFieldsSize(
        _internal_metadata_, total_size, &_cached_size_);
  }
  int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void SubmapsOptions2D::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:cartographer.mapping.proto.SubmapsOptions2D)
  GOOGLE_DCHECK_NE(&from, this);
  const SubmapsOptions2D* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<SubmapsOptions2D>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:cartographer.mapping.proto.SubmapsOptions2D)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:cartographer.mapping.proto.SubmapsOptions2D)
    MergeFrom(*source);
  }
}

void SubmapsOptions2D::MergeFrom(const SubmapsOptions2D& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:cartographer.mapping.proto.SubmapsOptions2D)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  if (from.has_grid_options_2d()) {
    _internal_mutable_grid_options_2d()->::cartographer::mapping::proto::GridOptions2D::MergeFrom(from._internal_grid_options_2d());
  }
  if (from.has_range_data_inserter_options()) {
    _internal_mutable_range_data_inserter_options()->::cartographer::mapping::proto::RangeDataInserterOptions::MergeFrom(from._internal_range_data_inserter_options());
  }
  if (from.num_range_data() != 0) {
    _internal_set_num_range_data(from._internal_num_range_data());
  }
}

void SubmapsOptions2D::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:cartographer.mapping.proto.SubmapsOptions2D)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void SubmapsOptions2D::CopyFrom(const SubmapsOptions2D& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:cartographer.mapping.proto.SubmapsOptions2D)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool SubmapsOptions2D::IsInitialized() const {
  return true;
}

void SubmapsOptions2D::InternalSwap(SubmapsOptions2D* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(grid_options_2d_, other->grid_options_2d_);
  swap(range_data_inserter_options_, other->range_data_inserter_options_);
  swap(num_range_data_, other->num_range_data_);
}

::PROTOBUF_NAMESPACE_ID::Metadata SubmapsOptions2D::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace proto
}  // namespace mapping
}  // namespace cartographer
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::cartographer::mapping::proto::SubmapsOptions2D* Arena::CreateMaybeMessage< ::cartographer::mapping::proto::SubmapsOptions2D >(Arena* arena) {
  return Arena::CreateInternal< ::cartographer::mapping::proto::SubmapsOptions2D >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
