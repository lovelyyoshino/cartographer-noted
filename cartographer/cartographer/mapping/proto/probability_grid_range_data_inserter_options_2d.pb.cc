// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: cartographer/mapping/proto/probability_grid_range_data_inserter_options_2d.proto

#include "cartographer/mapping/proto/probability_grid_range_data_inserter_options_2d.pb.h"

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
namespace cartographer {
namespace mapping {
namespace proto {
class ProbabilityGridRangeDataInserterOptions2DDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<ProbabilityGridRangeDataInserterOptions2D> _instance;
} _ProbabilityGridRangeDataInserterOptions2D_default_instance_;
}  // namespace proto
}  // namespace mapping
}  // namespace cartographer
static void InitDefaultsscc_info_ProbabilityGridRangeDataInserterOptions2D_cartographer_2fmapping_2fproto_2fprobability_5fgrid_5frange_5fdata_5finserter_5foptions_5f2d_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::cartographer::mapping::proto::_ProbabilityGridRangeDataInserterOptions2D_default_instance_;
    new (ptr) ::cartographer::mapping::proto::ProbabilityGridRangeDataInserterOptions2D();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::cartographer::mapping::proto::ProbabilityGridRangeDataInserterOptions2D::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_ProbabilityGridRangeDataInserterOptions2D_cartographer_2fmapping_2fproto_2fprobability_5fgrid_5frange_5fdata_5finserter_5foptions_5f2d_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 0, 0, InitDefaultsscc_info_ProbabilityGridRangeDataInserterOptions2D_cartographer_2fmapping_2fproto_2fprobability_5fgrid_5frange_5fdata_5finserter_5foptions_5f2d_2eproto}, {}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_cartographer_2fmapping_2fproto_2fprobability_5fgrid_5frange_5fdata_5finserter_5foptions_5f2d_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_cartographer_2fmapping_2fproto_2fprobability_5fgrid_5frange_5fdata_5finserter_5foptions_5f2d_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_cartographer_2fmapping_2fproto_2fprobability_5fgrid_5frange_5fdata_5finserter_5foptions_5f2d_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_cartographer_2fmapping_2fproto_2fprobability_5fgrid_5frange_5fdata_5finserter_5foptions_5f2d_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::cartographer::mapping::proto::ProbabilityGridRangeDataInserterOptions2D, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::cartographer::mapping::proto::ProbabilityGridRangeDataInserterOptions2D, hit_probability_),
  PROTOBUF_FIELD_OFFSET(::cartographer::mapping::proto::ProbabilityGridRangeDataInserterOptions2D, miss_probability_),
  PROTOBUF_FIELD_OFFSET(::cartographer::mapping::proto::ProbabilityGridRangeDataInserterOptions2D, insert_free_space_),
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, sizeof(::cartographer::mapping::proto::ProbabilityGridRangeDataInserterOptions2D)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::cartographer::mapping::proto::_ProbabilityGridRangeDataInserterOptions2D_default_instance_),
};

const char descriptor_table_protodef_cartographer_2fmapping_2fproto_2fprobability_5fgrid_5frange_5fdata_5finserter_5foptions_5f2d_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\nPcartographer/mapping/proto/probability"
  "_grid_range_data_inserter_options_2d.pro"
  "to\022\032cartographer.mapping.proto\"y\n)Probab"
  "ilityGridRangeDataInserterOptions2D\022\027\n\017h"
  "it_probability\030\001 \001(\001\022\030\n\020miss_probability"
  "\030\002 \001(\001\022\031\n\021insert_free_space\030\003 \001(\010b\006proto"
  "3"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_cartographer_2fmapping_2fproto_2fprobability_5fgrid_5frange_5fdata_5finserter_5foptions_5f2d_2eproto_deps[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_cartographer_2fmapping_2fproto_2fprobability_5fgrid_5frange_5fdata_5finserter_5foptions_5f2d_2eproto_sccs[1] = {
  &scc_info_ProbabilityGridRangeDataInserterOptions2D_cartographer_2fmapping_2fproto_2fprobability_5fgrid_5frange_5fdata_5finserter_5foptions_5f2d_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_cartographer_2fmapping_2fproto_2fprobability_5fgrid_5frange_5fdata_5finserter_5foptions_5f2d_2eproto_once;
static bool descriptor_table_cartographer_2fmapping_2fproto_2fprobability_5fgrid_5frange_5fdata_5finserter_5foptions_5f2d_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_cartographer_2fmapping_2fproto_2fprobability_5fgrid_5frange_5fdata_5finserter_5foptions_5f2d_2eproto = {
  &descriptor_table_cartographer_2fmapping_2fproto_2fprobability_5fgrid_5frange_5fdata_5finserter_5foptions_5f2d_2eproto_initialized, descriptor_table_protodef_cartographer_2fmapping_2fproto_2fprobability_5fgrid_5frange_5fdata_5finserter_5foptions_5f2d_2eproto, "cartographer/mapping/proto/probability_grid_range_data_inserter_options_2d.proto", 241,
  &descriptor_table_cartographer_2fmapping_2fproto_2fprobability_5fgrid_5frange_5fdata_5finserter_5foptions_5f2d_2eproto_once, descriptor_table_cartographer_2fmapping_2fproto_2fprobability_5fgrid_5frange_5fdata_5finserter_5foptions_5f2d_2eproto_sccs, descriptor_table_cartographer_2fmapping_2fproto_2fprobability_5fgrid_5frange_5fdata_5finserter_5foptions_5f2d_2eproto_deps, 1, 0,
  schemas, file_default_instances, TableStruct_cartographer_2fmapping_2fproto_2fprobability_5fgrid_5frange_5fdata_5finserter_5foptions_5f2d_2eproto::offsets,
  file_level_metadata_cartographer_2fmapping_2fproto_2fprobability_5fgrid_5frange_5fdata_5finserter_5foptions_5f2d_2eproto, 1, file_level_enum_descriptors_cartographer_2fmapping_2fproto_2fprobability_5fgrid_5frange_5fdata_5finserter_5foptions_5f2d_2eproto, file_level_service_descriptors_cartographer_2fmapping_2fproto_2fprobability_5fgrid_5frange_5fdata_5finserter_5foptions_5f2d_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_cartographer_2fmapping_2fproto_2fprobability_5fgrid_5frange_5fdata_5finserter_5foptions_5f2d_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_cartographer_2fmapping_2fproto_2fprobability_5fgrid_5frange_5fdata_5finserter_5foptions_5f2d_2eproto), true);
namespace cartographer {
namespace mapping {
namespace proto {

// ===================================================================

void ProbabilityGridRangeDataInserterOptions2D::InitAsDefaultInstance() {
}
class ProbabilityGridRangeDataInserterOptions2D::_Internal {
 public:
};

ProbabilityGridRangeDataInserterOptions2D::ProbabilityGridRangeDataInserterOptions2D()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:cartographer.mapping.proto.ProbabilityGridRangeDataInserterOptions2D)
}
ProbabilityGridRangeDataInserterOptions2D::ProbabilityGridRangeDataInserterOptions2D(const ProbabilityGridRangeDataInserterOptions2D& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::memcpy(&hit_probability_, &from.hit_probability_,
    static_cast<size_t>(reinterpret_cast<char*>(&insert_free_space_) -
    reinterpret_cast<char*>(&hit_probability_)) + sizeof(insert_free_space_));
  // @@protoc_insertion_point(copy_constructor:cartographer.mapping.proto.ProbabilityGridRangeDataInserterOptions2D)
}

void ProbabilityGridRangeDataInserterOptions2D::SharedCtor() {
  ::memset(&hit_probability_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&insert_free_space_) -
      reinterpret_cast<char*>(&hit_probability_)) + sizeof(insert_free_space_));
}

ProbabilityGridRangeDataInserterOptions2D::~ProbabilityGridRangeDataInserterOptions2D() {
  // @@protoc_insertion_point(destructor:cartographer.mapping.proto.ProbabilityGridRangeDataInserterOptions2D)
  SharedDtor();
}

void ProbabilityGridRangeDataInserterOptions2D::SharedDtor() {
}

void ProbabilityGridRangeDataInserterOptions2D::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ProbabilityGridRangeDataInserterOptions2D& ProbabilityGridRangeDataInserterOptions2D::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_ProbabilityGridRangeDataInserterOptions2D_cartographer_2fmapping_2fproto_2fprobability_5fgrid_5frange_5fdata_5finserter_5foptions_5f2d_2eproto.base);
  return *internal_default_instance();
}


void ProbabilityGridRangeDataInserterOptions2D::Clear() {
// @@protoc_insertion_point(message_clear_start:cartographer.mapping.proto.ProbabilityGridRangeDataInserterOptions2D)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  ::memset(&hit_probability_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&insert_free_space_) -
      reinterpret_cast<char*>(&hit_probability_)) + sizeof(insert_free_space_));
  _internal_metadata_.Clear();
}

const char* ProbabilityGridRangeDataInserterOptions2D::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // double hit_probability = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 9)) {
          hit_probability_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else goto handle_unusual;
        continue;
      // double miss_probability = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 17)) {
          miss_probability_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else goto handle_unusual;
        continue;
      // bool insert_free_space = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 24)) {
          insert_free_space_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
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

::PROTOBUF_NAMESPACE_ID::uint8* ProbabilityGridRangeDataInserterOptions2D::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:cartographer.mapping.proto.ProbabilityGridRangeDataInserterOptions2D)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // double hit_probability = 1;
  if (!(this->hit_probability() <= 0 && this->hit_probability() >= 0)) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(1, this->_internal_hit_probability(), target);
  }

  // double miss_probability = 2;
  if (!(this->miss_probability() <= 0 && this->miss_probability() >= 0)) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(2, this->_internal_miss_probability(), target);
  }

  // bool insert_free_space = 3;
  if (this->insert_free_space() != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(3, this->_internal_insert_free_space(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:cartographer.mapping.proto.ProbabilityGridRangeDataInserterOptions2D)
  return target;
}

size_t ProbabilityGridRangeDataInserterOptions2D::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:cartographer.mapping.proto.ProbabilityGridRangeDataInserterOptions2D)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // double hit_probability = 1;
  if (!(this->hit_probability() <= 0 && this->hit_probability() >= 0)) {
    total_size += 1 + 8;
  }

  // double miss_probability = 2;
  if (!(this->miss_probability() <= 0 && this->miss_probability() >= 0)) {
    total_size += 1 + 8;
  }

  // bool insert_free_space = 3;
  if (this->insert_free_space() != 0) {
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

void ProbabilityGridRangeDataInserterOptions2D::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:cartographer.mapping.proto.ProbabilityGridRangeDataInserterOptions2D)
  GOOGLE_DCHECK_NE(&from, this);
  const ProbabilityGridRangeDataInserterOptions2D* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<ProbabilityGridRangeDataInserterOptions2D>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:cartographer.mapping.proto.ProbabilityGridRangeDataInserterOptions2D)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:cartographer.mapping.proto.ProbabilityGridRangeDataInserterOptions2D)
    MergeFrom(*source);
  }
}

void ProbabilityGridRangeDataInserterOptions2D::MergeFrom(const ProbabilityGridRangeDataInserterOptions2D& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:cartographer.mapping.proto.ProbabilityGridRangeDataInserterOptions2D)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  if (!(from.hit_probability() <= 0 && from.hit_probability() >= 0)) {
    _internal_set_hit_probability(from._internal_hit_probability());
  }
  if (!(from.miss_probability() <= 0 && from.miss_probability() >= 0)) {
    _internal_set_miss_probability(from._internal_miss_probability());
  }
  if (from.insert_free_space() != 0) {
    _internal_set_insert_free_space(from._internal_insert_free_space());
  }
}

void ProbabilityGridRangeDataInserterOptions2D::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:cartographer.mapping.proto.ProbabilityGridRangeDataInserterOptions2D)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void ProbabilityGridRangeDataInserterOptions2D::CopyFrom(const ProbabilityGridRangeDataInserterOptions2D& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:cartographer.mapping.proto.ProbabilityGridRangeDataInserterOptions2D)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool ProbabilityGridRangeDataInserterOptions2D::IsInitialized() const {
  return true;
}

void ProbabilityGridRangeDataInserterOptions2D::InternalSwap(ProbabilityGridRangeDataInserterOptions2D* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(hit_probability_, other->hit_probability_);
  swap(miss_probability_, other->miss_probability_);
  swap(insert_free_space_, other->insert_free_space_);
}

::PROTOBUF_NAMESPACE_ID::Metadata ProbabilityGridRangeDataInserterOptions2D::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace proto
}  // namespace mapping
}  // namespace cartographer
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::cartographer::mapping::proto::ProbabilityGridRangeDataInserterOptions2D* Arena::CreateMaybeMessage< ::cartographer::mapping::proto::ProbabilityGridRangeDataInserterOptions2D >(Arena* arena) {
  return Arena::CreateInternal< ::cartographer::mapping::proto::ProbabilityGridRangeDataInserterOptions2D >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>