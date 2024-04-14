// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: cartographer/mapping/proto/range_data_inserter_options.proto

#include "cartographer/mapping/proto/range_data_inserter_options.pb.h"

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
extern PROTOBUF_INTERNAL_EXPORT_cartographer_2fmapping_2fproto_2fprobability_5fgrid_5frange_5fdata_5finserter_5foptions_5f2d_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_ProbabilityGridRangeDataInserterOptions2D_cartographer_2fmapping_2fproto_2fprobability_5fgrid_5frange_5fdata_5finserter_5foptions_5f2d_2eproto;
extern PROTOBUF_INTERNAL_EXPORT_cartographer_2fmapping_2fproto_2ftsdf_5frange_5fdata_5finserter_5foptions_5f2d_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_TSDFRangeDataInserterOptions2D_cartographer_2fmapping_2fproto_2ftsdf_5frange_5fdata_5finserter_5foptions_5f2d_2eproto;
namespace cartographer {
namespace mapping {
namespace proto {
class RangeDataInserterOptionsDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<RangeDataInserterOptions> _instance;
} _RangeDataInserterOptions_default_instance_;
}  // namespace proto
}  // namespace mapping
}  // namespace cartographer
static void InitDefaultsscc_info_RangeDataInserterOptions_cartographer_2fmapping_2fproto_2frange_5fdata_5finserter_5foptions_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::cartographer::mapping::proto::_RangeDataInserterOptions_default_instance_;
    new (ptr) ::cartographer::mapping::proto::RangeDataInserterOptions();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::cartographer::mapping::proto::RangeDataInserterOptions::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<2> scc_info_RangeDataInserterOptions_cartographer_2fmapping_2fproto_2frange_5fdata_5finserter_5foptions_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 2, 0, InitDefaultsscc_info_RangeDataInserterOptions_cartographer_2fmapping_2fproto_2frange_5fdata_5finserter_5foptions_2eproto}, {
      &scc_info_ProbabilityGridRangeDataInserterOptions2D_cartographer_2fmapping_2fproto_2fprobability_5fgrid_5frange_5fdata_5finserter_5foptions_5f2d_2eproto.base,
      &scc_info_TSDFRangeDataInserterOptions2D_cartographer_2fmapping_2fproto_2ftsdf_5frange_5fdata_5finserter_5foptions_5f2d_2eproto.base,}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_cartographer_2fmapping_2fproto_2frange_5fdata_5finserter_5foptions_2eproto[1];
static const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* file_level_enum_descriptors_cartographer_2fmapping_2fproto_2frange_5fdata_5finserter_5foptions_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_cartographer_2fmapping_2fproto_2frange_5fdata_5finserter_5foptions_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_cartographer_2fmapping_2fproto_2frange_5fdata_5finserter_5foptions_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::cartographer::mapping::proto::RangeDataInserterOptions, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::cartographer::mapping::proto::RangeDataInserterOptions, range_data_inserter_type_),
  PROTOBUF_FIELD_OFFSET(::cartographer::mapping::proto::RangeDataInserterOptions, probability_grid_range_data_inserter_options_2d_),
  PROTOBUF_FIELD_OFFSET(::cartographer::mapping::proto::RangeDataInserterOptions, tsdf_range_data_inserter_options_2d_),
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, sizeof(::cartographer::mapping::proto::RangeDataInserterOptions)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::cartographer::mapping::proto::_RangeDataInserterOptions_default_instance_),
};

const char descriptor_table_protodef_cartographer_2fmapping_2fproto_2frange_5fdata_5finserter_5foptions_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n<cartographer/mapping/proto/range_data_"
  "inserter_options.proto\022\032cartographer.map"
  "ping.proto\032Pcartographer/mapping/proto/p"
  "robability_grid_range_data_inserter_opti"
  "ons_2d.proto\032Dcartographer/mapping/proto"
  "/tsdf_range_data_inserter_options_2d.pro"
  "to\"\330\003\n\030RangeDataInserterOptions\022l\n\030range"
  "_data_inserter_type\030\001 \001(\0162J.cartographer"
  ".mapping.proto.RangeDataInserterOptions."
  "RangeDataInserterType\022~\n/probability_gri"
  "d_range_data_inserter_options_2d\030\002 \001(\0132E"
  ".cartographer.mapping.proto.ProbabilityG"
  "ridRangeDataInserterOptions2D\022g\n#tsdf_ra"
  "nge_data_inserter_options_2d\030\003 \001(\0132:.car"
  "tographer.mapping.proto.TSDFRangeDataIns"
  "erterOptions2D\"e\n\025RangeDataInserterType\022"
  "\024\n\020INVALID_INSERTER\020\000\022 \n\034PROBABILITY_GRI"
  "D_INSERTER_2D\020\001\022\024\n\020TSDF_INSERTER_2D\020\002b\006p"
  "roto3"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_cartographer_2fmapping_2fproto_2frange_5fdata_5finserter_5foptions_2eproto_deps[2] = {
  &::descriptor_table_cartographer_2fmapping_2fproto_2fprobability_5fgrid_5frange_5fdata_5finserter_5foptions_5f2d_2eproto,
  &::descriptor_table_cartographer_2fmapping_2fproto_2ftsdf_5frange_5fdata_5finserter_5foptions_5f2d_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_cartographer_2fmapping_2fproto_2frange_5fdata_5finserter_5foptions_2eproto_sccs[1] = {
  &scc_info_RangeDataInserterOptions_cartographer_2fmapping_2fproto_2frange_5fdata_5finserter_5foptions_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_cartographer_2fmapping_2fproto_2frange_5fdata_5finserter_5foptions_2eproto_once;
static bool descriptor_table_cartographer_2fmapping_2fproto_2frange_5fdata_5finserter_5foptions_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_cartographer_2fmapping_2fproto_2frange_5fdata_5finserter_5foptions_2eproto = {
  &descriptor_table_cartographer_2fmapping_2fproto_2frange_5fdata_5finserter_5foptions_2eproto_initialized, descriptor_table_protodef_cartographer_2fmapping_2fproto_2frange_5fdata_5finserter_5foptions_2eproto, "cartographer/mapping/proto/range_data_inserter_options.proto", 725,
  &descriptor_table_cartographer_2fmapping_2fproto_2frange_5fdata_5finserter_5foptions_2eproto_once, descriptor_table_cartographer_2fmapping_2fproto_2frange_5fdata_5finserter_5foptions_2eproto_sccs, descriptor_table_cartographer_2fmapping_2fproto_2frange_5fdata_5finserter_5foptions_2eproto_deps, 1, 2,
  schemas, file_default_instances, TableStruct_cartographer_2fmapping_2fproto_2frange_5fdata_5finserter_5foptions_2eproto::offsets,
  file_level_metadata_cartographer_2fmapping_2fproto_2frange_5fdata_5finserter_5foptions_2eproto, 1, file_level_enum_descriptors_cartographer_2fmapping_2fproto_2frange_5fdata_5finserter_5foptions_2eproto, file_level_service_descriptors_cartographer_2fmapping_2fproto_2frange_5fdata_5finserter_5foptions_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_cartographer_2fmapping_2fproto_2frange_5fdata_5finserter_5foptions_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_cartographer_2fmapping_2fproto_2frange_5fdata_5finserter_5foptions_2eproto), true);
namespace cartographer {
namespace mapping {
namespace proto {
const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* RangeDataInserterOptions_RangeDataInserterType_descriptor() {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_cartographer_2fmapping_2fproto_2frange_5fdata_5finserter_5foptions_2eproto);
  return file_level_enum_descriptors_cartographer_2fmapping_2fproto_2frange_5fdata_5finserter_5foptions_2eproto[0];
}
bool RangeDataInserterOptions_RangeDataInserterType_IsValid(int value) {
  switch (value) {
    case 0:
    case 1:
    case 2:
      return true;
    default:
      return false;
  }
}

#if (__cplusplus < 201703) && (!defined(_MSC_VER) || _MSC_VER >= 1900)
constexpr RangeDataInserterOptions_RangeDataInserterType RangeDataInserterOptions::INVALID_INSERTER;
constexpr RangeDataInserterOptions_RangeDataInserterType RangeDataInserterOptions::PROBABILITY_GRID_INSERTER_2D;
constexpr RangeDataInserterOptions_RangeDataInserterType RangeDataInserterOptions::TSDF_INSERTER_2D;
constexpr RangeDataInserterOptions_RangeDataInserterType RangeDataInserterOptions::RangeDataInserterType_MIN;
constexpr RangeDataInserterOptions_RangeDataInserterType RangeDataInserterOptions::RangeDataInserterType_MAX;
constexpr int RangeDataInserterOptions::RangeDataInserterType_ARRAYSIZE;
#endif  // (__cplusplus < 201703) && (!defined(_MSC_VER) || _MSC_VER >= 1900)

// ===================================================================

void RangeDataInserterOptions::InitAsDefaultInstance() {
  ::cartographer::mapping::proto::_RangeDataInserterOptions_default_instance_._instance.get_mutable()->probability_grid_range_data_inserter_options_2d_ = const_cast< ::cartographer::mapping::proto::ProbabilityGridRangeDataInserterOptions2D*>(
      ::cartographer::mapping::proto::ProbabilityGridRangeDataInserterOptions2D::internal_default_instance());
  ::cartographer::mapping::proto::_RangeDataInserterOptions_default_instance_._instance.get_mutable()->tsdf_range_data_inserter_options_2d_ = const_cast< ::cartographer::mapping::proto::TSDFRangeDataInserterOptions2D*>(
      ::cartographer::mapping::proto::TSDFRangeDataInserterOptions2D::internal_default_instance());
}
class RangeDataInserterOptions::_Internal {
 public:
  static const ::cartographer::mapping::proto::ProbabilityGridRangeDataInserterOptions2D& probability_grid_range_data_inserter_options_2d(const RangeDataInserterOptions* msg);
  static const ::cartographer::mapping::proto::TSDFRangeDataInserterOptions2D& tsdf_range_data_inserter_options_2d(const RangeDataInserterOptions* msg);
};

const ::cartographer::mapping::proto::ProbabilityGridRangeDataInserterOptions2D&
RangeDataInserterOptions::_Internal::probability_grid_range_data_inserter_options_2d(const RangeDataInserterOptions* msg) {
  return *msg->probability_grid_range_data_inserter_options_2d_;
}
const ::cartographer::mapping::proto::TSDFRangeDataInserterOptions2D&
RangeDataInserterOptions::_Internal::tsdf_range_data_inserter_options_2d(const RangeDataInserterOptions* msg) {
  return *msg->tsdf_range_data_inserter_options_2d_;
}
void RangeDataInserterOptions::clear_probability_grid_range_data_inserter_options_2d() {
  if (GetArenaNoVirtual() == nullptr && probability_grid_range_data_inserter_options_2d_ != nullptr) {
    delete probability_grid_range_data_inserter_options_2d_;
  }
  probability_grid_range_data_inserter_options_2d_ = nullptr;
}
void RangeDataInserterOptions::clear_tsdf_range_data_inserter_options_2d() {
  if (GetArenaNoVirtual() == nullptr && tsdf_range_data_inserter_options_2d_ != nullptr) {
    delete tsdf_range_data_inserter_options_2d_;
  }
  tsdf_range_data_inserter_options_2d_ = nullptr;
}
RangeDataInserterOptions::RangeDataInserterOptions()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:cartographer.mapping.proto.RangeDataInserterOptions)
}
RangeDataInserterOptions::RangeDataInserterOptions(const RangeDataInserterOptions& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  if (from._internal_has_probability_grid_range_data_inserter_options_2d()) {
    probability_grid_range_data_inserter_options_2d_ = new ::cartographer::mapping::proto::ProbabilityGridRangeDataInserterOptions2D(*from.probability_grid_range_data_inserter_options_2d_);
  } else {
    probability_grid_range_data_inserter_options_2d_ = nullptr;
  }
  if (from._internal_has_tsdf_range_data_inserter_options_2d()) {
    tsdf_range_data_inserter_options_2d_ = new ::cartographer::mapping::proto::TSDFRangeDataInserterOptions2D(*from.tsdf_range_data_inserter_options_2d_);
  } else {
    tsdf_range_data_inserter_options_2d_ = nullptr;
  }
  range_data_inserter_type_ = from.range_data_inserter_type_;
  // @@protoc_insertion_point(copy_constructor:cartographer.mapping.proto.RangeDataInserterOptions)
}

void RangeDataInserterOptions::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_RangeDataInserterOptions_cartographer_2fmapping_2fproto_2frange_5fdata_5finserter_5foptions_2eproto.base);
  ::memset(&probability_grid_range_data_inserter_options_2d_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&range_data_inserter_type_) -
      reinterpret_cast<char*>(&probability_grid_range_data_inserter_options_2d_)) + sizeof(range_data_inserter_type_));
}

RangeDataInserterOptions::~RangeDataInserterOptions() {
  // @@protoc_insertion_point(destructor:cartographer.mapping.proto.RangeDataInserterOptions)
  SharedDtor();
}

void RangeDataInserterOptions::SharedDtor() {
  if (this != internal_default_instance()) delete probability_grid_range_data_inserter_options_2d_;
  if (this != internal_default_instance()) delete tsdf_range_data_inserter_options_2d_;
}

void RangeDataInserterOptions::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const RangeDataInserterOptions& RangeDataInserterOptions::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_RangeDataInserterOptions_cartographer_2fmapping_2fproto_2frange_5fdata_5finserter_5foptions_2eproto.base);
  return *internal_default_instance();
}


void RangeDataInserterOptions::Clear() {
// @@protoc_insertion_point(message_clear_start:cartographer.mapping.proto.RangeDataInserterOptions)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  if (GetArenaNoVirtual() == nullptr && probability_grid_range_data_inserter_options_2d_ != nullptr) {
    delete probability_grid_range_data_inserter_options_2d_;
  }
  probability_grid_range_data_inserter_options_2d_ = nullptr;
  if (GetArenaNoVirtual() == nullptr && tsdf_range_data_inserter_options_2d_ != nullptr) {
    delete tsdf_range_data_inserter_options_2d_;
  }
  tsdf_range_data_inserter_options_2d_ = nullptr;
  range_data_inserter_type_ = 0;
  _internal_metadata_.Clear();
}

const char* RangeDataInserterOptions::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // .cartographer.mapping.proto.RangeDataInserterOptions.RangeDataInserterType range_data_inserter_type = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 8)) {
          ::PROTOBUF_NAMESPACE_ID::uint64 val = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
          _internal_set_range_data_inserter_type(static_cast<::cartographer::mapping::proto::RangeDataInserterOptions_RangeDataInserterType>(val));
        } else goto handle_unusual;
        continue;
      // .cartographer.mapping.proto.ProbabilityGridRangeDataInserterOptions2D probability_grid_range_data_inserter_options_2d = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 18)) {
          ptr = ctx->ParseMessage(_internal_mutable_probability_grid_range_data_inserter_options_2d(), ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // .cartographer.mapping.proto.TSDFRangeDataInserterOptions2D tsdf_range_data_inserter_options_2d = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 26)) {
          ptr = ctx->ParseMessage(_internal_mutable_tsdf_range_data_inserter_options_2d(), ptr);
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

::PROTOBUF_NAMESPACE_ID::uint8* RangeDataInserterOptions::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:cartographer.mapping.proto.RangeDataInserterOptions)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // .cartographer.mapping.proto.RangeDataInserterOptions.RangeDataInserterType range_data_inserter_type = 1;
  if (this->range_data_inserter_type() != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteEnumToArray(
      1, this->_internal_range_data_inserter_type(), target);
  }

  // .cartographer.mapping.proto.ProbabilityGridRangeDataInserterOptions2D probability_grid_range_data_inserter_options_2d = 2;
  if (this->has_probability_grid_range_data_inserter_options_2d()) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        2, _Internal::probability_grid_range_data_inserter_options_2d(this), target, stream);
  }

  // .cartographer.mapping.proto.TSDFRangeDataInserterOptions2D tsdf_range_data_inserter_options_2d = 3;
  if (this->has_tsdf_range_data_inserter_options_2d()) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        3, _Internal::tsdf_range_data_inserter_options_2d(this), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:cartographer.mapping.proto.RangeDataInserterOptions)
  return target;
}

size_t RangeDataInserterOptions::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:cartographer.mapping.proto.RangeDataInserterOptions)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // .cartographer.mapping.proto.ProbabilityGridRangeDataInserterOptions2D probability_grid_range_data_inserter_options_2d = 2;
  if (this->has_probability_grid_range_data_inserter_options_2d()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *probability_grid_range_data_inserter_options_2d_);
  }

  // .cartographer.mapping.proto.TSDFRangeDataInserterOptions2D tsdf_range_data_inserter_options_2d = 3;
  if (this->has_tsdf_range_data_inserter_options_2d()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *tsdf_range_data_inserter_options_2d_);
  }

  // .cartographer.mapping.proto.RangeDataInserterOptions.RangeDataInserterType range_data_inserter_type = 1;
  if (this->range_data_inserter_type() != 0) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::EnumSize(this->_internal_range_data_inserter_type());
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    return ::PROTOBUF_NAMESPACE_ID::internal::ComputeUnknownFieldsSize(
        _internal_metadata_, total_size, &_cached_size_);
  }
  int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void RangeDataInserterOptions::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:cartographer.mapping.proto.RangeDataInserterOptions)
  GOOGLE_DCHECK_NE(&from, this);
  const RangeDataInserterOptions* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<RangeDataInserterOptions>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:cartographer.mapping.proto.RangeDataInserterOptions)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:cartographer.mapping.proto.RangeDataInserterOptions)
    MergeFrom(*source);
  }
}

void RangeDataInserterOptions::MergeFrom(const RangeDataInserterOptions& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:cartographer.mapping.proto.RangeDataInserterOptions)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  if (from.has_probability_grid_range_data_inserter_options_2d()) {
    _internal_mutable_probability_grid_range_data_inserter_options_2d()->::cartographer::mapping::proto::ProbabilityGridRangeDataInserterOptions2D::MergeFrom(from._internal_probability_grid_range_data_inserter_options_2d());
  }
  if (from.has_tsdf_range_data_inserter_options_2d()) {
    _internal_mutable_tsdf_range_data_inserter_options_2d()->::cartographer::mapping::proto::TSDFRangeDataInserterOptions2D::MergeFrom(from._internal_tsdf_range_data_inserter_options_2d());
  }
  if (from.range_data_inserter_type() != 0) {
    _internal_set_range_data_inserter_type(from._internal_range_data_inserter_type());
  }
}

void RangeDataInserterOptions::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:cartographer.mapping.proto.RangeDataInserterOptions)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void RangeDataInserterOptions::CopyFrom(const RangeDataInserterOptions& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:cartographer.mapping.proto.RangeDataInserterOptions)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool RangeDataInserterOptions::IsInitialized() const {
  return true;
}

void RangeDataInserterOptions::InternalSwap(RangeDataInserterOptions* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(probability_grid_range_data_inserter_options_2d_, other->probability_grid_range_data_inserter_options_2d_);
  swap(tsdf_range_data_inserter_options_2d_, other->tsdf_range_data_inserter_options_2d_);
  swap(range_data_inserter_type_, other->range_data_inserter_type_);
}

::PROTOBUF_NAMESPACE_ID::Metadata RangeDataInserterOptions::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace proto
}  // namespace mapping
}  // namespace cartographer
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::cartographer::mapping::proto::RangeDataInserterOptions* Arena::CreateMaybeMessage< ::cartographer::mapping::proto::RangeDataInserterOptions >(Arena* arena) {
  return Arena::CreateInternal< ::cartographer::mapping::proto::RangeDataInserterOptions >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>