// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: cartographer/mapping/proto/connected_components.proto

#include "cartographer/mapping/proto/connected_components.pb.h"

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
extern PROTOBUF_INTERNAL_EXPORT_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_ConnectedComponents_ConnectedComponent_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto;
namespace cartographer {
namespace mapping {
namespace proto {
class ConnectedComponents_ConnectedComponentDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<ConnectedComponents_ConnectedComponent> _instance;
} _ConnectedComponents_ConnectedComponent_default_instance_;
class ConnectedComponentsDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<ConnectedComponents> _instance;
} _ConnectedComponents_default_instance_;
}  // namespace proto
}  // namespace mapping
}  // namespace cartographer
static void InitDefaultsscc_info_ConnectedComponents_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::cartographer::mapping::proto::_ConnectedComponents_default_instance_;
    new (ptr) ::cartographer::mapping::proto::ConnectedComponents();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::cartographer::mapping::proto::ConnectedComponents::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_ConnectedComponents_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_ConnectedComponents_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto}, {
      &scc_info_ConnectedComponents_ConnectedComponent_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto.base,}};

static void InitDefaultsscc_info_ConnectedComponents_ConnectedComponent_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::cartographer::mapping::proto::_ConnectedComponents_ConnectedComponent_default_instance_;
    new (ptr) ::cartographer::mapping::proto::ConnectedComponents_ConnectedComponent();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::cartographer::mapping::proto::ConnectedComponents_ConnectedComponent::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_ConnectedComponents_ConnectedComponent_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 0, 0, InitDefaultsscc_info_ConnectedComponents_ConnectedComponent_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto}, {}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto[2];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::cartographer::mapping::proto::ConnectedComponents_ConnectedComponent, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::cartographer::mapping::proto::ConnectedComponents_ConnectedComponent, trajectory_id_),
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::cartographer::mapping::proto::ConnectedComponents, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::cartographer::mapping::proto::ConnectedComponents, connected_component_),
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, sizeof(::cartographer::mapping::proto::ConnectedComponents_ConnectedComponent)},
  { 6, -1, sizeof(::cartographer::mapping::proto::ConnectedComponents)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::cartographer::mapping::proto::_ConnectedComponents_ConnectedComponent_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::cartographer::mapping::proto::_ConnectedComponents_default_instance_),
};

const char descriptor_table_protodef_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n5cartographer/mapping/proto/connected_c"
  "omponents.proto\022\032cartographer.mapping.pr"
  "oto\"\243\001\n\023ConnectedComponents\022_\n\023connected"
  "_component\030\001 \003(\0132B.cartographer.mapping."
  "proto.ConnectedComponents.ConnectedCompo"
  "nent\032+\n\022ConnectedComponent\022\025\n\rtrajectory"
  "_id\030\001 \003(\005B\037B\035ConnectedComponentsOuterCla"
  "ssb\006proto3"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto_deps[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto_sccs[2] = {
  &scc_info_ConnectedComponents_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto.base,
  &scc_info_ConnectedComponents_ConnectedComponent_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto_once;
static bool descriptor_table_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto = {
  &descriptor_table_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto_initialized, descriptor_table_protodef_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto, "cartographer/mapping/proto/connected_components.proto", 290,
  &descriptor_table_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto_once, descriptor_table_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto_sccs, descriptor_table_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto_deps, 2, 0,
  schemas, file_default_instances, TableStruct_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto::offsets,
  file_level_metadata_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto, 2, file_level_enum_descriptors_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto, file_level_service_descriptors_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto), true);
namespace cartographer {
namespace mapping {
namespace proto {

// ===================================================================

void ConnectedComponents_ConnectedComponent::InitAsDefaultInstance() {
}
class ConnectedComponents_ConnectedComponent::_Internal {
 public:
};

ConnectedComponents_ConnectedComponent::ConnectedComponents_ConnectedComponent()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:cartographer.mapping.proto.ConnectedComponents.ConnectedComponent)
}
ConnectedComponents_ConnectedComponent::ConnectedComponents_ConnectedComponent(const ConnectedComponents_ConnectedComponent& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      trajectory_id_(from.trajectory_id_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:cartographer.mapping.proto.ConnectedComponents.ConnectedComponent)
}

void ConnectedComponents_ConnectedComponent::SharedCtor() {
}

ConnectedComponents_ConnectedComponent::~ConnectedComponents_ConnectedComponent() {
  // @@protoc_insertion_point(destructor:cartographer.mapping.proto.ConnectedComponents.ConnectedComponent)
  SharedDtor();
}

void ConnectedComponents_ConnectedComponent::SharedDtor() {
}

void ConnectedComponents_ConnectedComponent::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ConnectedComponents_ConnectedComponent& ConnectedComponents_ConnectedComponent::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_ConnectedComponents_ConnectedComponent_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto.base);
  return *internal_default_instance();
}


void ConnectedComponents_ConnectedComponent::Clear() {
// @@protoc_insertion_point(message_clear_start:cartographer.mapping.proto.ConnectedComponents.ConnectedComponent)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  trajectory_id_.Clear();
  _internal_metadata_.Clear();
}

const char* ConnectedComponents_ConnectedComponent::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // repeated int32 trajectory_id = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 10)) {
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::PackedInt32Parser(_internal_mutable_trajectory_id(), ptr, ctx);
          CHK_(ptr);
        } else if (static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 8) {
          _internal_add_trajectory_id(::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr));
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

::PROTOBUF_NAMESPACE_ID::uint8* ConnectedComponents_ConnectedComponent::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:cartographer.mapping.proto.ConnectedComponents.ConnectedComponent)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated int32 trajectory_id = 1;
  {
    int byte_size = _trajectory_id_cached_byte_size_.load(std::memory_order_relaxed);
    if (byte_size > 0) {
      target = stream->WriteInt32Packed(
          1, _internal_trajectory_id(), byte_size, target);
    }
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:cartographer.mapping.proto.ConnectedComponents.ConnectedComponent)
  return target;
}

size_t ConnectedComponents_ConnectedComponent::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:cartographer.mapping.proto.ConnectedComponents.ConnectedComponent)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated int32 trajectory_id = 1;
  {
    size_t data_size = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      Int32Size(this->trajectory_id_);
    if (data_size > 0) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::Int32Size(
            static_cast<::PROTOBUF_NAMESPACE_ID::int32>(data_size));
    }
    int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(data_size);
    _trajectory_id_cached_byte_size_.store(cached_size,
                                    std::memory_order_relaxed);
    total_size += data_size;
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    return ::PROTOBUF_NAMESPACE_ID::internal::ComputeUnknownFieldsSize(
        _internal_metadata_, total_size, &_cached_size_);
  }
  int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void ConnectedComponents_ConnectedComponent::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:cartographer.mapping.proto.ConnectedComponents.ConnectedComponent)
  GOOGLE_DCHECK_NE(&from, this);
  const ConnectedComponents_ConnectedComponent* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<ConnectedComponents_ConnectedComponent>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:cartographer.mapping.proto.ConnectedComponents.ConnectedComponent)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:cartographer.mapping.proto.ConnectedComponents.ConnectedComponent)
    MergeFrom(*source);
  }
}

void ConnectedComponents_ConnectedComponent::MergeFrom(const ConnectedComponents_ConnectedComponent& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:cartographer.mapping.proto.ConnectedComponents.ConnectedComponent)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  trajectory_id_.MergeFrom(from.trajectory_id_);
}

void ConnectedComponents_ConnectedComponent::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:cartographer.mapping.proto.ConnectedComponents.ConnectedComponent)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void ConnectedComponents_ConnectedComponent::CopyFrom(const ConnectedComponents_ConnectedComponent& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:cartographer.mapping.proto.ConnectedComponents.ConnectedComponent)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool ConnectedComponents_ConnectedComponent::IsInitialized() const {
  return true;
}

void ConnectedComponents_ConnectedComponent::InternalSwap(ConnectedComponents_ConnectedComponent* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  trajectory_id_.InternalSwap(&other->trajectory_id_);
}

::PROTOBUF_NAMESPACE_ID::Metadata ConnectedComponents_ConnectedComponent::GetMetadata() const {
  return GetMetadataStatic();
}


// ===================================================================

void ConnectedComponents::InitAsDefaultInstance() {
}
class ConnectedComponents::_Internal {
 public:
};

ConnectedComponents::ConnectedComponents()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:cartographer.mapping.proto.ConnectedComponents)
}
ConnectedComponents::ConnectedComponents(const ConnectedComponents& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      connected_component_(from.connected_component_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:cartographer.mapping.proto.ConnectedComponents)
}

void ConnectedComponents::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_ConnectedComponents_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto.base);
}

ConnectedComponents::~ConnectedComponents() {
  // @@protoc_insertion_point(destructor:cartographer.mapping.proto.ConnectedComponents)
  SharedDtor();
}

void ConnectedComponents::SharedDtor() {
}

void ConnectedComponents::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ConnectedComponents& ConnectedComponents::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_ConnectedComponents_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto.base);
  return *internal_default_instance();
}


void ConnectedComponents::Clear() {
// @@protoc_insertion_point(message_clear_start:cartographer.mapping.proto.ConnectedComponents)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  connected_component_.Clear();
  _internal_metadata_.Clear();
}

const char* ConnectedComponents::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // repeated .cartographer.mapping.proto.ConnectedComponents.ConnectedComponent connected_component = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 10)) {
          ptr -= 1;
          do {
            ptr += 1;
            ptr = ctx->ParseMessage(_internal_add_connected_component(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<10>(ptr));
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

::PROTOBUF_NAMESPACE_ID::uint8* ConnectedComponents::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:cartographer.mapping.proto.ConnectedComponents)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .cartographer.mapping.proto.ConnectedComponents.ConnectedComponent connected_component = 1;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_connected_component_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(1, this->_internal_connected_component(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:cartographer.mapping.proto.ConnectedComponents)
  return target;
}

size_t ConnectedComponents::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:cartographer.mapping.proto.ConnectedComponents)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .cartographer.mapping.proto.ConnectedComponents.ConnectedComponent connected_component = 1;
  total_size += 1UL * this->_internal_connected_component_size();
  for (const auto& msg : this->connected_component_) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(msg);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    return ::PROTOBUF_NAMESPACE_ID::internal::ComputeUnknownFieldsSize(
        _internal_metadata_, total_size, &_cached_size_);
  }
  int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void ConnectedComponents::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:cartographer.mapping.proto.ConnectedComponents)
  GOOGLE_DCHECK_NE(&from, this);
  const ConnectedComponents* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<ConnectedComponents>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:cartographer.mapping.proto.ConnectedComponents)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:cartographer.mapping.proto.ConnectedComponents)
    MergeFrom(*source);
  }
}

void ConnectedComponents::MergeFrom(const ConnectedComponents& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:cartographer.mapping.proto.ConnectedComponents)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  connected_component_.MergeFrom(from.connected_component_);
}

void ConnectedComponents::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:cartographer.mapping.proto.ConnectedComponents)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void ConnectedComponents::CopyFrom(const ConnectedComponents& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:cartographer.mapping.proto.ConnectedComponents)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool ConnectedComponents::IsInitialized() const {
  return true;
}

void ConnectedComponents::InternalSwap(ConnectedComponents* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  connected_component_.InternalSwap(&other->connected_component_);
}

::PROTOBUF_NAMESPACE_ID::Metadata ConnectedComponents::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace proto
}  // namespace mapping
}  // namespace cartographer
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::cartographer::mapping::proto::ConnectedComponents_ConnectedComponent* Arena::CreateMaybeMessage< ::cartographer::mapping::proto::ConnectedComponents_ConnectedComponent >(Arena* arena) {
  return Arena::CreateInternal< ::cartographer::mapping::proto::ConnectedComponents_ConnectedComponent >(arena);
}
template<> PROTOBUF_NOINLINE ::cartographer::mapping::proto::ConnectedComponents* Arena::CreateMaybeMessage< ::cartographer::mapping::proto::ConnectedComponents >(Arena* arena) {
  return Arena::CreateInternal< ::cartographer::mapping::proto::ConnectedComponents >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>