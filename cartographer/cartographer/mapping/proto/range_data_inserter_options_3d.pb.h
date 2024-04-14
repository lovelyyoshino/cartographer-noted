// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: cartographer/mapping/proto/range_data_inserter_options_3d.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_cartographer_2fmapping_2fproto_2frange_5fdata_5finserter_5foptions_5f3d_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_cartographer_2fmapping_2fproto_2frange_5fdata_5finserter_5foptions_5f3d_2eproto

#include <limits>
#include <string>

#include <google/protobuf/port_def.inc>
#if PROTOBUF_VERSION < 3011000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers. Please update
#error your headers.
#endif
#if 3011002 < PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers. Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/port_undef.inc>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/inlined_string_field.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_cartographer_2fmapping_2fproto_2frange_5fdata_5finserter_5foptions_5f3d_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_cartographer_2fmapping_2fproto_2frange_5fdata_5finserter_5foptions_5f3d_2eproto {
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTableField entries[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::AuxillaryParseTableField aux[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTable schema[1]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::FieldMetadata field_metadata[];
  static const ::PROTOBUF_NAMESPACE_ID::internal::SerializationTable serialization_table[];
  static const ::PROTOBUF_NAMESPACE_ID::uint32 offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_cartographer_2fmapping_2fproto_2frange_5fdata_5finserter_5foptions_5f3d_2eproto;
namespace cartographer {
namespace mapping {
namespace proto {
class RangeDataInserterOptions3D;
class RangeDataInserterOptions3DDefaultTypeInternal;
extern RangeDataInserterOptions3DDefaultTypeInternal _RangeDataInserterOptions3D_default_instance_;
}  // namespace proto
}  // namespace mapping
}  // namespace cartographer
PROTOBUF_NAMESPACE_OPEN
template<> ::cartographer::mapping::proto::RangeDataInserterOptions3D* Arena::CreateMaybeMessage<::cartographer::mapping::proto::RangeDataInserterOptions3D>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace cartographer {
namespace mapping {
namespace proto {

// ===================================================================

class RangeDataInserterOptions3D :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:cartographer.mapping.proto.RangeDataInserterOptions3D) */ {
 public:
  RangeDataInserterOptions3D();
  virtual ~RangeDataInserterOptions3D();

  RangeDataInserterOptions3D(const RangeDataInserterOptions3D& from);
  RangeDataInserterOptions3D(RangeDataInserterOptions3D&& from) noexcept
    : RangeDataInserterOptions3D() {
    *this = ::std::move(from);
  }

  inline RangeDataInserterOptions3D& operator=(const RangeDataInserterOptions3D& from) {
    CopyFrom(from);
    return *this;
  }
  inline RangeDataInserterOptions3D& operator=(RangeDataInserterOptions3D&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }

  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* descriptor() {
    return GetDescriptor();
  }
  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* GetDescriptor() {
    return GetMetadataStatic().descriptor;
  }
  static const ::PROTOBUF_NAMESPACE_ID::Reflection* GetReflection() {
    return GetMetadataStatic().reflection;
  }
  static const RangeDataInserterOptions3D& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const RangeDataInserterOptions3D* internal_default_instance() {
    return reinterpret_cast<const RangeDataInserterOptions3D*>(
               &_RangeDataInserterOptions3D_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(RangeDataInserterOptions3D& a, RangeDataInserterOptions3D& b) {
    a.Swap(&b);
  }
  inline void Swap(RangeDataInserterOptions3D* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline RangeDataInserterOptions3D* New() const final {
    return CreateMaybeMessage<RangeDataInserterOptions3D>(nullptr);
  }

  RangeDataInserterOptions3D* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<RangeDataInserterOptions3D>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const RangeDataInserterOptions3D& from);
  void MergeFrom(const RangeDataInserterOptions3D& from);
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  const char* _InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) final;
  ::PROTOBUF_NAMESPACE_ID::uint8* _InternalSerialize(
      ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  inline void SharedCtor();
  inline void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(RangeDataInserterOptions3D* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "cartographer.mapping.proto.RangeDataInserterOptions3D";
  }
  private:
  inline ::PROTOBUF_NAMESPACE_ID::Arena* GetArenaNoVirtual() const {
    return nullptr;
  }
  inline void* MaybeArenaPtr() const {
    return nullptr;
  }
  public:

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;
  private:
  static ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadataStatic() {
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_cartographer_2fmapping_2fproto_2frange_5fdata_5finserter_5foptions_5f3d_2eproto);
    return ::descriptor_table_cartographer_2fmapping_2fproto_2frange_5fdata_5finserter_5foptions_5f3d_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kHitProbabilityFieldNumber = 1,
    kMissProbabilityFieldNumber = 2,
    kNumFreeSpaceVoxelsFieldNumber = 3,
  };
  // double hit_probability = 1;
  void clear_hit_probability();
  double hit_probability() const;
  void set_hit_probability(double value);
  private:
  double _internal_hit_probability() const;
  void _internal_set_hit_probability(double value);
  public:

  // double miss_probability = 2;
  void clear_miss_probability();
  double miss_probability() const;
  void set_miss_probability(double value);
  private:
  double _internal_miss_probability() const;
  void _internal_set_miss_probability(double value);
  public:

  // int32 num_free_space_voxels = 3;
  void clear_num_free_space_voxels();
  ::PROTOBUF_NAMESPACE_ID::int32 num_free_space_voxels() const;
  void set_num_free_space_voxels(::PROTOBUF_NAMESPACE_ID::int32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::int32 _internal_num_free_space_voxels() const;
  void _internal_set_num_free_space_voxels(::PROTOBUF_NAMESPACE_ID::int32 value);
  public:

  // @@protoc_insertion_point(class_scope:cartographer.mapping.proto.RangeDataInserterOptions3D)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  double hit_probability_;
  double miss_probability_;
  ::PROTOBUF_NAMESPACE_ID::int32 num_free_space_voxels_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_cartographer_2fmapping_2fproto_2frange_5fdata_5finserter_5foptions_5f3d_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// RangeDataInserterOptions3D

// double hit_probability = 1;
inline void RangeDataInserterOptions3D::clear_hit_probability() {
  hit_probability_ = 0;
}
inline double RangeDataInserterOptions3D::_internal_hit_probability() const {
  return hit_probability_;
}
inline double RangeDataInserterOptions3D::hit_probability() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.proto.RangeDataInserterOptions3D.hit_probability)
  return _internal_hit_probability();
}
inline void RangeDataInserterOptions3D::_internal_set_hit_probability(double value) {
  
  hit_probability_ = value;
}
inline void RangeDataInserterOptions3D::set_hit_probability(double value) {
  _internal_set_hit_probability(value);
  // @@protoc_insertion_point(field_set:cartographer.mapping.proto.RangeDataInserterOptions3D.hit_probability)
}

// double miss_probability = 2;
inline void RangeDataInserterOptions3D::clear_miss_probability() {
  miss_probability_ = 0;
}
inline double RangeDataInserterOptions3D::_internal_miss_probability() const {
  return miss_probability_;
}
inline double RangeDataInserterOptions3D::miss_probability() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.proto.RangeDataInserterOptions3D.miss_probability)
  return _internal_miss_probability();
}
inline void RangeDataInserterOptions3D::_internal_set_miss_probability(double value) {
  
  miss_probability_ = value;
}
inline void RangeDataInserterOptions3D::set_miss_probability(double value) {
  _internal_set_miss_probability(value);
  // @@protoc_insertion_point(field_set:cartographer.mapping.proto.RangeDataInserterOptions3D.miss_probability)
}

// int32 num_free_space_voxels = 3;
inline void RangeDataInserterOptions3D::clear_num_free_space_voxels() {
  num_free_space_voxels_ = 0;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 RangeDataInserterOptions3D::_internal_num_free_space_voxels() const {
  return num_free_space_voxels_;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 RangeDataInserterOptions3D::num_free_space_voxels() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.proto.RangeDataInserterOptions3D.num_free_space_voxels)
  return _internal_num_free_space_voxels();
}
inline void RangeDataInserterOptions3D::_internal_set_num_free_space_voxels(::PROTOBUF_NAMESPACE_ID::int32 value) {
  
  num_free_space_voxels_ = value;
}
inline void RangeDataInserterOptions3D::set_num_free_space_voxels(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _internal_set_num_free_space_voxels(value);
  // @@protoc_insertion_point(field_set:cartographer.mapping.proto.RangeDataInserterOptions3D.num_free_space_voxels)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace proto
}  // namespace mapping
}  // namespace cartographer

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_cartographer_2fmapping_2fproto_2frange_5fdata_5finserter_5foptions_5f3d_2eproto
