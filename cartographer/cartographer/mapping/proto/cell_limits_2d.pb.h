// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: cartographer/mapping/proto/cell_limits_2d.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_cartographer_2fmapping_2fproto_2fcell_5flimits_5f2d_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_cartographer_2fmapping_2fproto_2fcell_5flimits_5f2d_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_cartographer_2fmapping_2fproto_2fcell_5flimits_5f2d_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_cartographer_2fmapping_2fproto_2fcell_5flimits_5f2d_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_cartographer_2fmapping_2fproto_2fcell_5flimits_5f2d_2eproto;
namespace cartographer {
namespace mapping {
namespace proto {
class CellLimits;
class CellLimitsDefaultTypeInternal;
extern CellLimitsDefaultTypeInternal _CellLimits_default_instance_;
}  // namespace proto
}  // namespace mapping
}  // namespace cartographer
PROTOBUF_NAMESPACE_OPEN
template<> ::cartographer::mapping::proto::CellLimits* Arena::CreateMaybeMessage<::cartographer::mapping::proto::CellLimits>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace cartographer {
namespace mapping {
namespace proto {

// ===================================================================

class CellLimits :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:cartographer.mapping.proto.CellLimits) */ {
 public:
  CellLimits();
  virtual ~CellLimits();

  CellLimits(const CellLimits& from);
  CellLimits(CellLimits&& from) noexcept
    : CellLimits() {
    *this = ::std::move(from);
  }

  inline CellLimits& operator=(const CellLimits& from) {
    CopyFrom(from);
    return *this;
  }
  inline CellLimits& operator=(CellLimits&& from) noexcept {
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
  static const CellLimits& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const CellLimits* internal_default_instance() {
    return reinterpret_cast<const CellLimits*>(
               &_CellLimits_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(CellLimits& a, CellLimits& b) {
    a.Swap(&b);
  }
  inline void Swap(CellLimits* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline CellLimits* New() const final {
    return CreateMaybeMessage<CellLimits>(nullptr);
  }

  CellLimits* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<CellLimits>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const CellLimits& from);
  void MergeFrom(const CellLimits& from);
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
  void InternalSwap(CellLimits* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "cartographer.mapping.proto.CellLimits";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_cartographer_2fmapping_2fproto_2fcell_5flimits_5f2d_2eproto);
    return ::descriptor_table_cartographer_2fmapping_2fproto_2fcell_5flimits_5f2d_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kNumXCellsFieldNumber = 1,
    kNumYCellsFieldNumber = 2,
  };
  // int32 num_x_cells = 1;
  void clear_num_x_cells();
  ::PROTOBUF_NAMESPACE_ID::int32 num_x_cells() const;
  void set_num_x_cells(::PROTOBUF_NAMESPACE_ID::int32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::int32 _internal_num_x_cells() const;
  void _internal_set_num_x_cells(::PROTOBUF_NAMESPACE_ID::int32 value);
  public:

  // int32 num_y_cells = 2;
  void clear_num_y_cells();
  ::PROTOBUF_NAMESPACE_ID::int32 num_y_cells() const;
  void set_num_y_cells(::PROTOBUF_NAMESPACE_ID::int32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::int32 _internal_num_y_cells() const;
  void _internal_set_num_y_cells(::PROTOBUF_NAMESPACE_ID::int32 value);
  public:

  // @@protoc_insertion_point(class_scope:cartographer.mapping.proto.CellLimits)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::int32 num_x_cells_;
  ::PROTOBUF_NAMESPACE_ID::int32 num_y_cells_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_cartographer_2fmapping_2fproto_2fcell_5flimits_5f2d_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// CellLimits

// int32 num_x_cells = 1;
inline void CellLimits::clear_num_x_cells() {
  num_x_cells_ = 0;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 CellLimits::_internal_num_x_cells() const {
  return num_x_cells_;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 CellLimits::num_x_cells() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.proto.CellLimits.num_x_cells)
  return _internal_num_x_cells();
}
inline void CellLimits::_internal_set_num_x_cells(::PROTOBUF_NAMESPACE_ID::int32 value) {
  
  num_x_cells_ = value;
}
inline void CellLimits::set_num_x_cells(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _internal_set_num_x_cells(value);
  // @@protoc_insertion_point(field_set:cartographer.mapping.proto.CellLimits.num_x_cells)
}

// int32 num_y_cells = 2;
inline void CellLimits::clear_num_y_cells() {
  num_y_cells_ = 0;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 CellLimits::_internal_num_y_cells() const {
  return num_y_cells_;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 CellLimits::num_y_cells() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.proto.CellLimits.num_y_cells)
  return _internal_num_y_cells();
}
inline void CellLimits::_internal_set_num_y_cells(::PROTOBUF_NAMESPACE_ID::int32 value) {
  
  num_y_cells_ = value;
}
inline void CellLimits::set_num_y_cells(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _internal_set_num_y_cells(value);
  // @@protoc_insertion_point(field_set:cartographer.mapping.proto.CellLimits.num_y_cells)
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
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_cartographer_2fmapping_2fproto_2fcell_5flimits_5f2d_2eproto
