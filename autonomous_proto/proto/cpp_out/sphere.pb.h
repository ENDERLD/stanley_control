// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: sphere.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_sphere_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_sphere_2eproto

#include <limits>
#include <string>

#include <google/protobuf/port_def.inc>
#if PROTOBUF_VERSION < 3021000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers. Please update
#error your headers.
#endif
#if 3021005 < PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers. Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/port_undef.inc>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata_lite.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
#include <google/protobuf/wrappers.pb.h>
#include "message_info.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_sphere_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_sphere_2eproto {
  static const uint32_t offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_sphere_2eproto;
namespace autonomous_proto {
class Sphere;
struct SphereDefaultTypeInternal;
extern SphereDefaultTypeInternal _Sphere_default_instance_;
class Sphere_Raw;
struct Sphere_RawDefaultTypeInternal;
extern Sphere_RawDefaultTypeInternal _Sphere_Raw_default_instance_;
}  // namespace autonomous_proto
PROTOBUF_NAMESPACE_OPEN
template<> ::autonomous_proto::Sphere* Arena::CreateMaybeMessage<::autonomous_proto::Sphere>(Arena*);
template<> ::autonomous_proto::Sphere_Raw* Arena::CreateMaybeMessage<::autonomous_proto::Sphere_Raw>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace autonomous_proto {

// ===================================================================

class Sphere_Raw final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:autonomous_proto.Sphere.Raw) */ {
 public:
  inline Sphere_Raw() : Sphere_Raw(nullptr) {}
  ~Sphere_Raw() override;
  explicit PROTOBUF_CONSTEXPR Sphere_Raw(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  Sphere_Raw(const Sphere_Raw& from);
  Sphere_Raw(Sphere_Raw&& from) noexcept
    : Sphere_Raw() {
    *this = ::std::move(from);
  }

  inline Sphere_Raw& operator=(const Sphere_Raw& from) {
    CopyFrom(from);
    return *this;
  }
  inline Sphere_Raw& operator=(Sphere_Raw&& from) noexcept {
    if (this == &from) return *this;
    if (GetOwningArena() == from.GetOwningArena()
  #ifdef PROTOBUF_FORCE_COPY_IN_MOVE
        && GetOwningArena() != nullptr
  #endif  // !PROTOBUF_FORCE_COPY_IN_MOVE
    ) {
      InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }

  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* descriptor() {
    return GetDescriptor();
  }
  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* GetDescriptor() {
    return default_instance().GetMetadata().descriptor;
  }
  static const ::PROTOBUF_NAMESPACE_ID::Reflection* GetReflection() {
    return default_instance().GetMetadata().reflection;
  }
  static const Sphere_Raw& default_instance() {
    return *internal_default_instance();
  }
  static inline const Sphere_Raw* internal_default_instance() {
    return reinterpret_cast<const Sphere_Raw*>(
               &_Sphere_Raw_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(Sphere_Raw& a, Sphere_Raw& b) {
    a.Swap(&b);
  }
  inline void Swap(Sphere_Raw* other) {
    if (other == this) return;
  #ifdef PROTOBUF_FORCE_COPY_IN_SWAP
    if (GetOwningArena() != nullptr &&
        GetOwningArena() == other->GetOwningArena()) {
   #else  // PROTOBUF_FORCE_COPY_IN_SWAP
    if (GetOwningArena() == other->GetOwningArena()) {
  #endif  // !PROTOBUF_FORCE_COPY_IN_SWAP
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(Sphere_Raw* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  Sphere_Raw* New(::PROTOBUF_NAMESPACE_ID::Arena* arena = nullptr) const final {
    return CreateMaybeMessage<Sphere_Raw>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const Sphere_Raw& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom( const Sphere_Raw& from) {
    Sphere_Raw::MergeImpl(*this, from);
  }
  private:
  static void MergeImpl(::PROTOBUF_NAMESPACE_ID::Message& to_msg, const ::PROTOBUF_NAMESPACE_ID::Message& from_msg);
  public:
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  const char* _InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) final;
  uint8_t* _InternalSerialize(
      uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const final;
  int GetCachedSize() const final { return _impl_._cached_size_.Get(); }

  private:
  void SharedCtor(::PROTOBUF_NAMESPACE_ID::Arena* arena, bool is_message_owned);
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(Sphere_Raw* other);

  private:
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "autonomous_proto.Sphere.Raw";
  }
  protected:
  explicit Sphere_Raw(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                       bool is_message_owned = false);
  public:

  static const ClassData _class_data_;
  const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*GetClassData() const final;

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kInt64ArrayFieldNumber = 4,
    kFloat64ArrayFieldNumber = 5,
    kNameFieldNumber = 1,
    kInt64ValueFieldNumber = 2,
    kFloat64ValueFieldNumber = 3,
  };
  // repeated int64 int64_array = 4;
  int int64_array_size() const;
  private:
  int _internal_int64_array_size() const;
  public:
  void clear_int64_array();
  private:
  int64_t _internal_int64_array(int index) const;
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< int64_t >&
      _internal_int64_array() const;
  void _internal_add_int64_array(int64_t value);
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< int64_t >*
      _internal_mutable_int64_array();
  public:
  int64_t int64_array(int index) const;
  void set_int64_array(int index, int64_t value);
  void add_int64_array(int64_t value);
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< int64_t >&
      int64_array() const;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< int64_t >*
      mutable_int64_array();

  // repeated double float64_array = 5;
  int float64_array_size() const;
  private:
  int _internal_float64_array_size() const;
  public:
  void clear_float64_array();
  private:
  double _internal_float64_array(int index) const;
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
      _internal_float64_array() const;
  void _internal_add_float64_array(double value);
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
      _internal_mutable_float64_array();
  public:
  double float64_array(int index) const;
  void set_float64_array(int index, double value);
  void add_float64_array(double value);
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
      float64_array() const;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
      mutable_float64_array();

  // .google.protobuf.StringValue name = 1;
  bool has_name() const;
  private:
  bool _internal_has_name() const;
  public:
  void clear_name();
  const ::PROTOBUF_NAMESPACE_ID::StringValue& name() const;
  PROTOBUF_NODISCARD ::PROTOBUF_NAMESPACE_ID::StringValue* release_name();
  ::PROTOBUF_NAMESPACE_ID::StringValue* mutable_name();
  void set_allocated_name(::PROTOBUF_NAMESPACE_ID::StringValue* name);
  private:
  const ::PROTOBUF_NAMESPACE_ID::StringValue& _internal_name() const;
  ::PROTOBUF_NAMESPACE_ID::StringValue* _internal_mutable_name();
  public:
  void unsafe_arena_set_allocated_name(
      ::PROTOBUF_NAMESPACE_ID::StringValue* name);
  ::PROTOBUF_NAMESPACE_ID::StringValue* unsafe_arena_release_name();

  // .google.protobuf.Int64Value int64_value = 2;
  bool has_int64_value() const;
  private:
  bool _internal_has_int64_value() const;
  public:
  void clear_int64_value();
  const ::PROTOBUF_NAMESPACE_ID::Int64Value& int64_value() const;
  PROTOBUF_NODISCARD ::PROTOBUF_NAMESPACE_ID::Int64Value* release_int64_value();
  ::PROTOBUF_NAMESPACE_ID::Int64Value* mutable_int64_value();
  void set_allocated_int64_value(::PROTOBUF_NAMESPACE_ID::Int64Value* int64_value);
  private:
  const ::PROTOBUF_NAMESPACE_ID::Int64Value& _internal_int64_value() const;
  ::PROTOBUF_NAMESPACE_ID::Int64Value* _internal_mutable_int64_value();
  public:
  void unsafe_arena_set_allocated_int64_value(
      ::PROTOBUF_NAMESPACE_ID::Int64Value* int64_value);
  ::PROTOBUF_NAMESPACE_ID::Int64Value* unsafe_arena_release_int64_value();

  // .google.protobuf.DoubleValue float64_value = 3;
  bool has_float64_value() const;
  private:
  bool _internal_has_float64_value() const;
  public:
  void clear_float64_value();
  const ::PROTOBUF_NAMESPACE_ID::DoubleValue& float64_value() const;
  PROTOBUF_NODISCARD ::PROTOBUF_NAMESPACE_ID::DoubleValue* release_float64_value();
  ::PROTOBUF_NAMESPACE_ID::DoubleValue* mutable_float64_value();
  void set_allocated_float64_value(::PROTOBUF_NAMESPACE_ID::DoubleValue* float64_value);
  private:
  const ::PROTOBUF_NAMESPACE_ID::DoubleValue& _internal_float64_value() const;
  ::PROTOBUF_NAMESPACE_ID::DoubleValue* _internal_mutable_float64_value();
  public:
  void unsafe_arena_set_allocated_float64_value(
      ::PROTOBUF_NAMESPACE_ID::DoubleValue* float64_value);
  ::PROTOBUF_NAMESPACE_ID::DoubleValue* unsafe_arena_release_float64_value();

  // @@protoc_insertion_point(class_scope:autonomous_proto.Sphere.Raw)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  struct Impl_ {
    ::PROTOBUF_NAMESPACE_ID::RepeatedField< int64_t > int64_array_;
    mutable std::atomic<int> _int64_array_cached_byte_size_;
    ::PROTOBUF_NAMESPACE_ID::RepeatedField< double > float64_array_;
    ::PROTOBUF_NAMESPACE_ID::StringValue* name_;
    ::PROTOBUF_NAMESPACE_ID::Int64Value* int64_value_;
    ::PROTOBUF_NAMESPACE_ID::DoubleValue* float64_value_;
    mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  };
  union { Impl_ _impl_; };
  friend struct ::TableStruct_sphere_2eproto;
};
// -------------------------------------------------------------------

class Sphere final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:autonomous_proto.Sphere) */ {
 public:
  inline Sphere() : Sphere(nullptr) {}
  ~Sphere() override;
  explicit PROTOBUF_CONSTEXPR Sphere(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  Sphere(const Sphere& from);
  Sphere(Sphere&& from) noexcept
    : Sphere() {
    *this = ::std::move(from);
  }

  inline Sphere& operator=(const Sphere& from) {
    CopyFrom(from);
    return *this;
  }
  inline Sphere& operator=(Sphere&& from) noexcept {
    if (this == &from) return *this;
    if (GetOwningArena() == from.GetOwningArena()
  #ifdef PROTOBUF_FORCE_COPY_IN_MOVE
        && GetOwningArena() != nullptr
  #endif  // !PROTOBUF_FORCE_COPY_IN_MOVE
    ) {
      InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }

  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* descriptor() {
    return GetDescriptor();
  }
  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* GetDescriptor() {
    return default_instance().GetMetadata().descriptor;
  }
  static const ::PROTOBUF_NAMESPACE_ID::Reflection* GetReflection() {
    return default_instance().GetMetadata().reflection;
  }
  static const Sphere& default_instance() {
    return *internal_default_instance();
  }
  static inline const Sphere* internal_default_instance() {
    return reinterpret_cast<const Sphere*>(
               &_Sphere_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(Sphere& a, Sphere& b) {
    a.Swap(&b);
  }
  inline void Swap(Sphere* other) {
    if (other == this) return;
  #ifdef PROTOBUF_FORCE_COPY_IN_SWAP
    if (GetOwningArena() != nullptr &&
        GetOwningArena() == other->GetOwningArena()) {
   #else  // PROTOBUF_FORCE_COPY_IN_SWAP
    if (GetOwningArena() == other->GetOwningArena()) {
  #endif  // !PROTOBUF_FORCE_COPY_IN_SWAP
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(Sphere* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  Sphere* New(::PROTOBUF_NAMESPACE_ID::Arena* arena = nullptr) const final {
    return CreateMaybeMessage<Sphere>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const Sphere& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom( const Sphere& from) {
    Sphere::MergeImpl(*this, from);
  }
  private:
  static void MergeImpl(::PROTOBUF_NAMESPACE_ID::Message& to_msg, const ::PROTOBUF_NAMESPACE_ID::Message& from_msg);
  public:
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  const char* _InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) final;
  uint8_t* _InternalSerialize(
      uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const final;
  int GetCachedSize() const final { return _impl_._cached_size_.Get(); }

  private:
  void SharedCtor(::PROTOBUF_NAMESPACE_ID::Arena* arena, bool is_message_owned);
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(Sphere* other);

  private:
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "autonomous_proto.Sphere";
  }
  protected:
  explicit Sphere(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                       bool is_message_owned = false);
  public:

  static const ClassData _class_data_;
  const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*GetClassData() const final;

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  typedef Sphere_Raw Raw;

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 2,
    kHeaderFieldNumber = 1,
  };
  // repeated .autonomous_proto.Sphere.Raw data = 2;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::autonomous_proto::Sphere_Raw* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::autonomous_proto::Sphere_Raw >*
      mutable_data();
  private:
  const ::autonomous_proto::Sphere_Raw& _internal_data(int index) const;
  ::autonomous_proto::Sphere_Raw* _internal_add_data();
  public:
  const ::autonomous_proto::Sphere_Raw& data(int index) const;
  ::autonomous_proto::Sphere_Raw* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::autonomous_proto::Sphere_Raw >&
      data() const;

  // .autonomous_proto.MessageInfo header = 1;
  bool has_header() const;
  private:
  bool _internal_has_header() const;
  public:
  void clear_header();
  const ::autonomous_proto::MessageInfo& header() const;
  PROTOBUF_NODISCARD ::autonomous_proto::MessageInfo* release_header();
  ::autonomous_proto::MessageInfo* mutable_header();
  void set_allocated_header(::autonomous_proto::MessageInfo* header);
  private:
  const ::autonomous_proto::MessageInfo& _internal_header() const;
  ::autonomous_proto::MessageInfo* _internal_mutable_header();
  public:
  void unsafe_arena_set_allocated_header(
      ::autonomous_proto::MessageInfo* header);
  ::autonomous_proto::MessageInfo* unsafe_arena_release_header();

  // @@protoc_insertion_point(class_scope:autonomous_proto.Sphere)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  struct Impl_ {
    ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::autonomous_proto::Sphere_Raw > data_;
    ::autonomous_proto::MessageInfo* header_;
    mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  };
  union { Impl_ _impl_; };
  friend struct ::TableStruct_sphere_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// Sphere_Raw

// .google.protobuf.StringValue name = 1;
inline bool Sphere_Raw::_internal_has_name() const {
  return this != internal_default_instance() && _impl_.name_ != nullptr;
}
inline bool Sphere_Raw::has_name() const {
  return _internal_has_name();
}
inline const ::PROTOBUF_NAMESPACE_ID::StringValue& Sphere_Raw::_internal_name() const {
  const ::PROTOBUF_NAMESPACE_ID::StringValue* p = _impl_.name_;
  return p != nullptr ? *p : reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::StringValue&>(
      ::PROTOBUF_NAMESPACE_ID::_StringValue_default_instance_);
}
inline const ::PROTOBUF_NAMESPACE_ID::StringValue& Sphere_Raw::name() const {
  // @@protoc_insertion_point(field_get:autonomous_proto.Sphere.Raw.name)
  return _internal_name();
}
inline void Sphere_Raw::unsafe_arena_set_allocated_name(
    ::PROTOBUF_NAMESPACE_ID::StringValue* name) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(_impl_.name_);
  }
  _impl_.name_ = name;
  if (name) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:autonomous_proto.Sphere.Raw.name)
}
inline ::PROTOBUF_NAMESPACE_ID::StringValue* Sphere_Raw::release_name() {
  
  ::PROTOBUF_NAMESPACE_ID::StringValue* temp = _impl_.name_;
  _impl_.name_ = nullptr;
#ifdef PROTOBUF_FORCE_COPY_IN_RELEASE
  auto* old =  reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(temp);
  temp = ::PROTOBUF_NAMESPACE_ID::internal::DuplicateIfNonNull(temp);
  if (GetArenaForAllocation() == nullptr) { delete old; }
#else  // PROTOBUF_FORCE_COPY_IN_RELEASE
  if (GetArenaForAllocation() != nullptr) {
    temp = ::PROTOBUF_NAMESPACE_ID::internal::DuplicateIfNonNull(temp);
  }
#endif  // !PROTOBUF_FORCE_COPY_IN_RELEASE
  return temp;
}
inline ::PROTOBUF_NAMESPACE_ID::StringValue* Sphere_Raw::unsafe_arena_release_name() {
  // @@protoc_insertion_point(field_release:autonomous_proto.Sphere.Raw.name)
  
  ::PROTOBUF_NAMESPACE_ID::StringValue* temp = _impl_.name_;
  _impl_.name_ = nullptr;
  return temp;
}
inline ::PROTOBUF_NAMESPACE_ID::StringValue* Sphere_Raw::_internal_mutable_name() {
  
  if (_impl_.name_ == nullptr) {
    auto* p = CreateMaybeMessage<::PROTOBUF_NAMESPACE_ID::StringValue>(GetArenaForAllocation());
    _impl_.name_ = p;
  }
  return _impl_.name_;
}
inline ::PROTOBUF_NAMESPACE_ID::StringValue* Sphere_Raw::mutable_name() {
  ::PROTOBUF_NAMESPACE_ID::StringValue* _msg = _internal_mutable_name();
  // @@protoc_insertion_point(field_mutable:autonomous_proto.Sphere.Raw.name)
  return _msg;
}
inline void Sphere_Raw::set_allocated_name(::PROTOBUF_NAMESPACE_ID::StringValue* name) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaForAllocation();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(_impl_.name_);
  }
  if (name) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
        ::PROTOBUF_NAMESPACE_ID::Arena::InternalGetOwningArena(
                reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(name));
    if (message_arena != submessage_arena) {
      name = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, name, submessage_arena);
    }
    
  } else {
    
  }
  _impl_.name_ = name;
  // @@protoc_insertion_point(field_set_allocated:autonomous_proto.Sphere.Raw.name)
}

// .google.protobuf.Int64Value int64_value = 2;
inline bool Sphere_Raw::_internal_has_int64_value() const {
  return this != internal_default_instance() && _impl_.int64_value_ != nullptr;
}
inline bool Sphere_Raw::has_int64_value() const {
  return _internal_has_int64_value();
}
inline const ::PROTOBUF_NAMESPACE_ID::Int64Value& Sphere_Raw::_internal_int64_value() const {
  const ::PROTOBUF_NAMESPACE_ID::Int64Value* p = _impl_.int64_value_;
  return p != nullptr ? *p : reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Int64Value&>(
      ::PROTOBUF_NAMESPACE_ID::_Int64Value_default_instance_);
}
inline const ::PROTOBUF_NAMESPACE_ID::Int64Value& Sphere_Raw::int64_value() const {
  // @@protoc_insertion_point(field_get:autonomous_proto.Sphere.Raw.int64_value)
  return _internal_int64_value();
}
inline void Sphere_Raw::unsafe_arena_set_allocated_int64_value(
    ::PROTOBUF_NAMESPACE_ID::Int64Value* int64_value) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(_impl_.int64_value_);
  }
  _impl_.int64_value_ = int64_value;
  if (int64_value) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:autonomous_proto.Sphere.Raw.int64_value)
}
inline ::PROTOBUF_NAMESPACE_ID::Int64Value* Sphere_Raw::release_int64_value() {
  
  ::PROTOBUF_NAMESPACE_ID::Int64Value* temp = _impl_.int64_value_;
  _impl_.int64_value_ = nullptr;
#ifdef PROTOBUF_FORCE_COPY_IN_RELEASE
  auto* old =  reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(temp);
  temp = ::PROTOBUF_NAMESPACE_ID::internal::DuplicateIfNonNull(temp);
  if (GetArenaForAllocation() == nullptr) { delete old; }
#else  // PROTOBUF_FORCE_COPY_IN_RELEASE
  if (GetArenaForAllocation() != nullptr) {
    temp = ::PROTOBUF_NAMESPACE_ID::internal::DuplicateIfNonNull(temp);
  }
#endif  // !PROTOBUF_FORCE_COPY_IN_RELEASE
  return temp;
}
inline ::PROTOBUF_NAMESPACE_ID::Int64Value* Sphere_Raw::unsafe_arena_release_int64_value() {
  // @@protoc_insertion_point(field_release:autonomous_proto.Sphere.Raw.int64_value)
  
  ::PROTOBUF_NAMESPACE_ID::Int64Value* temp = _impl_.int64_value_;
  _impl_.int64_value_ = nullptr;
  return temp;
}
inline ::PROTOBUF_NAMESPACE_ID::Int64Value* Sphere_Raw::_internal_mutable_int64_value() {
  
  if (_impl_.int64_value_ == nullptr) {
    auto* p = CreateMaybeMessage<::PROTOBUF_NAMESPACE_ID::Int64Value>(GetArenaForAllocation());
    _impl_.int64_value_ = p;
  }
  return _impl_.int64_value_;
}
inline ::PROTOBUF_NAMESPACE_ID::Int64Value* Sphere_Raw::mutable_int64_value() {
  ::PROTOBUF_NAMESPACE_ID::Int64Value* _msg = _internal_mutable_int64_value();
  // @@protoc_insertion_point(field_mutable:autonomous_proto.Sphere.Raw.int64_value)
  return _msg;
}
inline void Sphere_Raw::set_allocated_int64_value(::PROTOBUF_NAMESPACE_ID::Int64Value* int64_value) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaForAllocation();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(_impl_.int64_value_);
  }
  if (int64_value) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
        ::PROTOBUF_NAMESPACE_ID::Arena::InternalGetOwningArena(
                reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(int64_value));
    if (message_arena != submessage_arena) {
      int64_value = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, int64_value, submessage_arena);
    }
    
  } else {
    
  }
  _impl_.int64_value_ = int64_value;
  // @@protoc_insertion_point(field_set_allocated:autonomous_proto.Sphere.Raw.int64_value)
}

// .google.protobuf.DoubleValue float64_value = 3;
inline bool Sphere_Raw::_internal_has_float64_value() const {
  return this != internal_default_instance() && _impl_.float64_value_ != nullptr;
}
inline bool Sphere_Raw::has_float64_value() const {
  return _internal_has_float64_value();
}
inline const ::PROTOBUF_NAMESPACE_ID::DoubleValue& Sphere_Raw::_internal_float64_value() const {
  const ::PROTOBUF_NAMESPACE_ID::DoubleValue* p = _impl_.float64_value_;
  return p != nullptr ? *p : reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::DoubleValue&>(
      ::PROTOBUF_NAMESPACE_ID::_DoubleValue_default_instance_);
}
inline const ::PROTOBUF_NAMESPACE_ID::DoubleValue& Sphere_Raw::float64_value() const {
  // @@protoc_insertion_point(field_get:autonomous_proto.Sphere.Raw.float64_value)
  return _internal_float64_value();
}
inline void Sphere_Raw::unsafe_arena_set_allocated_float64_value(
    ::PROTOBUF_NAMESPACE_ID::DoubleValue* float64_value) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(_impl_.float64_value_);
  }
  _impl_.float64_value_ = float64_value;
  if (float64_value) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:autonomous_proto.Sphere.Raw.float64_value)
}
inline ::PROTOBUF_NAMESPACE_ID::DoubleValue* Sphere_Raw::release_float64_value() {
  
  ::PROTOBUF_NAMESPACE_ID::DoubleValue* temp = _impl_.float64_value_;
  _impl_.float64_value_ = nullptr;
#ifdef PROTOBUF_FORCE_COPY_IN_RELEASE
  auto* old =  reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(temp);
  temp = ::PROTOBUF_NAMESPACE_ID::internal::DuplicateIfNonNull(temp);
  if (GetArenaForAllocation() == nullptr) { delete old; }
#else  // PROTOBUF_FORCE_COPY_IN_RELEASE
  if (GetArenaForAllocation() != nullptr) {
    temp = ::PROTOBUF_NAMESPACE_ID::internal::DuplicateIfNonNull(temp);
  }
#endif  // !PROTOBUF_FORCE_COPY_IN_RELEASE
  return temp;
}
inline ::PROTOBUF_NAMESPACE_ID::DoubleValue* Sphere_Raw::unsafe_arena_release_float64_value() {
  // @@protoc_insertion_point(field_release:autonomous_proto.Sphere.Raw.float64_value)
  
  ::PROTOBUF_NAMESPACE_ID::DoubleValue* temp = _impl_.float64_value_;
  _impl_.float64_value_ = nullptr;
  return temp;
}
inline ::PROTOBUF_NAMESPACE_ID::DoubleValue* Sphere_Raw::_internal_mutable_float64_value() {
  
  if (_impl_.float64_value_ == nullptr) {
    auto* p = CreateMaybeMessage<::PROTOBUF_NAMESPACE_ID::DoubleValue>(GetArenaForAllocation());
    _impl_.float64_value_ = p;
  }
  return _impl_.float64_value_;
}
inline ::PROTOBUF_NAMESPACE_ID::DoubleValue* Sphere_Raw::mutable_float64_value() {
  ::PROTOBUF_NAMESPACE_ID::DoubleValue* _msg = _internal_mutable_float64_value();
  // @@protoc_insertion_point(field_mutable:autonomous_proto.Sphere.Raw.float64_value)
  return _msg;
}
inline void Sphere_Raw::set_allocated_float64_value(::PROTOBUF_NAMESPACE_ID::DoubleValue* float64_value) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaForAllocation();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(_impl_.float64_value_);
  }
  if (float64_value) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
        ::PROTOBUF_NAMESPACE_ID::Arena::InternalGetOwningArena(
                reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(float64_value));
    if (message_arena != submessage_arena) {
      float64_value = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, float64_value, submessage_arena);
    }
    
  } else {
    
  }
  _impl_.float64_value_ = float64_value;
  // @@protoc_insertion_point(field_set_allocated:autonomous_proto.Sphere.Raw.float64_value)
}

// repeated int64 int64_array = 4;
inline int Sphere_Raw::_internal_int64_array_size() const {
  return _impl_.int64_array_.size();
}
inline int Sphere_Raw::int64_array_size() const {
  return _internal_int64_array_size();
}
inline void Sphere_Raw::clear_int64_array() {
  _impl_.int64_array_.Clear();
}
inline int64_t Sphere_Raw::_internal_int64_array(int index) const {
  return _impl_.int64_array_.Get(index);
}
inline int64_t Sphere_Raw::int64_array(int index) const {
  // @@protoc_insertion_point(field_get:autonomous_proto.Sphere.Raw.int64_array)
  return _internal_int64_array(index);
}
inline void Sphere_Raw::set_int64_array(int index, int64_t value) {
  _impl_.int64_array_.Set(index, value);
  // @@protoc_insertion_point(field_set:autonomous_proto.Sphere.Raw.int64_array)
}
inline void Sphere_Raw::_internal_add_int64_array(int64_t value) {
  _impl_.int64_array_.Add(value);
}
inline void Sphere_Raw::add_int64_array(int64_t value) {
  _internal_add_int64_array(value);
  // @@protoc_insertion_point(field_add:autonomous_proto.Sphere.Raw.int64_array)
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< int64_t >&
Sphere_Raw::_internal_int64_array() const {
  return _impl_.int64_array_;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< int64_t >&
Sphere_Raw::int64_array() const {
  // @@protoc_insertion_point(field_list:autonomous_proto.Sphere.Raw.int64_array)
  return _internal_int64_array();
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< int64_t >*
Sphere_Raw::_internal_mutable_int64_array() {
  return &_impl_.int64_array_;
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< int64_t >*
Sphere_Raw::mutable_int64_array() {
  // @@protoc_insertion_point(field_mutable_list:autonomous_proto.Sphere.Raw.int64_array)
  return _internal_mutable_int64_array();
}

// repeated double float64_array = 5;
inline int Sphere_Raw::_internal_float64_array_size() const {
  return _impl_.float64_array_.size();
}
inline int Sphere_Raw::float64_array_size() const {
  return _internal_float64_array_size();
}
inline void Sphere_Raw::clear_float64_array() {
  _impl_.float64_array_.Clear();
}
inline double Sphere_Raw::_internal_float64_array(int index) const {
  return _impl_.float64_array_.Get(index);
}
inline double Sphere_Raw::float64_array(int index) const {
  // @@protoc_insertion_point(field_get:autonomous_proto.Sphere.Raw.float64_array)
  return _internal_float64_array(index);
}
inline void Sphere_Raw::set_float64_array(int index, double value) {
  _impl_.float64_array_.Set(index, value);
  // @@protoc_insertion_point(field_set:autonomous_proto.Sphere.Raw.float64_array)
}
inline void Sphere_Raw::_internal_add_float64_array(double value) {
  _impl_.float64_array_.Add(value);
}
inline void Sphere_Raw::add_float64_array(double value) {
  _internal_add_float64_array(value);
  // @@protoc_insertion_point(field_add:autonomous_proto.Sphere.Raw.float64_array)
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
Sphere_Raw::_internal_float64_array() const {
  return _impl_.float64_array_;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
Sphere_Raw::float64_array() const {
  // @@protoc_insertion_point(field_list:autonomous_proto.Sphere.Raw.float64_array)
  return _internal_float64_array();
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
Sphere_Raw::_internal_mutable_float64_array() {
  return &_impl_.float64_array_;
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
Sphere_Raw::mutable_float64_array() {
  // @@protoc_insertion_point(field_mutable_list:autonomous_proto.Sphere.Raw.float64_array)
  return _internal_mutable_float64_array();
}

// -------------------------------------------------------------------

// Sphere

// .autonomous_proto.MessageInfo header = 1;
inline bool Sphere::_internal_has_header() const {
  return this != internal_default_instance() && _impl_.header_ != nullptr;
}
inline bool Sphere::has_header() const {
  return _internal_has_header();
}
inline const ::autonomous_proto::MessageInfo& Sphere::_internal_header() const {
  const ::autonomous_proto::MessageInfo* p = _impl_.header_;
  return p != nullptr ? *p : reinterpret_cast<const ::autonomous_proto::MessageInfo&>(
      ::autonomous_proto::_MessageInfo_default_instance_);
}
inline const ::autonomous_proto::MessageInfo& Sphere::header() const {
  // @@protoc_insertion_point(field_get:autonomous_proto.Sphere.header)
  return _internal_header();
}
inline void Sphere::unsafe_arena_set_allocated_header(
    ::autonomous_proto::MessageInfo* header) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(_impl_.header_);
  }
  _impl_.header_ = header;
  if (header) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:autonomous_proto.Sphere.header)
}
inline ::autonomous_proto::MessageInfo* Sphere::release_header() {
  
  ::autonomous_proto::MessageInfo* temp = _impl_.header_;
  _impl_.header_ = nullptr;
#ifdef PROTOBUF_FORCE_COPY_IN_RELEASE
  auto* old =  reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(temp);
  temp = ::PROTOBUF_NAMESPACE_ID::internal::DuplicateIfNonNull(temp);
  if (GetArenaForAllocation() == nullptr) { delete old; }
#else  // PROTOBUF_FORCE_COPY_IN_RELEASE
  if (GetArenaForAllocation() != nullptr) {
    temp = ::PROTOBUF_NAMESPACE_ID::internal::DuplicateIfNonNull(temp);
  }
#endif  // !PROTOBUF_FORCE_COPY_IN_RELEASE
  return temp;
}
inline ::autonomous_proto::MessageInfo* Sphere::unsafe_arena_release_header() {
  // @@protoc_insertion_point(field_release:autonomous_proto.Sphere.header)
  
  ::autonomous_proto::MessageInfo* temp = _impl_.header_;
  _impl_.header_ = nullptr;
  return temp;
}
inline ::autonomous_proto::MessageInfo* Sphere::_internal_mutable_header() {
  
  if (_impl_.header_ == nullptr) {
    auto* p = CreateMaybeMessage<::autonomous_proto::MessageInfo>(GetArenaForAllocation());
    _impl_.header_ = p;
  }
  return _impl_.header_;
}
inline ::autonomous_proto::MessageInfo* Sphere::mutable_header() {
  ::autonomous_proto::MessageInfo* _msg = _internal_mutable_header();
  // @@protoc_insertion_point(field_mutable:autonomous_proto.Sphere.header)
  return _msg;
}
inline void Sphere::set_allocated_header(::autonomous_proto::MessageInfo* header) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaForAllocation();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(_impl_.header_);
  }
  if (header) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
        ::PROTOBUF_NAMESPACE_ID::Arena::InternalGetOwningArena(
                reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(header));
    if (message_arena != submessage_arena) {
      header = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, header, submessage_arena);
    }
    
  } else {
    
  }
  _impl_.header_ = header;
  // @@protoc_insertion_point(field_set_allocated:autonomous_proto.Sphere.header)
}

// repeated .autonomous_proto.Sphere.Raw data = 2;
inline int Sphere::_internal_data_size() const {
  return _impl_.data_.size();
}
inline int Sphere::data_size() const {
  return _internal_data_size();
}
inline void Sphere::clear_data() {
  _impl_.data_.Clear();
}
inline ::autonomous_proto::Sphere_Raw* Sphere::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:autonomous_proto.Sphere.data)
  return _impl_.data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::autonomous_proto::Sphere_Raw >*
Sphere::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:autonomous_proto.Sphere.data)
  return &_impl_.data_;
}
inline const ::autonomous_proto::Sphere_Raw& Sphere::_internal_data(int index) const {
  return _impl_.data_.Get(index);
}
inline const ::autonomous_proto::Sphere_Raw& Sphere::data(int index) const {
  // @@protoc_insertion_point(field_get:autonomous_proto.Sphere.data)
  return _internal_data(index);
}
inline ::autonomous_proto::Sphere_Raw* Sphere::_internal_add_data() {
  return _impl_.data_.Add();
}
inline ::autonomous_proto::Sphere_Raw* Sphere::add_data() {
  ::autonomous_proto::Sphere_Raw* _add = _internal_add_data();
  // @@protoc_insertion_point(field_add:autonomous_proto.Sphere.data)
  return _add;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::autonomous_proto::Sphere_Raw >&
Sphere::data() const {
  // @@protoc_insertion_point(field_list:autonomous_proto.Sphere.data)
  return _impl_.data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace autonomous_proto

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_sphere_2eproto