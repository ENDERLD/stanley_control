# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: sphere.proto
"""Generated protocol buffer code."""
from google.protobuf.internal import builder as _builder
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from google.protobuf import wrappers_pb2 as google_dot_protobuf_dot_wrappers__pb2
import message_info_pb2 as message__info__pb2


DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\x0csphere.proto\x12\x10\x61utonomous_proto\x1a\x1egoogle/protobuf/wrappers.proto\x1a\x12message_info.proto\"\xaa\x02\n\x06Sphere\x12-\n\x06header\x18\x01 \x01(\x0b\x32\x1d.autonomous_proto.MessageInfo\x12*\n\x04\x64\x61ta\x18\x02 \x03(\x0b\x32\x1c.autonomous_proto.Sphere.Raw\x1a\xc4\x01\n\x03Raw\x12*\n\x04name\x18\x01 \x01(\x0b\x32\x1c.google.protobuf.StringValue\x12\x30\n\x0bint64_value\x18\x02 \x01(\x0b\x32\x1b.google.protobuf.Int64Value\x12\x33\n\rfloat64_value\x18\x03 \x01(\x0b\x32\x1c.google.protobuf.DoubleValue\x12\x13\n\x0bint64_array\x18\x04 \x03(\x03\x12\x15\n\rfloat64_array\x18\x05 \x03(\x01\x62\x06proto3')

_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, globals())
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'sphere_pb2', globals())
if _descriptor._USE_C_DESCRIPTORS == False:

  DESCRIPTOR._options = None
  _SPHERE._serialized_start=87
  _SPHERE._serialized_end=385
  _SPHERE_RAW._serialized_start=189
  _SPHERE_RAW._serialized_end=385
# @@protoc_insertion_point(module_scope)
