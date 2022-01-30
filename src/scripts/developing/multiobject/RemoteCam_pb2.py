# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: RemoteCam.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='RemoteCam.proto',
  package='RemoteCamProto',
  syntax='proto3',
  serialized_options=b'\n%io.grpc.custom.Disinfection.RemoteCamB\016RemoteCamProtoP\001\242\002\004FDDO',
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n\x0fRemoteCam.proto\x12\x0eRemoteCamProto\"&\n\x10GetConfigRequest\x12\x12\n\nrequest_id\x18\x01 \x01(\x05\"\x90\x01\n\x11GetConfigResponse\x12\x13\n\x0bresponse_id\x18\x01 \x01(\x05\x12\x19\n\rcamera_matrix\x18\x02 \x03(\x02\x42\x02\x10\x01\x12\x17\n\x0b\x64ist_coeffs\x18\x03 \x03(\x02\x42\x02\x10\x01\x12\x13\n\x0b\x64\x65pth_scale\x18\x04 \x01(\x02\x12\r\n\x05width\x18\x05 \x01(\x05\x12\x0e\n\x06height\x18\x06 \x01(\x05\"%\n\x0fGetImageRequest\x12\x12\n\nrequest_id\x18\x01 \x01(\x05\"Y\n\x10GetImageResponse\x12\x13\n\x0bresponse_id\x18\x01 \x01(\x05\x12\r\n\x05width\x18\x02 \x01(\x05\x12\x0e\n\x06height\x18\x03 \x01(\x05\x12\x11\n\x05\x63olor\x18\x04 \x03(\x05\x42\x02\x10\x01\"-\n\x17GetImageDepthmapRequest\x12\x12\n\nrequest_id\x18\x01 \x01(\x05\"t\n\x18GetImageDepthmapResponse\x12\x13\n\x0bresponse_id\x18\x01 \x01(\x05\x12\r\n\x05width\x18\x02 \x01(\x05\x12\x0e\n\x06height\x18\x03 \x01(\x05\x12\x11\n\x05\x63olor\x18\x04 \x03(\x05\x42\x02\x10\x01\x12\x11\n\x05\x64\x65pth\x18\x05 \x03(\x05\x42\x02\x10\x01\x32\x9e\x02\n\x0eRemoteCamProto\x12R\n\tGetConfig\x12 .RemoteCamProto.GetConfigRequest\x1a!.RemoteCamProto.GetConfigResponse\"\x00\x12O\n\x08GetImage\x12\x1f.RemoteCamProto.GetImageRequest\x1a .RemoteCamProto.GetImageResponse\"\x00\x12g\n\x10GetImageDepthmap\x12\'.RemoteCamProto.GetImageDepthmapRequest\x1a(.RemoteCamProto.GetImageDepthmapResponse\"\x00\x42@\n%io.grpc.custom.Disinfection.RemoteCamB\x0eRemoteCamProtoP\x01\xa2\x02\x04\x46\x44\x44Ob\x06proto3'
)




_GETCONFIGREQUEST = _descriptor.Descriptor(
  name='GetConfigRequest',
  full_name='RemoteCamProto.GetConfigRequest',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='request_id', full_name='RemoteCamProto.GetConfigRequest.request_id', index=0,
      number=1, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=35,
  serialized_end=73,
)


_GETCONFIGRESPONSE = _descriptor.Descriptor(
  name='GetConfigResponse',
  full_name='RemoteCamProto.GetConfigResponse',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='response_id', full_name='RemoteCamProto.GetConfigResponse.response_id', index=0,
      number=1, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='camera_matrix', full_name='RemoteCamProto.GetConfigResponse.camera_matrix', index=1,
      number=2, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=b'\020\001', file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='dist_coeffs', full_name='RemoteCamProto.GetConfigResponse.dist_coeffs', index=2,
      number=3, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=b'\020\001', file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='depth_scale', full_name='RemoteCamProto.GetConfigResponse.depth_scale', index=3,
      number=4, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='width', full_name='RemoteCamProto.GetConfigResponse.width', index=4,
      number=5, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='height', full_name='RemoteCamProto.GetConfigResponse.height', index=5,
      number=6, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=76,
  serialized_end=220,
)


_GETIMAGEREQUEST = _descriptor.Descriptor(
  name='GetImageRequest',
  full_name='RemoteCamProto.GetImageRequest',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='request_id', full_name='RemoteCamProto.GetImageRequest.request_id', index=0,
      number=1, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=222,
  serialized_end=259,
)


_GETIMAGERESPONSE = _descriptor.Descriptor(
  name='GetImageResponse',
  full_name='RemoteCamProto.GetImageResponse',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='response_id', full_name='RemoteCamProto.GetImageResponse.response_id', index=0,
      number=1, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='width', full_name='RemoteCamProto.GetImageResponse.width', index=1,
      number=2, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='height', full_name='RemoteCamProto.GetImageResponse.height', index=2,
      number=3, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='color', full_name='RemoteCamProto.GetImageResponse.color', index=3,
      number=4, type=5, cpp_type=1, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=b'\020\001', file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=261,
  serialized_end=350,
)


_GETIMAGEDEPTHMAPREQUEST = _descriptor.Descriptor(
  name='GetImageDepthmapRequest',
  full_name='RemoteCamProto.GetImageDepthmapRequest',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='request_id', full_name='RemoteCamProto.GetImageDepthmapRequest.request_id', index=0,
      number=1, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=352,
  serialized_end=397,
)


_GETIMAGEDEPTHMAPRESPONSE = _descriptor.Descriptor(
  name='GetImageDepthmapResponse',
  full_name='RemoteCamProto.GetImageDepthmapResponse',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='response_id', full_name='RemoteCamProto.GetImageDepthmapResponse.response_id', index=0,
      number=1, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='width', full_name='RemoteCamProto.GetImageDepthmapResponse.width', index=1,
      number=2, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='height', full_name='RemoteCamProto.GetImageDepthmapResponse.height', index=2,
      number=3, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='color', full_name='RemoteCamProto.GetImageDepthmapResponse.color', index=3,
      number=4, type=5, cpp_type=1, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=b'\020\001', file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='depth', full_name='RemoteCamProto.GetImageDepthmapResponse.depth', index=4,
      number=5, type=5, cpp_type=1, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=b'\020\001', file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=399,
  serialized_end=515,
)

DESCRIPTOR.message_types_by_name['GetConfigRequest'] = _GETCONFIGREQUEST
DESCRIPTOR.message_types_by_name['GetConfigResponse'] = _GETCONFIGRESPONSE
DESCRIPTOR.message_types_by_name['GetImageRequest'] = _GETIMAGEREQUEST
DESCRIPTOR.message_types_by_name['GetImageResponse'] = _GETIMAGERESPONSE
DESCRIPTOR.message_types_by_name['GetImageDepthmapRequest'] = _GETIMAGEDEPTHMAPREQUEST
DESCRIPTOR.message_types_by_name['GetImageDepthmapResponse'] = _GETIMAGEDEPTHMAPRESPONSE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

GetConfigRequest = _reflection.GeneratedProtocolMessageType('GetConfigRequest', (_message.Message,), {
  'DESCRIPTOR' : _GETCONFIGREQUEST,
  '__module__' : 'RemoteCam_pb2'
  # @@protoc_insertion_point(class_scope:RemoteCamProto.GetConfigRequest)
  })
_sym_db.RegisterMessage(GetConfigRequest)

GetConfigResponse = _reflection.GeneratedProtocolMessageType('GetConfigResponse', (_message.Message,), {
  'DESCRIPTOR' : _GETCONFIGRESPONSE,
  '__module__' : 'RemoteCam_pb2'
  # @@protoc_insertion_point(class_scope:RemoteCamProto.GetConfigResponse)
  })
_sym_db.RegisterMessage(GetConfigResponse)

GetImageRequest = _reflection.GeneratedProtocolMessageType('GetImageRequest', (_message.Message,), {
  'DESCRIPTOR' : _GETIMAGEREQUEST,
  '__module__' : 'RemoteCam_pb2'
  # @@protoc_insertion_point(class_scope:RemoteCamProto.GetImageRequest)
  })
_sym_db.RegisterMessage(GetImageRequest)

GetImageResponse = _reflection.GeneratedProtocolMessageType('GetImageResponse', (_message.Message,), {
  'DESCRIPTOR' : _GETIMAGERESPONSE,
  '__module__' : 'RemoteCam_pb2'
  # @@protoc_insertion_point(class_scope:RemoteCamProto.GetImageResponse)
  })
_sym_db.RegisterMessage(GetImageResponse)

GetImageDepthmapRequest = _reflection.GeneratedProtocolMessageType('GetImageDepthmapRequest', (_message.Message,), {
  'DESCRIPTOR' : _GETIMAGEDEPTHMAPREQUEST,
  '__module__' : 'RemoteCam_pb2'
  # @@protoc_insertion_point(class_scope:RemoteCamProto.GetImageDepthmapRequest)
  })
_sym_db.RegisterMessage(GetImageDepthmapRequest)

GetImageDepthmapResponse = _reflection.GeneratedProtocolMessageType('GetImageDepthmapResponse', (_message.Message,), {
  'DESCRIPTOR' : _GETIMAGEDEPTHMAPRESPONSE,
  '__module__' : 'RemoteCam_pb2'
  # @@protoc_insertion_point(class_scope:RemoteCamProto.GetImageDepthmapResponse)
  })
_sym_db.RegisterMessage(GetImageDepthmapResponse)


DESCRIPTOR._options = None
_GETCONFIGRESPONSE.fields_by_name['camera_matrix']._options = None
_GETCONFIGRESPONSE.fields_by_name['dist_coeffs']._options = None
_GETIMAGERESPONSE.fields_by_name['color']._options = None
_GETIMAGEDEPTHMAPRESPONSE.fields_by_name['color']._options = None
_GETIMAGEDEPTHMAPRESPONSE.fields_by_name['depth']._options = None

_REMOTECAMPROTO = _descriptor.ServiceDescriptor(
  name='RemoteCamProto',
  full_name='RemoteCamProto.RemoteCamProto',
  file=DESCRIPTOR,
  index=0,
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_start=518,
  serialized_end=804,
  methods=[
  _descriptor.MethodDescriptor(
    name='GetConfig',
    full_name='RemoteCamProto.RemoteCamProto.GetConfig',
    index=0,
    containing_service=None,
    input_type=_GETCONFIGREQUEST,
    output_type=_GETCONFIGRESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetImage',
    full_name='RemoteCamProto.RemoteCamProto.GetImage',
    index=1,
    containing_service=None,
    input_type=_GETIMAGEREQUEST,
    output_type=_GETIMAGERESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetImageDepthmap',
    full_name='RemoteCamProto.RemoteCamProto.GetImageDepthmap',
    index=2,
    containing_service=None,
    input_type=_GETIMAGEDEPTHMAPREQUEST,
    output_type=_GETIMAGEDEPTHMAPRESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
])
_sym_db.RegisterServiceDescriptor(_REMOTECAMPROTO)

DESCRIPTOR.services_by_name['RemoteCamProto'] = _REMOTECAMPROTO

# @@protoc_insertion_point(module_scope)
