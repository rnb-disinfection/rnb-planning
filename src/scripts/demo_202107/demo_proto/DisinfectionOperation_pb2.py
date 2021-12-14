# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: DisinfectionOperation.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='DisinfectionOperation.proto',
  package='DisinfectionOperation',
  syntax='proto3',
  serialized_options=b'\n1io.grpc.custom.Disinfection.DisinfectionOperationB\032DisinfectionOperationProtoP\001\242\002\004FDDO',
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n\x1b\x44isinfectionOperation.proto\x12\x15\x44isinfectionOperation\"t\n\x15\x44oDisinfectionRequest\x12\x11\n\tobject_id\x18\x01 \x01(\x05\x12\x13\n\x0bobject_name\x18\x02 \x01(\t\x12\x10\n\x08\x63\x65nter_x\x18\x03 \x01(\x01\x12\x10\n\x08\x63\x65nter_y\x18\x04 \x01(\x01\x12\x0f\n\x07heading\x18\x05 \x01(\x01\"/\n\x16\x44oDisinfectionResponse\x12\x15\n\rresponse_flag\x18\x01 \x01(\x05\"5\n\x1d\x44oDisinfectionCompleteRequest\x12\x14\n\x0crequest_flag\x18\x01 \x01(\x05\"7\n\x1e\x44oDisinfectionCompleteResponse\x12\x15\n\rresponse_flag\x18\x01 \x01(\x05\x32\x92\x02\n\x15\x44isinfectionOperation\x12o\n\x0e\x44oDisinfection\x12,.DisinfectionOperation.DoDisinfectionRequest\x1a-.DisinfectionOperation.DoDisinfectionResponse\"\x00\x12\x87\x01\n\x16\x44oDisinfectionComplete\x12\x34.DisinfectionOperation.DoDisinfectionCompleteRequest\x1a\x35.DisinfectionOperation.DoDisinfectionCompleteResponse\"\x00\x42X\n1io.grpc.custom.Disinfection.DisinfectionOperationB\x1a\x44isinfectionOperationProtoP\x01\xa2\x02\x04\x46\x44\x44Ob\x06proto3'
)




_DODISINFECTIONREQUEST = _descriptor.Descriptor(
  name='DoDisinfectionRequest',
  full_name='DisinfectionOperation.DoDisinfectionRequest',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='object_id', full_name='DisinfectionOperation.DoDisinfectionRequest.object_id', index=0,
      number=1, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='object_name', full_name='DisinfectionOperation.DoDisinfectionRequest.object_name', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='center_x', full_name='DisinfectionOperation.DoDisinfectionRequest.center_x', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='center_y', full_name='DisinfectionOperation.DoDisinfectionRequest.center_y', index=3,
      number=4, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='heading', full_name='DisinfectionOperation.DoDisinfectionRequest.heading', index=4,
      number=5, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
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
  serialized_start=54,
  serialized_end=170,
)


_DODISINFECTIONRESPONSE = _descriptor.Descriptor(
  name='DoDisinfectionResponse',
  full_name='DisinfectionOperation.DoDisinfectionResponse',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='response_flag', full_name='DisinfectionOperation.DoDisinfectionResponse.response_flag', index=0,
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
  serialized_start=172,
  serialized_end=219,
)


_DODISINFECTIONCOMPLETEREQUEST = _descriptor.Descriptor(
  name='DoDisinfectionCompleteRequest',
  full_name='DisinfectionOperation.DoDisinfectionCompleteRequest',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='request_flag', full_name='DisinfectionOperation.DoDisinfectionCompleteRequest.request_flag', index=0,
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
  serialized_start=221,
  serialized_end=274,
)


_DODISINFECTIONCOMPLETERESPONSE = _descriptor.Descriptor(
  name='DoDisinfectionCompleteResponse',
  full_name='DisinfectionOperation.DoDisinfectionCompleteResponse',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='response_flag', full_name='DisinfectionOperation.DoDisinfectionCompleteResponse.response_flag', index=0,
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
  serialized_start=276,
  serialized_end=331,
)

DESCRIPTOR.message_types_by_name['DoDisinfectionRequest'] = _DODISINFECTIONREQUEST
DESCRIPTOR.message_types_by_name['DoDisinfectionResponse'] = _DODISINFECTIONRESPONSE
DESCRIPTOR.message_types_by_name['DoDisinfectionCompleteRequest'] = _DODISINFECTIONCOMPLETEREQUEST
DESCRIPTOR.message_types_by_name['DoDisinfectionCompleteResponse'] = _DODISINFECTIONCOMPLETERESPONSE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

DoDisinfectionRequest = _reflection.GeneratedProtocolMessageType('DoDisinfectionRequest', (_message.Message,), {
  'DESCRIPTOR' : _DODISINFECTIONREQUEST,
  '__module__' : 'DisinfectionOperation_pb2'
  # @@protoc_insertion_point(class_scope:DisinfectionOperation.DoDisinfectionRequest)
  })
_sym_db.RegisterMessage(DoDisinfectionRequest)

DoDisinfectionResponse = _reflection.GeneratedProtocolMessageType('DoDisinfectionResponse', (_message.Message,), {
  'DESCRIPTOR' : _DODISINFECTIONRESPONSE,
  '__module__' : 'DisinfectionOperation_pb2'
  # @@protoc_insertion_point(class_scope:DisinfectionOperation.DoDisinfectionResponse)
  })
_sym_db.RegisterMessage(DoDisinfectionResponse)

DoDisinfectionCompleteRequest = _reflection.GeneratedProtocolMessageType('DoDisinfectionCompleteRequest', (_message.Message,), {
  'DESCRIPTOR' : _DODISINFECTIONCOMPLETEREQUEST,
  '__module__' : 'DisinfectionOperation_pb2'
  # @@protoc_insertion_point(class_scope:DisinfectionOperation.DoDisinfectionCompleteRequest)
  })
_sym_db.RegisterMessage(DoDisinfectionCompleteRequest)

DoDisinfectionCompleteResponse = _reflection.GeneratedProtocolMessageType('DoDisinfectionCompleteResponse', (_message.Message,), {
  'DESCRIPTOR' : _DODISINFECTIONCOMPLETERESPONSE,
  '__module__' : 'DisinfectionOperation_pb2'
  # @@protoc_insertion_point(class_scope:DisinfectionOperation.DoDisinfectionCompleteResponse)
  })
_sym_db.RegisterMessage(DoDisinfectionCompleteResponse)


DESCRIPTOR._options = None

_DISINFECTIONOPERATION = _descriptor.ServiceDescriptor(
  name='DisinfectionOperation',
  full_name='DisinfectionOperation.DisinfectionOperation',
  file=DESCRIPTOR,
  index=0,
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_start=334,
  serialized_end=608,
  methods=[
  _descriptor.MethodDescriptor(
    name='DoDisinfection',
    full_name='DisinfectionOperation.DisinfectionOperation.DoDisinfection',
    index=0,
    containing_service=None,
    input_type=_DODISINFECTIONREQUEST,
    output_type=_DODISINFECTIONRESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='DoDisinfectionComplete',
    full_name='DisinfectionOperation.DisinfectionOperation.DoDisinfectionComplete',
    index=1,
    containing_service=None,
    input_type=_DODISINFECTIONCOMPLETEREQUEST,
    output_type=_DODISINFECTIONCOMPLETERESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
])
_sym_db.RegisterServiceDescriptor(_DISINFECTIONOPERATION)

DESCRIPTOR.services_by_name['DisinfectionOperation'] = _DISINFECTIONOPERATION

# @@protoc_insertion_point(module_scope)
