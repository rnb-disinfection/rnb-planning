# -*- coding: utf-8 -*- 

# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: IndyScriptServices.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


import IndyGeneral_pb2 as IndyGeneral__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='IndyScriptServices.proto',
  package='IndyFramework.rpc.Script',
  syntax='proto3',
  serialized_options=None,
  serialized_pb=_b('\n\x18IndyScriptServices.proto\x12\x18IndyFramework.rpc.Script\x1a\x11IndyGeneral.proto2\xe9\x02\n\x0eScriptDirector\x12U\n\x0cstartProgram\x12 .IndyFramework.rpc.StringMessage\x1a!.IndyFramework.rpc.BooleanMessage\"\x00\x12S\n\x0bstopProgram\x12\x1f.IndyFramework.rpc.EmptyMessage\x1a!.IndyFramework.rpc.BooleanMessage\"\x00\x12T\n\x0cpauseProgram\x12\x1f.IndyFramework.rpc.EmptyMessage\x1a!.IndyFramework.rpc.BooleanMessage\"\x00\x12U\n\rresumeProgram\x12\x1f.IndyFramework.rpc.EmptyMessage\x1a!.IndyFramework.rpc.BooleanMessage\"\x00\x62\x06proto3')
  ,
  dependencies=[IndyGeneral__pb2.DESCRIPTOR,])



_sym_db.RegisterFileDescriptor(DESCRIPTOR)



_SCRIPTDIRECTOR = _descriptor.ServiceDescriptor(
  name='ScriptDirector',
  full_name='IndyFramework.rpc.Script.ScriptDirector',
  file=DESCRIPTOR,
  index=0,
  serialized_options=None,
  serialized_start=74,
  serialized_end=435,
  methods=[
  _descriptor.MethodDescriptor(
    name='startProgram',
    full_name='IndyFramework.rpc.Script.ScriptDirector.startProgram',
    index=0,
    containing_service=None,
    input_type=IndyGeneral__pb2._STRINGMESSAGE,
    output_type=IndyGeneral__pb2._BOOLEANMESSAGE,
    serialized_options=None,
  ),
  _descriptor.MethodDescriptor(
    name='stopProgram',
    full_name='IndyFramework.rpc.Script.ScriptDirector.stopProgram',
    index=1,
    containing_service=None,
    input_type=IndyGeneral__pb2._EMPTYMESSAGE,
    output_type=IndyGeneral__pb2._BOOLEANMESSAGE,
    serialized_options=None,
  ),
  _descriptor.MethodDescriptor(
    name='pauseProgram',
    full_name='IndyFramework.rpc.Script.ScriptDirector.pauseProgram',
    index=2,
    containing_service=None,
    input_type=IndyGeneral__pb2._EMPTYMESSAGE,
    output_type=IndyGeneral__pb2._BOOLEANMESSAGE,
    serialized_options=None,
  ),
  _descriptor.MethodDescriptor(
    name='resumeProgram',
    full_name='IndyFramework.rpc.Script.ScriptDirector.resumeProgram',
    index=3,
    containing_service=None,
    input_type=IndyGeneral__pb2._EMPTYMESSAGE,
    output_type=IndyGeneral__pb2._BOOLEANMESSAGE,
    serialized_options=None,
  ),
])
_sym_db.RegisterServiceDescriptor(_SCRIPTDIRECTOR)

DESCRIPTOR.services_by_name['ScriptDirector'] = _SCRIPTDIRECTOR

# @@protoc_insertion_point(module_scope)
