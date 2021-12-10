import os
import sys
sys.path.append(os.path.join(os.path.join(
    os.environ["RNB_PLANNING_DIR"], 'src')))
from pkg.utils.utils import *

import serial
import serial.tools.list_ports as sp
import numpy as np
import time
from enum import Enum
from ctypes import c_int16

KIRO_TOOL_PORT = '/dev/ttyUSB0'
KIRO_TOOL_BDRATE = 115200
STX_BYTE, ETX_BYTE = 0x02, 0x03


class CMD_BASE(Enum):
    
    @classmethod
    def from_value(cls, value):
        for _, item in enumerate(cls):
            if item.value==value:
                return item
    
    @classmethod
    def from_name(cls, name):
        for _, item in enumerate(cls):
            if item.name==name:
                return item

class MOTOR_CMD(CMD_BASE):
    DISABLE = 0xff
    ENABLE = 0x01
    OP = 0x02
    OP_INIT = 0x03

class OP_CMD(CMD_BASE):
    NONE = 0x00
    FWD = 0x01
    BWD = 0x02
    ANGLE = 0x03
    LED_ON = 0x04
    LED_OFF = 0x05
    STOP = 0xff
    DEGREE = 0x06
     
class KiroToolPacket:
    DATA_LEN=4
    PAD_LEN=3
    def __init__(self, motor_cmd, op_cmd=OP_CMD.NONE, *args):
        if isinstance(motor_cmd, MOTOR_CMD):
            motor_cmd = motor_cmd.value
        if isinstance(op_cmd, OP_CMD):
            op_cmd = op_cmd.value
        self.data_fields = [motor_cmd, op_cmd] + list(args) +[0]*(self.DATA_LEN-2-len(args))
        
    def pack(self):
        return bytearray(
            [STX_BYTE]
            +self.data_fields
            +[(STX_BYTE+sum(self.data_fields))%0x100, ETX_BYTE]
        )
    
    def to_hex(self):
        return ("{:02}:"*self.DATA_LEN).format(*self.data_fields)[:-1]    
    
    @classmethod
    def unpack(cls, resp, checksum=False):
        stx_data_fields = resp[0:-2]
        assert resp[0]==STX_BYTE, "[Packet] Wrong STX {}/{}".format(resp[0], STX_BYTE)
        assert resp[-1]==ETX_BYTE, "[Packet] Wrong ETX {}/{}".format(resp[-1], ETX_BYTE)
        if checksum:
            assert resp[-2]==sum(stx_data_fields)%0x100, "[Packet] Check Sum failure {}/{}".format(
                resp[-2], sum(stx_data_fields)%0x100)
        return cls(*stx_data_fields[1:])
    
    
    @classmethod
    def packet_length(cls):
        return cls.DATA_LEN + cls.PAD_LEN

##
# @class KiroToolPort
# @brief  Connection interface for Kiro Tool USB Connection
class KiroToolPort:
    def __init__(self, port_name=KIRO_TOOL_PORT, 
                 baudrate=KIRO_TOOL_BDRATE, timeout=1):
        self.port_name, self.baudrate = port_name, baudrate
        self.sport = serial.Serial(port_name, baudrate, timeout=timeout)
#         sudo chwon $USER port_name
        print("==== Kiro Tool connected ====")
        self.flush()
#         try:
#             self.initialize()
#         except Exception as e: 
#             print("[ERROR] Run this bash command to allow USB access: {}".format(
#                 "sudo usermod -a -G dialout $USER"))
#             print(e)
#             raise(e)
        
    def disable(self):
        data_fields = self.send_recv(MOTOR_CMD.DISABLE)
        return ascii2str(data_fields)
        
    def enable(self):
        return self.send_recv_check_a0(MOTOR_CMD.ENABLE)
        
    def op_init(self):
        return self.send_recv_check_a0(MOTOR_CMD.OP_INIT)
    
    def initialize(self):
        for _ in range(10):
            if self.enable():
                break
            time.sleep(1)
        for _ in range(10):
            if self.op_init():
                break
            time.sleep(1)
    
    def stop(self):
        return self.send_recv_check_a0(MOTOR_CMD.OP, OP_CMD.STOP)
    
    def roll_forward(self):
        return self.send_recv_check_a0(MOTOR_CMD.OP, OP_CMD.FWD)
    
    def roll_back(self):
        return self.send_recv_check_a0(MOTOR_CMD.OP, OP_CMD.BWD)
    
    def led_on(self):
        return self.send_recv_check_a0(MOTOR_CMD.OP, OP_CMD.LED_ON)
    
    def led_off(self):
        return self.send_recv_check_a0(MOTOR_CMD.OP, OP_CMD.LED_OFF)
    
    def send_raw(self, raw):
        return self.send_recv_check_a0(MOTOR_CMD.OP, OP_CMD.DEGREE, raw/0x100%0x100, raw%0x100)
    
    def send_degree(self, deci_degree):
        return self.send_recv_check_a0(MOTOR_CMD.OP, OP_CMD.DEGREE, *divide_bytes(deci_degree%0x10000))
        
    def close(self):
        self.sport.close()
        
    def send_recv(self, motor_cmd, op_cmd=OP_CMD.NONE, *args):
        self.send(motor_cmd, op_cmd, *args)
        return self.recv()
        
    def send_recv_check_a0(self, motor_cmd, op_cmd=OP_CMD.NONE, *args):
        recv_dat = self.send_recv(motor_cmd, op_cmd, *args)
        return recv_dat[0] == (motor_cmd.value + 0xa0)%0x100
            
        
    def send(self, motor_cmd, op_cmd=OP_CMD.NONE, *args):
        cmd_pkt = KiroToolPacket(motor_cmd, op_cmd, *args)
        self.sport.write(cmd_pkt.pack())
        
    def recv(self):
        resp = self.sport.read(KiroToolPacket.packet_length())
        if len(resp)==0:
            raise(RuntimeError("ERROR: packet not returned"))
        if type(resp) == str:
            resp = str2ascii(resp)
        res_pkt = KiroToolPacket.unpack(resp)
        return res_pkt.data_fields
    
    def flush(self):
        self.sport.read_all()