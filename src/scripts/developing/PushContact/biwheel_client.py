import os
import sys
base_path = os.path.dirname(__file__)
idx_path = base_path.rfind("rnb-planning") + len("rnb-planning")
RNB_PLANNING_DIR = base_path[:idx_path]
sys.path.append(os.path.join(RNB_PLANNING_DIR, 'src'))
from pkg.utils.utils import *
from pkg.utils.packet_utils import *

import serial
import serial.tools.list_ports as sp

DYN_BOARD_PORT = '/dev/ttyACM0'
DYN_BOARD_BD = 115200

CMD_DISABLE = 0
CMD_ENABLE = 1
CMD_VEL_LIM = 2
CMD_VEL_TAR = 3
CMD_DEBUG = 100


class BiWheelCommand(Packet):
    DATA_LEN = 5


class BiWheel:
    def __init__(self, port=DYN_BOARD_PORT, baudrate=DYN_BOARD_BD, timeout=1):
        try:
            self.wport = serial.Serial(port, baudrate, timeout=timeout)
        except Exception as e:
            TextColors.RED.println("[ERROR] Connecting to OpenCR Board")
            print_com_ports()
            print("----------------------------------------------------")
            raise (e)

    def disable(self):
        return self.send_recv(CMD_DISABLE)

    def set_velocity_limit(self, value):
        return self.send_recv(CMD_VEL_LIM, *list(divide_bytes(min(value, 1000))))

    def enable(self):
        return self.send_recv(CMD_ENABLE)

    def set_velocity(self, velocity_l, velocity_r):
        return self.send_recv(CMD_VEL_TAR, *(list(divide_bytes(velocity_l)) + list(divide_bytes(velocity_r))))

    def debug(self, onoff):
        self.send(CMD_DEBUG, 0, int(onoff))
        if onoff:
            return self.recv()
        else:
            self.flush()

    def send(self, cmd, *args):
        cmd_pkt = BiWheelCommand(cmd, *args)
        self.wport.write(cmd_pkt.pack())

    def read_raw(self):
        return self.wport.read(BiWheelCommand.packet_length())

    def send_read_raw(self, cmd, *args):
        self.send(cmd, *args)
        return self.read_raw()

    def recv(self):
        resp = self.read_raw()
        if type(resp) == str:
            resp = str2ascii(resp)
        res_pkt = BiWheelCommand.unpack(resp)
        return res_pkt.data_fields

    def send_recv(self, cmd, *args):
        self.send(cmd, *args)
        return self.recv()

    def flush(self):
        while True:
            recved = self.read_raw()
            print(recved)
            if not recved:
                break