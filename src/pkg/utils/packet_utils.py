from .utils import *

class Packet:
    DATA_LEN = 5
    PAD_LEN = 3
    SOP_BYTE = 0x55
    EOP_BYTE = 0xAA

    def __init__(self, cmd, *args):
        self.data_fields = [cmd] + list(args) + [0] * (self.DATA_LEN - 1 - len(args))

    def pack(self):
        return bytearray(
            [self.SOP_BYTE]
            + self.data_fields
            + [sum(self.data_fields) % 0x100, self.EOP_BYTE]
        )

    def to_hex(self):
        return ("{:02}:" * self.DATA_LEN).format(*self.data_fields)[:-1]

    @classmethod
    def unpack(cls, resp):
        data_fields = resp[1:-2]
        assert resp[0] == cls.SOP_BYTE, "[Packet] Wrong SOP {}/{}".format(resp[0], cls.SOP_BYTE)
        assert resp[-1] == cls.EOP_BYTE, "[Packet] Wrong EOP {}/{}".format(resp[-1], cls.EOP_BYTE)
        assert resp[-2] == sum(data_fields) % 0x100, "[Packet] Check Sum failure {}/{}".format(
            resp[-2], sum(data_fields) % 0x100)
        return cls(*data_fields)

    @classmethod
    def packet_length(cls):
        return cls.DATA_LEN + cls.PAD_LEN