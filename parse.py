from __future__ import print_function
import sys
import struct

MSG_UWB_EVT_TAG_LOC_CHANGED = 0x88
MSG_UWB_EVT_ANCHOR_LOC_CHANGED = 0x89

LCM_MAGIC = 0x4C433032
CIH_MAGIC = 0xC1401A51


def parse_dcc(data, handler):
    """
    | SYNC_CODE | Msg Type | Msg SRC | Msg DST | Seq Num | Length  |   Data   |   CRC   |
    --------------------------------------------------------------------------------------
    |  3 Bytes  |  1 Byte  | 2 Bytes | 2 Bytes | 2 Bytes | 4 Bytes | variable | 2 Bytes |
    """
    data_len_exp = len(data) - 16
    sync_code, msg_type, msg_src, msg_dst, seq_num, data_len, msg_data, crc = \
        struct.unpack("<3sBHHHI{}sH".format(data_len_exp))

    if data_len != data_len_exp:
        print("dcc: length mismatch {} != {}".format(data_len, data_len_exp))
        return None

    if msg_type not in (MSG_UWB_EVT_ANCHOR_LOC_CHANGED, MSG_UWB_EVT_TAG_LOC_CHANGED):
        return None

    pos_num, pos_idx, px, py, pz = struct.unpack("<BBHHH", msg_data[:8])

    return handler(msg_src, (px/100.0, py/100.0, pz/100.0))


def parse_lcm(data, handler):
    if len(data) < 8:
        print("lcm: bad length {}".format(len(data)), file=sys.stderr)
        return None

    magic_lcm, random1 = struct.unpack(">II", data[:8])

    if magic_lcm != LCM_MAGIC:
        print("lcm: bad magic 0x{:08X} != 0x{:08X}".format(magic_lcm, LCM_MAGIC), file=sys.stderr)
        return None

    channel_name = ''
    data = data[8:]
    index = 0

    if len(data) < 3:
        print("lcm: message too short (parse channel name)", file=sys.stderr)
        return None

    while data[index] != '\x00':
        channel_name += data[index]
        index += 1
        if index >= len(data):
            print("lcm: could not parse channel name", file=sys.stderr)
            return None

    data = data[index+1:]

    if len(data) < 4:
        print("lcm: message too short (parse ciholas magic)", file=sys.stderr)
        return None

    magic_cih = struct.unpack(">I", data[:4])[0]

    if magic_cih != CIH_MAGIC:
        print("lcm: bad ciholas magic 0x{:08X} != 0x{:08X}".format(magic_cih, CIH_MAGIC), file=sys.stderr)
        return None

    data = data[4:]

    if channel_name == 'P3':
        if len(data) < 35:
            print("lcm: P3 message too short", file=sys.stderr)
            return None

        random2, dwusb_serial, random3, px, py, pz, random4, size = struct.unpack(
           ">2sI2sfff13sH", data[:35]
        )

        user_data = data[35:]

        if len(user_data) != (size + 4):
            print("lcm: invalid P3 message length {} != {}".format(size, len(user_data) - 4), file=sys.stderr)
            return None

        user_data = user_data[:-4]

        return handler(dwusb_serial, (px, py, pz))

    else:
        return None
