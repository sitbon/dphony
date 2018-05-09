from __future__ import print_function
import sys
import struct

MSG_UWB_EVT_TAG_LOC_CHANGED = 0x88
MSG_UWB_EVT_ANCHOR_LOC_CHANGED = 0x89

LCM_MAGIC = 0x4C433032
CIH_MAGIC = 0xC1401A51
LCM_FOOTR = 0x15A1041C

CDP_MAGIC = 0x3230434C
CDP_VERSN = "CDP0002\0"
CDP_T_USER = 0x0007
CDP_T_POS = 0x0100


def parse_cdp(data, handler):
    if len(data) < 20:
        print("cdp: data too short for header", file=sys.stderr)
        return

    mark, seq, version, uid = struct.unpack("<II8sI", data[:20])

    if mark != CDP_MAGIC:
        print("cdp: bad mark 0x{:08X} != 0x%08X (expected)", mark, CDP_MAGIC, file=sys.stderr)
        return

    if version != CDP_VERSN:
        print("cdp: bad version '{}' != '{}' (expected)", version, CDP_VERSN, file=sys.stderr)
        return

    data = data[20:]

    results = []

    while len(data) >= 4:
        typ, size = struct.unpack("<HH", data[:4])

        # print("mark={} seq={} ver={} uid={} typ=0x{:04X} size={}".format(mark, seq, version, uid, typ, size))

        data = data[4:]

        if len(data) < size:
            print("cdp: message specified too small data length {} != {}".format(size, len(data)), file=sys.stderr)
            return

        if typ == CDP_T_USER:
            subtyp = ord(data[0])
            if subtyp == 0x04:
                result = handler(uid, None, data)

                if result:
                    results.extend(result)
            else:
                pass
                # print("cdp: unknown subtype 0x{:02X}".format(subtyp), file=sys.stderr)

        elif typ == CDP_T_POS:
            if len(data) != 24:
                print("cdp: position message has bad length", file=sys.stderr)
                return

            px, py, pz, quality, smoothing, sequence, network_time = struct.unpack("<iiiIHHI", data)

            result = handler(uid, (px / 1000.0, py / 1000.0, pz / 1000.0), None)

            if result:
                results.extend(result)
        else:
            pass

        data = data[size:]

    return results


def parse_dcc(data, handler):
    """
    | SYNC_CODE | Msg Type | Msg SRC | Msg DST | Seq Num | Length  |   Data   |   CRC   |
    --------------------------------------------------------------------------------------
    |  3 Bytes  |  1 Byte  | 2 Bytes | 2 Bytes | 2 Bytes | 4 Bytes | variable | 2 Bytes |
    """
    data_len_exp = len(data) - 16
    sync_code, msg_type, msg_src, msg_dst, seq_num, data_len, msg_data, crc = \
        struct.unpack("<3sBHHHI{}sH".format(data_len_exp), data)

    if data_len != data_len_exp:
        print("dcc: length mismatch {} != {}".format(data_len, data_len_exp))
        return

    if msg_type == MSG_UWB_EVT_ANCHOR_LOC_CHANGED:
        id_hi = 0x00010000
        return
    elif msg_type == MSG_UWB_EVT_TAG_LOC_CHANGED:
        id_hi = 0x00020000
    else:
        return

    count = struct.unpack("<B", msg_data[0])[0]
    msg_data = msg_data[1:]

    results = []

    while count:
        ts, idx, px, py, pz = struct.unpack("<IBfff", msg_data[:17])
        msg_data = msg_data[17:]

        if msg_type == MSG_UWB_EVT_TAG_LOC_CHANGED:
            idx += 128

        result = handler(float(ts) / 1000, id_hi | idx, (px, py, pz))

        if result is not None:
            if type(result) in (list, tuple):
                results.extend(result)
            else:
                results.append(result)

        count -= 1

    return results


def parse_lcm(data, handler):
    if len(data) < 8:
        print("lcm: bad length {}".format(len(data)), file=sys.stderr)
        return None

    lcm_magic, lcm_sequence = struct.unpack(">II", data[:8])

    if lcm_magic != LCM_MAGIC:
        print("lcm: bad magic 0x{:08X} != 0x{:08X}".format(lcm_magic, LCM_MAGIC), file=sys.stderr)
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

        payload_size, dwusb_serial, network_id, px, py, pz, quality, smoothing, timestamp, size = struct.unpack(
           ">HIHffffBQH", data[:35]
        )

        # payload_size == len(data[2:-4])
        # data[:-4] == LCM_FOOTR
        # size == len(user_data)

        user_data = data[35:-4]

        if len(user_data) != size:
            print("lcm: invalid P3 message length {} != {}".format(size, len(user_data) - 4), file=sys.stderr)
            return None

        user_data = user_data[:-4]

        return handler(dwusb_serial, (px, py, pz), user_data)

    else:
        return None
