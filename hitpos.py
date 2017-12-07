#!/usr/bin/env python2
"""Drone position to syncphony mapper.
"""
from __future__ import print_function
import sys
import struct
import argparse
import traceback
import forward
import parse

POSITION_SAMPLE_COUNT = 5

cdp_pos = {}
cdp_dedup = {}
pos_post = {}
cdp_out = {}


def handle_position_cdp(serial, position, user_data):
    if position is not None:
        if serial in cdp_pos:
            cdp_pos[serial].append(position)
        else:
            cdp_pos[serial] = [position]

    if serial not in pos_post:
        pos_post[serial] = False

    if pos_post[serial] and len(cdp_pos[serial]) >= POSITION_SAMPLE_COUNT:
        for position in cdp_pos[serial][-POSITION_SAMPLE_COUNT:]:
            cdp_out[serial].append("{:08X}: {}".format(
                serial, " ".join(str(round(p, 3)).rjust(12) for p in position)
            ))

        cdp_pos[serial] = []
        pos_post[serial] = False

        for line in cdp_out[serial]:
            print(line)
        print()

    elif serial in cdp_pos and len(cdp_pos[serial]) > 100:
        cdp_pos[serial] = cdp_pos[serial][100:]

    if user_data is None or not len(user_data) or len(user_data) < 12:
        return

    typ = struct.unpack("<B", user_data[:1])[0]
    user_data = user_data[1:]

    if typ == 4:
        sequence, mask, w_ang, v_ang, h_ang, tap_d, tap_v, omni_d, omni_v, shake_d, shake_v, shake_du, lasso_d, lasso_v = struct.unpack(
            '<BBbbbbbbbbbbbb', user_data[:14])

        if serial in cdp_dedup and cdp_dedup[serial] == sequence:
            return
        else:
            cdp_dedup[serial] = sequence

        if (not pos_post[serial]) and (mask & 2) and (serial in cdp_pos):
            cdp_out[serial] = []

            for position in cdp_pos[serial][-POSITION_SAMPLE_COUNT:]:
                cdp_out[serial].append("{:08X}: {}".format(
                    serial, " ".join(str(round(p, 3)).rjust(12) for p in position)
                ))

            cdp_out[serial].append("{:08X}: hit".format(serial))
            cdp_pos[serial] = []
            pos_post[serial] = True


def display_position(serial, position, data):
    if data is not None and len(data):
        data = " ".join("{:02X}".format(ord(c)) for c in data)
    else:
        data = ''

    if position is None:
        pos = "-".rjust(12) * 3 + "\t"
    else:
        pos = " ".join(str(round(p, 3)).rjust(12) for p in position)

    # print("{:08X}: {}\t{}".format(serial, pos, data))
    print("{:08X}: {}".format(serial, pos))


def main(args):
    handler = parse.parse_cdp
    position_handler = handle_position_cdp

    fwd = forward.Forward(
        args.input, args.port, "127.0.0.1", args.port,
        iface=args.iface,
        verbose=False,
        handler=lambda data: handler(data, position_handler)
    )

    print(fwd)
    fwd.start()


if __name__ == '__main__':
    # default video: 239.255.0.80  10077
    # default music: 239.255.0.81  10081
    parser = argparse.ArgumentParser()
    parser.add_argument('-I', '--iface', metavar='ADDR', required=False, help='source interface address (NOT name!)')
    parser.add_argument('-i', '--input', metavar='ADDR', required=True, help='source address')
    parser.add_argument('-p', '--port', metavar='PORT', type=int, required=True, help='source port')

    try:
        params = parser.parse_args()
        main(params)

    except KeyboardInterrupt:
        pass

    except SystemExit:
        raise

    except:
        traceback.print_exc()
        sys.exit(1)

    sys.exit(0)
