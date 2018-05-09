#!/usr/bin/env python2
"""General purpose CDP position relay via OSC.
"""
from __future__ import print_function
import os
import sys
import argparse
import traceback
import math
import forward
import parse
import OSC


ORIGIN_DEFAULT = (0, 0, 0)
DIRECTION = (1, 1, 1)
DELTA_THRESHOLD = 0.35

DEVICE_FILTER = {

    0x0602136A: ("target", ORIGIN_DEFAULT),

}


def handle_position(serial, position, user_data):
    if serial not in DEVICE_FILTER:
        return

    name, origin = DEVICE_FILTER[serial]

    result = []

    if position is not None:
        position = transform_position(position, origin)
        position = position_smooth(serial, position)
        position = position_hysteresis(serial, position)

        if position:
            if params.verbose:
                pos = " ".join(str(round(p, 3)).rjust(12) for p in position)
                print("{:08X}: {}".format(serial, pos))

            result.append(osc_position(name, position))

    return result


position_prev = {}
NO_POSITION = [float('inf')] * 3


def position_hysteresis(serial, position):
    prev = position_prev.get(serial, NO_POSITION)

    if distance(position, prev) < DELTA_THRESHOLD:
        return None  # Alternatively, continue reporting the same position

    position_prev[serial] = position
    return position


def transform_position(position, origin=(0, 0, 0)):
    return [(a * b) - c for a, b, c in zip(position, DIRECTION, origin)]


lowpass_o1 = {}
lowpass_o2 = {}


def position_smooth(serial, position):
    if serial not in lowpass_o1:
        lowpass_o1[serial] = [0, 0, 0]
        lowpass_o2[serial] = [0, 0, 0]

    lowpass_o1[serial] = [lp1 * 0.7 + p * 0.3 for lp1, p in zip(lowpass_o1[serial], position)]
    lowpass_o2[serial] = [lp2 * 0.7 + lp1 * 0.3 for lp2, lp1 in zip(lowpass_o2[serial], lowpass_o1[serial])]

    return lowpass_o2[serial]


def distance(p1, p2):
    return math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2 + (p2[2] - p1[2])**2)


def planar_distance(p1, p2):
    return math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)


def median(arr):
    sarr = sorted(arr)
    count = len(sarr)

    if not count % 2:
        return (sarr[count / 2] + sarr[count / 2 - 1]) / 2.0

    return sarr[count / 2]


def osc_position(serial, position):
    return osc_message("/position/{}".format(serial), *position)


def osc_message(path, *data):
    return OSC.OSCMessage(path, data).getBinary()


def display_position(serial, position, data):
    if data is not None and len(data):
        data = " ".join("{:02X}".format(ord(c)) for c in data)
    else:
        data = ''

    if position is None:
        pos = "-".rjust(12) * 3 + "\t"
    else:
        position = transform_position(position, DEVICE_FILTER.get(serial, (None, ORIGIN_DEFAULT))[1])
        position = position_smooth(serial, position)
        position = position_hysteresis(serial, position)

        if not position:
            return

        pos = " ".join(str(round(p, 3)).rjust(12) for p in position)

    # print("{:08X}: {}\t{}".format(serial, pos, data))
    print("{:08X}: {}".format(serial, pos))


def main(args):
    if not args.out_port:
        args.out_port = args.port

    handler = parse.parse_cdp

    if args.debug:
        position_handler = display_position
    else:
        position_handler = handle_position

    fwd = forward.Forward(
        args.input, args.port, args.out, args.out_port,
        iface=args.iface,
        iface_out=args.out_iface,
        verbose=False,
        handler=lambda data: handler(data, position_handler)
    )

    print(fwd)
    fwd.start()


if __name__ == '__main__':
    # default: 239.255.0.90  10090
    parser = argparse.ArgumentParser()
    parser.add_argument('-I', '--iface', metavar='ADDR', required=False, help='source interface address (NOT name!)')
    parser.add_argument('-i', '--input', metavar='ADDR', required=True, help='source address')
    parser.add_argument('-p', '--port', metavar='PORT', type=int, required=True, help='source port')
    parser.add_argument('-o', '--out', metavar='ADDR', required=True, help='destination address')
    parser.add_argument('-O', '--out-iface', metavar='ADDR', required=False, help='outgoing iface address (NOT name!)')
    parser.add_argument('-P', '--out-port', metavar='PORT', type=int, help='destination port')
    parser.add_argument('-v', '--verbose', action='store_true', help='verbose output')
    parser.add_argument('-D', '--debug', action='store_true', help='debug mode')

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