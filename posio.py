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
import cleanup
import parse
import OSC


ORIGIN_DEFAULT = (0, 0, 0)
DIRECTION = (1, 1, 1)
HYSTERESIS_ENABLE = False
HYSTERESIS_REPEAT = True
DELTA_THRESHOLD = 0.05
WINDOW_MEAN_SIZE = 1
SMOOTH_ENABLE = True
SMOOTH_COEFF = [(0.98, 0.02), (0.9, 0.1)]
VELOCITY_FILTER_ENABLE = True
VELOCITY_THRESHOLD = 10

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
        position = position_smooth_old(serial, position)
        position = position_window_mean(serial, position)
        position = position_hysteresis(serial, position)

        if position:
            if params.verbose:
                pos = " ".join(str(round(p, 3)).rjust(12) for p in position)
                print("{:08X}: {}".format(serial, pos))

            result.append(osc_position(name, position))

    return result


window_mean = {}


def position_window_mean(serial, position):
    if WINDOW_MEAN_SIZE <= 1 or not position:
        return position

    points = window_mean[serial] = window_mean.get(serial, [])
    points.append(position)

    if len(points) > WINDOW_MEAN_SIZE:
        points = window_mean[serial] = points[-WINDOW_MEAN_SIZE:]

    # Initial case of len(points) < SAMPLE_AVG ignored
    return position_mean(points)


hysteresis_prev = {}


def position_hysteresis(serial, position):
    if HYSTERESIS_ENABLE is not True or not position:
        return position

    prev = hysteresis_prev.get(serial, None)

    if prev and distance(position, prev) < DELTA_THRESHOLD:
        if HYSTERESIS_REPEAT:
            return prev

        return None  # Alternatively, continue reporting the same position

    hysteresis_prev[serial] = position
    return position


def transform_position(position, origin=(0, 0, 0)):
    return [(a * b) - c for a, b, c in zip(position, DIRECTION, origin)]


smooth = {}
SMOOTH_COEFF_COUNT = len(SMOOTH_COEFF[0])
SMOOTH_ORDER = len(SMOOTH_COEFF)


def position_smooth(serial, position):
    if not SMOOTH_ENABLE or not position:
        return position

    if serial not in smooth:
        smooth[serial] = [[position] * SMOOTH_COEFF_COUNT] * SMOOTH_ORDER

    smooth_order_prev = position

    for lowpass, coeff in zip(smooth[serial], SMOOTH_COEFF):
        lowpass.append(smooth_order_prev)

        if len(lowpass) > SMOOTH_COEFF_COUNT:
            lowpass[:] = lowpass[-SMOOTH_COEFF_COUNT:]

        smooth_order_prev = lowpass[-1] = [sum(p*c for p, c in zip(p, coeff)) for p in zip(*lowpass)]

    return smooth_order_prev


lowpass_o1 = {}
lowpass_o2 = {}


def position_smooth_old(serial, position):
    """Brown's double linear exponential smoothing.

    Same as the generic method above save for the final calculation. (So, more than a lowpass?)
    TODO: Determine if this can be made generic as well.
    """
    if not SMOOTH_ENABLE or not position:
        return position

    if serial not in lowpass_o1:
        lowpass_o1[serial] = position
        lowpass_o2[serial] = position

    # note that alpha is the second coefficient in the first pass and the first coefficient in the
    # second pass, unlike the chaining method done above.
    lowpass_o1[serial] = [lp1 * 0.99 + p * 0.01 for lp1, p in zip(lowpass_o1[serial], position)]
    lowpass_o2[serial] = [lp2 * 0.002 + lp1 * 0.998 for lp2, lp1 in zip(lowpass_o2[serial], lowpass_o1[serial])]

    return [2 * lp1 - lp2 for lp2, lp1 in zip(lowpass_o2[serial], lowpass_o1[serial])]


def distance(p1, p2):
    return math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2 + (p2[2] - p1[2])**2)


def planar_distance(p1, p2):
    return math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)


def position_mean(points):
    return [float(sum(p)) / len(p) for p in zip(*points)]


def position_median(points):
    return [median(p) for p in zip(*points)]


def median(arr):
    sarr = sorted(arr)
    count = len(sarr)

    if not count % 2:
        return (sarr[count / 2] + sarr[count / 2 - 1]) / 2.0

    return sarr[count / 2]


def osc_position(serial, position):
    return osc_message("/{}/position".format(serial), *position)


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
        position = position_window_mean(serial, position)
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

    cleanup.install(lambda: os._exit(0))

    print(fwd)
    fwd.start()
    fwd.join()


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
