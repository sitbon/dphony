#!/usr/bin/env python2
"""Drone position to syncphony mapper.
"""
from __future__ import print_function
import os
import sys
import struct
import argparse
import time
import random
import threading
import traceback
import socket
import math

import ipaddress
import forward
import parse
import OSC


MIDI_EVENT_NOTE_OFF = 0x80
MIDI_EVENT_NOTE_ON = 0x90
MIDI_EVENT_AFTERTOUCH = 0xA0
MIDI_EVENT_CONTROL_CHANGE = 0xB0
MIDI_EVENT_PROGRAM_CHANGE = 0xC0
MIDI_EVENT_CHANNEL_PRESSURE = 0xD0
MIDI_EVENT_PITCH_BEND_CHANGE = 0xE0

NOTE_BASE = 41

ORIGIN = (2.895 + 0.6, -6.595, 0.0)
DIRECTION = (-1, -1, 1)

NOTE_AXIS_MAP_CDP = {
    0.3 * 0:    0,
    0.3 * 1:    2,
    0.3 * 2:    4,
    0.3 * 3:    6,
    0.3 * 4:    7,
    0.3 * 5:    9,
    0.3 * 6:    11,
    0.3 * 7:    12,
    0.3 * 8:    14,
    0.3 * 9:    16,
    0.3 * 10:   18,
    0.3 * 11:   19,
    0.3 * 12:   21,
    0.3 * 13:   23,
    0.3 * 14:   24,
    0.3 * 15:   26,
    0.3 * 16:   28,
    0.3 * 17:   30,
    0.3 * 18:   31,
    0.3 * 19:   33,
    0.3 * 20:   35,
    0.3 * 21:   36,
    0.3 * 22:   38,
    0.3 * 23:   40,
}

NOTE_AXIS_MAP_CDP_BLACK = {
    0.3 * 0.0:  1,       # first black key, 0-1.5x
    0.3 * 1.5:  3,       # second, 1.5x-2.5x
    0.3 * 2.5:  5,       # third, 2.5x-3.5x
    0.3 * 3.5:  None,    # nothing, 3.5x-4.5x
    0.3 * 4.5:  8,       # 4: 4.5x-5.5x
    0.3 * 5.5:  10,      # 5: 5.5x-6.5x
    0.3 * 6.5:  None,    # nothing: 6.5x-7.5x

    0.3 * 7.5:  13,
    0.3 * 8.5:  15,
    0.3 * 9.5:  17,
    0.3 * 10.5: None,
    0.3 * 11.5: 20,
    0.3 * 12.5: 22,
    0.3 * 13.5: None,

    0.3 * 14.5: 25,
    0.3 * 15.5: 27,
    0.3 * 16.5: 29,
    0.3 * 17.5: None,
    0.3 * 18.5: 32,
    0.3 * 19.5: 34,
    0.3 * 20.5: None,

    0.3 * 21.5: 37,
    0.3 * 22.5: 39,
}

NOTE_THRESHOLD_CDP_BLACK = 0.75

DEVICE_FILTER_CDP = {

    0x0602139F: "pianist/kevin/left",
    0x06021348: "pianist/kevin/right",
    0x06021394: "pianist/sergio/left",
    0x06021373: "pianist/sergio/right",
    0x06021379: "pianist/angie/left",
    0x06021345: "pianist/angie/right",

    0x06021349: "dancer/right-ankle",
    0x06021395: "dancer/left-ankle",

    0x0602137E: "tramp/left",
    0x06021340: "tramp/right",

}


note_info = {}
note_last = {}

cdp_pos = {}
cdp_dedup = {}
cdp_reject = {}


def handle_position_cdp_music(serial, position, user_data):
    if serial not in DEVICE_FILTER_CDP:
        return

    name = DEVICE_FILTER_CDP[serial]

    result = []

    if position is not None:
        position = [(a - b) * c for a, b, c in zip(position, ORIGIN, DIRECTION)]

        position = median_filter_update(serial, position)

        if position is not None and not reject_position(serial, position):
            result.append(osc_position(name, position))

    if user_data is None or not len(user_data) or len(user_data) < 15:
        if len(result):
            return result
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

        if (mask & 2) and (serial in cdp_pos) and len(cdp_pos[serial]):
            pos = cdp_pos[serial][-1]

            if pos[1] >= NOTE_THRESHOLD_CDP_BLACK:
                note = map_note_cdp_black(pos[0])

                if note is None:
                    note = map_note_cdp(pos[0])
            else:
                note = map_note_cdp(pos[0])

            if not note_last_block(serial):
                result.append(osc_midi_note_on(name, note))

                if params.verbose:
                    print("{:08X}: note: {}".format(serial, note))

    if len(result):
        return result


def handle_position_cdp(serial, position, user_data):
    if serial not in DEVICE_FILTER_CDP:
        return

    name = DEVICE_FILTER_CDP[serial]

    result = []

    if position is not None:
        position = [a * b for a, b in zip(position, DIRECTION)]

        # if not reject_position(serial, position):
        #     cdp_pos[serial] = position
        #     result.append(osc_position(name, position))

        position = median_filter_update(serial, position)

        if position is not None and not reject_position(serial, position):
            result.append(osc_position(name, position))

    if user_data is None or not len(user_data) or len(user_data) < 15:
        if len(result):
            return result
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

        if mask & 1:
            result.append(osc_wrist(name, w_ang, h_ang, v_ang))

        if mask & 2:
            result.append(osc_tap(name, tap_d, tap_v, h_ang, v_ang))

        if mask & 4:
            result.append(osc_omni(name, omni_d, omni_v, h_ang, v_ang))

        if mask & 8:
            result.append(osc_shake(name, shake_d, shake_v, h_ang, v_ang))

    if len(result):
        return result


def median_filter_update(serial, position):
    poss = cdp_pos.setdefault(serial, [])
    poss.append(position)

    if len(poss) < 10:
        return None

    position = median(p[0] for p in poss),\
               median(p[1] for p in poss),\
               median(p[2] for p in poss)

    cdp_pos[serial] = []
    return position


def reject_position(serial, pos):
    reject = False
    poss = cdp_pos.get(serial, [])

    if len(poss):
        pos_prev = poss[-1]
    else:
        pos_prev = None

    if pos_prev is not None:
        dz = abs(pos[2] - pos_prev[2])

        if dz >= 1:
            reject = True
            print("{:08X}: reject dz={}".format(serial, dz))

        if (-1.5 >= pos) or (pos[2] >= 2.5):
            reject = True
            print("{:08X}: reject z={}".format(serial, pos[2]))

        else:
            pdist = planar_distance(pos_prev, pos)

            if (0.1 >= pdist) or (pdist >= 1):
                reject = True

                if pdist >= 1:
                    print("{:08X}: reject large pdist={}".format(serial, pdist))
            else:
                dist = distance(pos_prev, pos)

                if (0.1 >= dist) or (dist >= 1):
                    reject = True

                    if dist >= 1:
                        print("{:08X}: reject large dist={}".format(serial, dist))

    if serial not in cdp_reject:
        cdp_reject[serial] = int(reject)
    else:
        cdp_reject[serial] += int(reject)

        if cdp_reject[serial] > 10:
            cdp_reject[serial] = 0
            reject = False

    return reject


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


def note_last_block(serial):
    # For now, rely on sensor hysteresis
    return False

    now = time.time()
    block = False

    if (serial in note_last) and (now - note_last[serial]) <= 0.10:
            block = True

    note_last[serial] = now  # should be above?
    return block


def map_note_cdp_black(lateral_position):
    return map_note(NOTE_AXIS_MAP_CDP_BLACK, lateral_position)


def map_note_cdp(lateral_position):
    return map_note(NOTE_AXIS_MAP_CDP, lateral_position)


def map_note(note_map, lateral_position):
    note = None
    thresholds = list(reversed(sorted(note_map.keys())))

    for threshold in thresholds:
        if lateral_position >= threshold:
            if note_map[threshold] is None:
                return None
            return NOTE_BASE + note_map[threshold]

    if note_map[thresholds[-1]] is None:
        return None

    return NOTE_BASE + note_map[thresholds[-1]]


sequence_event = 0


def osc_shake(*args):
    return osc_gesture("shake", *args)


def osc_tap(*args):
    return osc_gesture("tap", *args)


def osc_omni(*args):
    return osc_gesture("omni", *args)


def osc_wrist(*args):
    return osc_gesture("wrist", *args)


def osc_gesture(gesture, serial, *args):
    global sequence_event
    sequence_event += 1
    return osc_message("/gesture/{}/{}".format(serial, gesture), sequence_event, *args)


def osc_position(serial, position):
    return osc_message("/position/{}".format(serial), position)


def osc_midi_note_off(serial, note, velocity=0):
    return osc_midi(serial, MIDI_EVENT_NOTE_OFF, note, velocity)


def osc_midi_note_on(serial, note, velocity=127):
    return osc_midi(serial, MIDI_EVENT_NOTE_ON, note, velocity)


def osc_midi(serial, event, p1, p2):
    return osc_message("/midi/dancer", event, p1, p2)


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
        if params.music:
            position = [(a - b) * c for a, b, c in zip(position, ORIGIN, DIRECTION)]

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
        if args.music:
            position_handler = handle_position_cdp_music
        else:
            position_handler = handle_position_cdp

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
    # default video: 239.255.0.80  10077
    # default music: 239.255.0.81  10081
    parser = argparse.ArgumentParser()
    parser.add_argument('-I', '--iface', metavar='ADDR', required=False, help='source interface address (NOT name!)')
    parser.add_argument('-i', '--input', metavar='ADDR', required=True, help='source address')
    parser.add_argument('-p', '--port', metavar='PORT', type=int, required=True, help='source port')
    parser.add_argument('-o', '--out', metavar='ADDR', required=True, help='destination address')
    parser.add_argument('-O', '--out-iface', metavar='ADDR', required=False, help='outgoing iface address (NOT name!)')
    parser.add_argument('-P', '--out-port', metavar='PORT', type=int, help='destination port')
    parser.add_argument('-v', '--verbose', action='store_true', help='verbose output')
    parser.add_argument('-D', '--debug', action='store_true', help='debug mode')
    parser.add_argument('-M', '--music', action='store_true', help='music system mode (default: video system mode)')


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
