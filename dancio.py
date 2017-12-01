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

NOTE_BASE = 60

NOTE_AXIS_MAP_LCM = {
    0.00: 5,
    0.50: 7,
    1.00: 4,
    1.50: 2,
    2.00: 9,
}

"""
    Z: floor=653 waist=654
    X: front=~0/0.75 back=3
    Y: left=0 right=6.2 middle=~3
"""

NOTE_AXIS_MAP_DCC = {
    3.00: 5,
    3.50: 7,
    4.00: 4,
    4.50: 2,
    5.00: 9,
}

TRIG_AXIS_MAP_DCC = (653, 654)

DEVICE_FILTER_DCC = (0x00020001, )

DEVICE_FILTER_CDP = {
    0x060212E6: "cdp/left_hand",
    0x0602136F: "cdp/left_ankle",
}

DEVICE_FILTER_LCM = {
    0x010201BF: "left_hand",
    0x0102019A: "right_hand",
    0x010201B3: "left_ankle",
    0x010201E9: "right_ankle",
    0x010201F9: "right_ankle",
    0x01020183: "testing",
}

note_info = {}
note_last = {}

cdp_dedup = {}
lcm_dedup = {}


def handle_position_cdp(serial, position, user_data):
    if serial not in DEVICE_FILTER_CDP:
        return

    name = DEVICE_FILTER_CDP[serial]

    result = []

    if position is not None:
        result.append(osc_position(name, position))

    if not len(user_data) or len(user_data) < 12:
        if len(result):
            return result
        return

    whatever, sequence, mask, wrist, angle_vert, angle_horz, \
        dir_tap, vel_tap, dir_omni, vel_omni, dir_shake, vel_shake = \
        struct.unpack("<12B", user_data[:12])

    if serial in cdp_dedup and cdp_dedup[serial] == sequence:
        return
    else:
        cdp_dedup[serial] = sequence

    if params.verbose:
        print("{:08X}: 0x{:02X} 0x{:02X}".format(serial, sequence, mask))

    if mask & 1:
        result.append(osc_wrist(name, wrist, angle_horz, angle_vert))

    if mask & 2:
        result.append(osc_tap(name, dir_tap, vel_tap, angle_horz, angle_vert))

    if mask & 4:
        result.append(osc_omni(name, dir_omni, vel_omni, angle_horz, angle_vert))

    if mask & 8:
        result.append(osc_shake(name, dir_shake, vel_shake, angle_horz, angle_vert))

    if len(result):
        return result


def handle_position_lcm(serial, position, user_data):
    if serial not in DEVICE_FILTER_LCM:
        return

    name = DEVICE_FILTER_LCM[serial]

    result = [osc_position(name, position)]

    if not len(user_data):
        # TODO: dedup?
        return result

    whatever, sequence, mask, wrist, angle_vert, angle_horz, dir_tap, dir_omni, dir_shake,\
        vel_tap, vel_omni, vel_shake = \
        struct.unpack("<12B", user_data[:12])

    if serial in lcm_dedup and lcm_dedup[serial] == sequence:
        return
    else:
        lcm_dedup[serial] = sequence

    if mask & 1:
        result.append(osc_wrist(name, wrist, angle_horz, angle_vert))

    if mask & 2:
        result.append(osc_tap(name, dir_tap, vel_tap, angle_horz, angle_vert))

    if mask & 4:
        result.append(osc_omni(name, dir_omni, vel_omni, angle_horz, angle_vert))

    if mask & 8:
        result.append(osc_shake(name, dir_shake, vel_shake, angle_horz, angle_vert))

    if len(result):
        return result


def note_last_block(serial):
    now = time.time()
    block = False

    if (serial in note_last) and (now - note_last[serial]) <= 0.35:
            block = True

    note_last[serial] = now
    return block


def map_note_dcc(lateral_position):
    return map_note(NOTE_AXIS_MAP_DCC, lateral_position)


def map_note_lcm(lateral_position):
    return map_note(NOTE_AXIS_MAP_LCM, lateral_position)


def map_note(note_map, lateral_position):
    note = None
    thresholds = list(reversed(sorted(note_map.keys())))

    for threshold in thresholds:
        if lateral_position >= threshold:
            return NOTE_BASE + note_map[threshold]

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
    return osc_message("/gesture/dancer/{}/{}".format(serial, gesture), sequence_event, *args)


def osc_position(serial, position):
    return osc_message("/position/dancer/{}".format(serial), position)


def osc_midi(channel, event, p1, p2):
    if event == MIDI_EVENT_CONTROL_CHANGE:
        return osc_message("/midicc", channel, p1, p2)
    else:
        return osc_message("/midi", channel, event, p1, p2)


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
        pos = " ".join(str(round(p, 3)).rjust(12) for p in position)

    print("{:08X}: {}\t{}".format(serial, pos, data))


def main(args):
    if not args.out_port:
        args.out_port = args.port

    handler = parse.parse_cdp

    if args.lcm:
        handler = parse.parse_lcm

    if args.debug:
        position_handler = display_position
    else:
        if args.lcm:
            position_handler = handle_position_lcm
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
    # default: 239.255.0.80  10077
    parser = argparse.ArgumentParser()
    parser.add_argument('-I', '--iface', metavar='ADDR', required=False, help='source interface address (NOT name!)')
    parser.add_argument('-i', '--input', metavar='ADDR', required=True, help='source address')
    parser.add_argument('-p', '--port', metavar='PORT', type=int, required=True, help='source port')
    parser.add_argument('-o', '--out', metavar='ADDR', required=True, help='destination address')
    parser.add_argument('-O', '--out-iface', metavar='ADDR', required=False, help='outgoing iface address (NOT name!)')
    parser.add_argument('-P', '--out-port', metavar='PORT', type=int, help='destination port')
    parser.add_argument('-l', '--lcm', action='store_true', help='use lcm protocol for input')
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
