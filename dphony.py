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
import math
import cleanup
import trigger

MIDI_EVENT_NOTE_OFF = 0x80
MIDI_EVENT_NOTE_ON = 0x90
MIDI_EVENT_AFTERTOUCH = 0xA0
MIDI_EVENT_CONTROL_CHANGE = 0xB0
MIDI_EVENT_PROGRAM_CHANGE = 0xC0
MIDI_EVENT_CHANNEL_PRESSURE = 0xD0
MIDI_EVENT_PITCH_BEND_CHANGE = 0xE0

NOTE_BASE = 60

# +x == stage right
# +y == downstage
NOTE_AXIS_MAP_DCC = {
    -10: None,

    # -23: -40,
    # -22: -38,
    # -21: -36,
    # -20: -35,
    # -19: -33,
    # -18: -31,
    # -17: -30,
    # -16: -28,
    # -15: -26,
    # -14: -24,
    # -13: -23,
    # -12: -21,
    # -11: -19,
    # -10: -18,
    -9: -16,
    -8: -14,
    -7: -12,
    -6: -11,
    -5: -9,
    -4: -7,
    -3: -5,
    -2: -4,
    -1: -2,
    0: 0,
    1: 2,
    2: 4,
    3: 5,
    4: 7,
    5: 9,
    6: 11,
    7: 12,
    8: 14,
    9: 16,
    # 10: 18,
    # 11: 19,
    # 12: 21,
    # 13: 23,
    # 14: 24,
    # 15: 26,
    # 16: 28,
    # 17: 30,
    # 18: 31,
    # 19: 33,
    # 20: 35,
    # 21: 36,
    # 22: 38,
    # 23: 40,

    10: None
}

TRIG_AXIS_MAP_DCC = (0, 2.4)

pre_trig = {}

note_info = {}
note_last = {}

log_files = {}

uwb_rotation_angle = math.radians(225.73)
uwb_origin = (-0.25, -25.59, -14.85)  # origin for Cow Palace


def transform_position(position):

    inx, iny, inz = position
    crot = math.cos(uwb_rotation_angle)
    srot = math.sin(uwb_rotation_angle)
    rx = inx * crot - iny * srot
    ry = inx * srot + iny * crot

    return rx - uwb_origin[0], ry - uwb_origin[1], inz - uwb_origin[2]


def handle_position_music(serial, position):

    position = transform_position(position)

    if params.log:
        log_position(serial, position)

    if serial not in note_info:
        note_info[serial] = None

    if note_info[serial] is None:

        if trigger.is_trigger(serial, position):
            note_info[serial] = map_note_dcc(position[1])

            if (note_info[serial] is not None) and (not note_last_block(serial)):
                print("[{:08X}] note: {} ON".format(serial, note_info[serial]))
                return osc_midi(serial, MIDI_EVENT_NOTE_ON, note_info[serial], 127)

    elif (note_info[serial] is not None) and not trigger.is_trigger(serial, position):
        note = note_info[serial]
        note_info[serial] = None
        pre_trig[serial] = False

        print("[{:08X}] note: {} OFF".format(serial, note))
        return  # osc_midi(serial, MIDI_EVENT_NOTE_OFF, note, 0)

    return None


def handle_position(serial, position):
    position = transform_position(position)

    if params.log:
        log_position(serial, position)

    return osc_position(serial, position)


def note_last_block(serial):
    now = time.time()
    block = False

    if (serial in note_last) and (now - note_last[serial]) <= 0.500:
            block = True

    note_last[serial] = now
    return block


def map_note_dcc(lateral_position):
    return map_note(NOTE_AXIS_MAP_DCC, lateral_position)


def map_note_lcm(lateral_position):
    return map_note(NOTE_AXIS_MAP_LCM, lateral_position)


def map_note(note_map, lateral_position):
    thresholds = list(reversed(sorted(note_map.keys())))

    for threshold in thresholds:
        if lateral_position >= threshold:
            return NOTE_BASE + note_map[threshold]

    return NOTE_BASE + note_map[thresholds[-1]]


def osc_midi(serial, event, p1, p2):
    # format: /drone [event, note, value]
    return osc_message("/midi/drone", event, p1, p2)


def osc_position(serial, position):
    return osc_message("/position/drone/{:08X}".format(serial), position)


def osc_message(path, *data):
    return OSC.OSCMessage(path, data).getBinary()


def display_position(serial, position):
    # position = transform_position(position)
    print("{:08X}: {}".format(serial, " ".join(str(p).rjust(12) for p in position)))


def log_init():
    try:
        os.makedirs(params.log)
    except OSError as exc:
        if exc.errno != 17:
            raise

    global log_start
    log_start = time.time()


def log_position(serial, position):
    fil = log_files.get(serial, None)

    if fil is None:
        fil = log_files[serial] = file(os.path.join(params.log, "{:08X}.csv".format(serial)), "w")

    elapsed = time.time() - log_start
    line = "{},{},{},{}\n".format(elapsed, *position)

    fil.write(line)
    fil.flush()


def log_close_files():
    for fil in log_files.values():
        fil.flush()
        fil.close()

    log_files.clear()


def main(args):
    if not args.out_port:
        args.out_port = args.port

    handler = parse.parse_dcc

    if args.debug:
        position_handler = display_position
    else:
        if args.music:
            position_handler = handle_position_music
        else:
            position_handler = handle_position

    fwd = forward.Forward(
        args.input, args.port, args.out, args.out_port,
        iface=args.iface,
        iface_out=args.out_iface,
        verbose=args.verbose,
        handler=lambda data: handler(data, position_handler)
    )

    if params.log:
        log_init()

    cleanup.install(lambda: (log_close_files(), os._exit(0)))

    print(fwd)
    fwd.start()


if __name__ == '__main__':
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
    parser.add_argument('-l', '--log', metavar='LOGDIR', help='log positions to a folder, one file per drone')

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
