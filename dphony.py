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

NOTE_BASE = 0

# +x == stage right
# +y == downstage
NOTE_AXIS_MAP_DCC = {
    -10: None,

    -9: 83,
    -8: 81,
    -7: 79,
    -6: 77,
    -5: 76,
    -4: 74,
    -3: 72,
    -2: 71,
    -1: 69,
    0: 67,
    1: 65,
    2: 64,
    3: 62,
    4: 60,
    5: 59,
    6: 57,
    7: 55,
    8: 53,
    9: 52,

    10: None
}

NOTE_AXIS_MAP_DCC_KEYS = list(reversed(sorted(NOTE_AXIS_MAP_DCC.keys())))

TRIG_AXIS_MAP_DCC = (0, 2.4)

pre_trig = {}

note_info = {}
note_last = {}

log_files = {}

uwb_rotation_angle = math.radians(227.07)
uwb_origin = (-0.16, -28.21, -14.30)  # origin for Park Theater


def transform_position(position):

    inx, iny, inz = position
    crot = math.cos(uwb_rotation_angle)
    srot = math.sin(uwb_rotation_angle)
    rx = inx * crot - iny * srot
    ry = inx * srot + iny * crot

    return rx - uwb_origin[0], ry - uwb_origin[1], inz - uwb_origin[2]


ts_base = {}


def handle_position_music(ts, serial, position):
    if serial not in ts_base:
        ts_base[serial] = ts

    ts -= ts_base[serial]

    position_raw = transform_position(position)
    position = position_raw
    # position = position_smooth(serial, position_raw)

    trigger.zwin_update(serial, position[2])

    if serial not in note_info:
        note_info[serial] = None

    if note_info[serial] is None:

        if trigger.zwin_trigger(serial):
            # print("[{:08X}] trigger v={} y={}".format(serial, velocity, position[1]))
            note_info[serial] = map_note_dcc(position[1])

            if (note_info[serial] is not None) and (not note_last_block(ts, serial)):
                print("[{:08X}] note: {}".format(serial, note_info[serial]))
                return osc_midi(serial, MIDI_EVENT_NOTE_ON, note_info[serial], 127)

    elif note_info[serial] is not None:
        note_info[serial] = None

    return None


def handle_position(ts, serial, position):
    if serial not in ts_base:
        ts_base[serial] = ts

    ts -= ts_base[serial]

    position_raw = transform_position(position)
    position = position_raw
    # position = position_smooth(serial, position_raw)

    if params.log:
        velocityi, velocity = trigger.velocity_update(ts, serial, position)
        log_position(ts, serial, position_raw, position, velocityi, velocity)

    return osc_position(serial, position)


lowpass_o1 = {}
lowpass_o2 = {}


def position_smooth(serial, position):
    if serial not in lowpass_o1:
        lowpass_o1[serial] = list(position)
        lowpass_o2[serial] = list(position)

    lowpass_o1[serial] = [lp1 * 0.7 + p * 0.3 for lp1, p in zip(lowpass_o1[serial], position)]
    lowpass_o2[serial] = [lp2 * 0.5 + lp1 * 0.5 for lp2, lp1 in zip(lowpass_o2[serial], lowpass_o1[serial])]

    return lowpass_o2[serial]


def note_last_block(ts, serial):
    block = False

    if (serial in note_last) and (ts - note_last[serial]) <= 0.5:
            block = True

    note_last[serial] = ts
    return block


def map_note_dcc(lateral_position):
    return map_note(NOTE_AXIS_MAP_DCC, NOTE_AXIS_MAP_DCC_KEYS, lateral_position)


def map_note(note_map, keys, lateral_position):
    thresholds = keys

    for threshold in thresholds:
        if lateral_position >= threshold:
            if note_map[threshold] is None:
                return None
            return NOTE_BASE + note_map[threshold]

    if note_map[thresholds[-1]] is None:
        return None

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


def log_position(ts, serial, position_raw, position, velocityi, velocity):
    fil = log_files.get(serial, None)

    if fil is None:
        fil = log_files[serial] = file(os.path.join(params.log, "{:08X}.csv".format(serial)), "w")
        fil.write("time,atime,ivelocity,velocity,rx,ry,rz,x,y,z\n")

    elapsed = time.time() - log_start
    line = "{},{},{},{},{},{},{},{},{},{}\n".format(ts, elapsed, velocityi, velocity, *(tuple(position_raw) + tuple(position)))

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

        if params.log and params.music:
            raise ValueError, "Cannot log in music mode"

        main(params)

    except KeyboardInterrupt:
        pass

    except SystemExit:
        raise

    except:
        traceback.print_exc()
        sys.exit(1)

    sys.exit(0)
