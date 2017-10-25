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

NOTE_AXIS_MAP = {
    0.0: 60,
    0.3: 62,
    0.6: 64,
    0.9: 65,
    1.2: 67,
    1.5: 69,
    1.8: 71,
    2.1: 72
}

note_info = {}


def handle_position(serial, position):
    global note_info

    if serial not in note_info:
        note_info[serial] = None

    if note_info[serial] is None and (position[0] < 0):
        note_info[serial] = map_note(position[1])
        print("note: {} ON".format(note_info[serial]))
        return osc_midi(8, MIDI_EVENT_NOTE_ON, note_info[serial], 127)
    elif note_info[serial] is not None and (position[0] > 0):
        note = note_info[serial]
        note_info[serial] = None
        print("note: {} OFF".format(note))
        return osc_midi(8, MIDI_EVENT_NOTE_OFF, note, 0)
    elif note_info[serial] is not None:
        note_prev = note_info[serial]
        note = map_note(position[1])
        if note != note_prev:
            note_info[serial] = note
            print("note: {} -> {}".format(note_prev, note))
            return osc_midi(8, MIDI_EVENT_NOTE_OFF, note_prev, 0), osc_midi(8, MIDI_EVENT_NOTE_ON, note, 127)
        else:
            value = min(127, int(round(abs(position[0])/1.5 * 127)))
            print("cc/{}".format(value))
            return osc_midi(8, MIDI_EVENT_CONTROL_CHANGE, 7, value)

    return None


def map_note(lateral_position):
    note = None
    thresholds = list(reversed(sorted(NOTE_AXIS_MAP.keys())))

    for threshold in thresholds:
        if lateral_position >= threshold:
            return NOTE_AXIS_MAP[threshold]

    return NOTE_AXIS_MAP[thresholds[-1]]


def osc_midi(channel, event, p1, p2):
    # return OSC.OSCMessage("/midi/{}".format(channel), [event, p1, p2]).getBinary()

    if event == MIDI_EVENT_CONTROL_CHANGE:
        return OSC.OSCMessage("/midicc", [channel, p1, p2]).getBinary()
    else:
        return OSC.OSCMessage("/midi", [channel, event, p1, p2]).getBinary()


def display_position(serial, position):
    print("{:08X}: {}".format(serial, " ".join(str(p).rjust(24) for p in position)))


def main(args):
    if not args.out_port:
        args.out_port = args.port

    handler = parse.parse_dcc

    if args.lcm:
        handler = parse.parse_lcm

    if args.debug:
        position_handler = display_position
    else:
        position_handler = handle_position

    fwd = forward.Forward(
        args.input, args.port, args.out, args.out_port,
        iface=args.iface,
        verbose=args.verbose,
        handler=lambda data: handler(data, position_handler)
    )

    print(fwd)
    fwd.start()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-I', '--iface', metavar='ADDR', required=False, help='source interface address (NOT name!)')
    parser.add_argument('-i', '--input', metavar='ADDR', required=True, help='source address')
    parser.add_argument('-p', '--port', metavar='PORT', type=int, required=True, help='source port')
    parser.add_argument('-o', '--out', metavar='ADDR', required=True, help='destination address')
    parser.add_argument('-P', '--out-port', metavar='PORT', type=int, help='destination port')
    parser.add_argument('-l', '--lcm', action='store_true', help='use lcm protocol for input')
    parser.add_argument('-v', '--verbose', action='store_true', help='verbose output')
    parser.add_argument('-D', '--debug', action='store_true', help='debug mode')

    try:
        main(parser.parse_args())

    except KeyboardInterrupt:
        pass

    except SystemExit:
        raise

    except:
        traceback.print_exc()
        sys.exit(1)

    sys.exit(0)
