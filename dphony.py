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

seq_prev = 0
positions = {}

MIDI_EVENT_NOTE_OFF = 0x80
MIDI_EVENT_NOTE_ON = 0x90
MIDI_EVENT_AFTERTOUCH = 0xA0
MIDI_EVENT_CONTROL_CHANGE = 0xB0
MIDI_EVENT_PROGRAM_CHANGE = 0xC0
MIDI_EVENT_CHANNEL_PRESSURE = 0xD0
MIDI_EVENT_PITCH_BEND_CHANGE = 0xE0


def osc_midi(channel, event, p1, p2):
    # return OSC.OSCMessage("/midi/{}".format(channel), [event, p1, p2]).getBinary()

    if event == MIDI_EVENT_CONTROL_CHANGE:
        return OSC.OSCMessage("/midicc", [channel, p1, p2]).getBinary()
    else:
        return OSC.OSCMessage("/midi", [channel, event, p1, p2]).getBinary()


note_on = {}


def map_note(lateral_position):
    if lateral_position <= 0.25:
        return 60
    elif lateral_position <= 0.5:
        return 62
    elif lateral_position <= 0.75:
        return 64
    elif lateral_position <= 1.0:
        return 68
    elif lateral_position <= 1.25:
        return 70
    elif lateral_position <= 1.5:
        return 72
    elif lateral_position <= 1.75:
        return 74
    elif lateral_position <= 2.0:
        return 76
    elif lateral_position <= 2.25:
        return 78
    else:
        return 80


def handle_position(serial, position):
    # if serial not in positions:
    #     positions[serial] = position
    #     return None

    # position_prev = positions[serial]
    # positions[serial] = position

    global note_on

    if serial not in note_on:
        note_on[serial] = None

    if note_on[serial] is None and (position[0] < 0):
        note_on[serial] = map_note(position[1])
        print("note on")
        return osc_midi(8, MIDI_EVENT_NOTE_ON, note_on[serial], 127)
    elif note_on[serial] is not None and (position[0] > 0):
        note = note_on[serial]
        note_on[serial] = None
        print("note off")
        return osc_midi(8, MIDI_EVENT_NOTE_OFF, note, 0)
    elif note_on[serial] is not None:
        note_prev = note_on[serial]
        note = map_note(position[1])
        if note != note_prev:
            note_on[serial] = note
            print("note: {} -> {}".format(note_prev, note))
            return osc_midi(8, MIDI_EVENT_NOTE_OFF, note_prev, 0), osc_midi(8, MIDI_EVENT_NOTE_ON, note, 127)
        else:
            value = min(127, int(round(abs(position[0])/1.5 * 127)))
            print("cc/{}".format(value))
            return osc_midi(8, MIDI_EVENT_CONTROL_CHANGE, 7, value)

    return None


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
