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
import wand


MIDI_EVENT_NOTE_OFF = 0x80
MIDI_EVENT_NOTE_ON = 0x90
MIDI_EVENT_AFTERTOUCH = 0xA0
MIDI_EVENT_CONTROL_CHANGE = 0xB0
MIDI_EVENT_PROGRAM_CHANGE = 0xC0
MIDI_EVENT_CHANNEL_PRESSURE = 0xD0
MIDI_EVENT_PITCH_BEND_CHANGE = 0xE0

NOTE_BASE = 0

ORIGIN_DEFAULT = (-6.86, -12, 0)
DIRECTION = (-1, -1, 1)

NOTE_AXIS_MAP_CDP = {
    0.3 * -1:   None,

    0.3 * 0:    36,
    0.3 * 1:    38,
    0.3 * 2:    40,
    0.3 * 3:    41,
    0.3 * 4:    43,
    0.3 * 5:    45,
    0.3 * 6:    47,
    0.3 * 7:    48,
    0.3 * 8:    50,
    0.3 * 9:    52,
    0.3 * 10:   53,
    0.3 * 11:   55,
    0.3 * 12:   57,
    0.3 * 13:   59,
    0.3 * 14:   60,
    0.3 * 15:   62,
    0.3 * 16:   64,
    0.3 * 17:   65,
    0.3 * 18:   67,
    0.3 * 19:   69,
    0.3 * 20:   71,
    0.3 * 21:   72,
    0.3 * 22:   74,
    0.3 * 23:   76,
    0.3 * 24:   77,
    0.3 * 25:   79,
    0.3 * 26:   81,
    0.3 * 27:   83,
    0.3 * 28:   84,
    0.3 * 29:   86,
    0.3 * 30:   88,
    0.3 * 31:   89,
    0.3 * 32:   91,
    0.3 * 33:   93,
    0.3 * 34:   95,
    0.3 * 35:   96,

    0.3 * 36:   None,
}

DEVICE_FILTER_CDP = {

    0x06021373: ("pianist/sergio/right", (-6.85, -12.1, 0)),
    0x06021394: ("pianist/sergio/left", (-6.86, -12.4, 0)),
    0x06021368: ("pianist/sergio/spare", (-6.88, -12.3, 0)),

    0x06021356: ("pianist/kevin/right", (-6.84, -12.4, 0)),
    0x0602139F: ("pianist/kevin/left", (-6.8, -12.25, 0)),
    0x06021351: ("pianist/kevin/spare", (-6.8, -12.2, 0)),

    0x06021345: ("pianist/angie/right", (-6.84, -12.3, 0)),
    0x06021379: ("pianist/angie/left", (-6.86, -12.4, 0)),
    0x06021356: ("pianist/angie/spare", ORIGIN_DEFAULT),

    0x0602135C: ("dancer/left-wrist", ORIGIN_DEFAULT),
    0x06021344: ("dancer/right-wrist", ORIGIN_DEFAULT),
    0x06021349: ("dancer/right-ankle", ORIGIN_DEFAULT),
    0x06021395: ("dancer/left-ankle", ORIGIN_DEFAULT),
    0x06021367: ("dancer/wand", ORIGIN_DEFAULT),
    0x0602134F: ("dancer/wand", ORIGIN_DEFAULT),
    0x06021387: ("dancer/spare", ORIGIN_DEFAULT),

    0x06021340: ("tramp/right", ORIGIN_DEFAULT),
    0x0602137E: ("tramp/left", ORIGIN_DEFAULT),
    0x060213A2: ("tramp/spare", ORIGIN_DEFAULT),

    0x06021359: ("none/spare", ORIGIN_DEFAULT),
    0x0602136A: ("none/spare", ORIGIN_DEFAULT),

}


note_info = {}
note_last = {}

cdp_pos = {}
cdp_pos_raw = {}
cdp_dedup = {}
cdp_reject = {}

log_files = {}


def handle_position_cdp_music(serial, position, user_data):
    if serial not in DEVICE_FILTER_CDP:
        return

    name, origin = DEVICE_FILTER_CDP[serial]

    result = []

    if position is not None:
        position_raw = [(a * b) - c for a, b, c in zip(position, DIRECTION, origin)]
        position = human_filter_update(serial, position_raw)

        if position is not None and "tramp" not in name:
            cdp_pos_raw[serial] = position_raw
            cdp_pos[serial] = position

    if user_data is None or not len(user_data) or len(user_data) < 15:
        if params.log and (position is not None):
            log_position(serial, position_raw, position, False, 0)

        if len(result):
            return result
        return

    typ = struct.unpack("<B", user_data[:1])[0]

    user_data = user_data[1:]
    has_event = False
    event_note = 0

    if typ == 4:
        sequence, mask, w_ang, v_ang, h_ang, tap_d, tap_v, omni_d, omni_v, shake_d, shake_v, shake_du, lasso_d, lasso_v = struct.unpack(
            '<BBbbbbbbbbbbbb', user_data[:14])

        if serial in cdp_dedup and cdp_dedup[serial] == sequence:
            return
        else:
            cdp_dedup[serial] = sequence

        if mask & 2:
            has_event = 1

        if (mask & 2) and (serial in cdp_pos) and ("dancer" not in name):
            pos = cdp_pos[serial]
            note = map_note_cdp(pos[0])

            if (note is not None) and (not note_last_block(serial)):
                event_note = note
                result.append(osc_midi_note_on(name, note))

                if params.verbose:
                    print("{:08X}: note: {}".format(serial, note))

    if params.log and (serial in cdp_pos):
        log_position(serial, cdp_pos_raw[serial], cdp_pos[serial], has_event, event_note)

    if len(result):
        return result


def handle_position_cdp(serial, position, user_data):
    if serial not in DEVICE_FILTER_CDP:
        return

    name, origin = DEVICE_FILTER_CDP[serial]

    result = []

    if position is not None:
        position = [a * b for a, b in zip(position, DIRECTION)]
        # position = human_filter_update(serial, position)

        if position is not None:  # and not reject_position(serial, position):
            if name in ("dancer/left-wrist", "dancer/right-wrist", "dancer/wand"):
                vec = wand.calculate_pointing(name, position)

                if vec is not None:
                    result.append(osc_gesture("pointing", "dancer", *vec))

            if "tramp" not in name:
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


hf_thr = (0.25 / 100, 1.0 / 100)
hf_cleanx = {}


def human_filter_update(serial, position):
    x = position[0]

    if serial not in hf_cleanx:
        hf_cleanx[serial] = (1, x, 0)

    val, xv, tv = hf_cleanx[serial]

    tv += 1

    if tv > 300:
        tv = 300

    delta = x - xv
    adel = abs(delta)

    thn = adel / tv

    if thn < hf_thr[0]:
        alp = 1
        val = 1
        tv = 0
    else:
        if thn >= hf_thr[1]:
            alp = 0
            val = 0
        else:
            alp = (hf_thr[1] - thn) / (hf_thr[1] - hf_thr[0])

    xv = xv + alp * delta

    hf_cleanx[serial] = (val, xv, tv)

    return xv, position[1], position[2]


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
    return osc_message("/position/{}".format(serial), *position)


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
            position = [(a - b) * c for a, b, c in zip(position, ORIGIN_DEFAULT, DIRECTION)]

        pos = " ".join(str(round(p, 3)).rjust(12) for p in position)

    # print("{:08X}: {}\t{}".format(serial, pos, data))
    print("{:08X}: {}".format(serial, pos))


def log_init():
    try:
        os.makedirs(params.log)
    except OSError as exc:
        if exc.errno != 17:
            raise

    global log_start
    log_start = time.time()


def log_position(serial, position_raw, position, hit_event, event_note):
    fil = log_files.get(serial, None)

    if fil is None:
        fil = log_files[serial] = file(os.path.join(params.log, "{:08X}.csv".format(serial)), "w")
        if serial in DEVICE_FILTER_CDP:
            name, origin = DEVICE_FILTER_CDP[serial]
            fil.write("{}@({},{},{}): time, hit, note, rx, ry, rz, x, y, z\n".format(name, *origin))

    elapsed = time.time() - log_start
    line = "{},{},{},{},{},{},{},{},{}\n".format(
        elapsed, 1 if hit_event else 0, event_note, *(tuple(position_raw) + tuple(position))
    )

    fil.write(line)
    fil.flush()


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

    if params.log:
        log_init()

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
    parser.add_argument('-l', '--log', metavar='LOGDIR', help='log positions to a folder, one file per drone')


    try:
        params = parser.parse_args()

        if params.log and not params.music:
            raise ValueError, "Cannot log in video mode"

        main(params)

    except KeyboardInterrupt:
        pass

    except SystemExit:
        raise

    except:
        traceback.print_exc()
        sys.exit(1)

    sys.exit(0)
