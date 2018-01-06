from __future__ import print_function
import sys
import os
import csv
import argparse
import traceback
import re
import time
import sched
import wand
import trigger

lowpass_o1 = {}
lowpass_o2 = {}


def position_smooth(serial, position):
    if serial not in lowpass_o1:
        lowpass_o1[serial] = [0, 0, 0]
        lowpass_o2[serial] = [0, 0, 0]

    lowpass_o1[serial] = [lp1 * 0.005 + p * 0.995 for lp1, p in zip(lowpass_o1[serial], position)]
    lowpass_o2[serial] = [lp2 * 0.005 + lp1 * 0.995 for lp2, lp1 in zip(lowpass_o2[serial], lowpass_o1[serial])]

    return lowpass_o2[serial]


def position_smooth_dphony(serial, position):
    if serial not in lowpass_o1:
        lowpass_o1[serial] = [0, 0, 0]
        lowpass_o2[serial] = [0, 0, 0]

    lowpass_o1[serial] = [lp1 * 0.4 + p * 0.6 for lp1, p in zip(lowpass_o1[serial], position)]
    lowpass_o2[serial] = [lp2 * 0.4 + lp1 * 0.6 for lp2, lp1 in zip(lowpass_o2[serial], lowpass_o1[serial])]

    return lowpass_o2[serial]


def handle_data_dancio(ts, serial, name, origin, data):
    position = data[2:5]

    if name in ("dancer/left-wrist", "dancer/right-wrist", "dancer/wand"):
        position = position_smooth(serial, position)
        dist, vec = wand.calculate_pointing(name, position)
        if vec is not None:
            print(ts, dist, vec)


dphony_out_files = {}
dphony_out_folder = time.strftime("replay-%m%d.%H%M")


def handle_data_dphony(ts, serial, data):
    if serial not in dphony_out_files:
        dphony_out_files[serial] = file(os.path.join(dphony_out_folder, serial + ".csv"), 'w')
        dphony_out_files[serial].write("ts,z,sz\n")

    position = data[-3:]
    position_smoothed = position_smooth_dphony(serial, position)
    z, sz = position[2], position_smoothed[2]

    dphony_out_files[serial].write("{},{},{}\n".format(ts, z, sz))
    dphony_out_files[serial].flush()


def main(args):
    data = {}

    for path in os.listdir(args.folder):
        if path.endswith('.csv'):
            serial = os.path.basename(path).rsplit('.', 1)[0]
            lines = file(os.path.join(args.folder, path), 'r').readlines()
            header = lines[0]
            lines = lines[1:]

            if args.mode is "dancio":
                name, origin = re.match("^(.*)@\((.*)\):", header).groups()
                origin = [float(p) for p in origin.split(',')]
                data[serial] = (name, origin, lines)
            else:
                data[serial] = lines

    sch = sched.scheduler(time.time, time.sleep)

    if args.mode is "dancio":
        for serial, (name, origin, lines) in data.items():
            for line in lines:
                ts, data = line.split(',', 1)
                ts = float(ts)

                if ts >= args.start:
                    ts_actual = ts
                    ts -= args.start
                    data = [float(p) for p in data.split(',')]

                    if not args.bulk:
                        sch.enter(ts, 1, handle_data_dancio, (ts_actual, serial, name, origin, data))
                    else:
                        handle_data_dancio(ts_actual, serial, name, origin, data)
    else:
        global dphony_out_folder
        dphony_out_folder = os.path.join(params.folder, dphony_out_folder)

        try:
            os.makedirs(dphony_out_folder)
        except OSError as exc:
            if exc.errno != 17:
                raise

        for serial, lines in data.items():
            for line in lines:
                ts, data = line.split(',', 1)
                ts = float(ts)

                if ts >= args.start:
                    ts_actual = ts
                    ts -= args.start
                    data = [float(p) for p in data.split(',')]

                    if not args.bulk:
                        sch.enter(ts, 1, handle_data_dphony, (ts_actual, serial, data))
                    else:
                        handle_data_dphony(ts_actual, serial, data)

    if not args.bulk:
        sch.run()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('mode', metavar='MODE', choices=['dancio', 'dphony'], help='parsing mode')
    parser.add_argument('folder', metavar='FOLDER', help='log folder to read from')
    parser.add_argument('-s', '--start', metavar='START', default=0, type=float, help='time index to start from')
    parser.add_argument('-B', '--bulk', action='store_true', help='run all at once')

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
