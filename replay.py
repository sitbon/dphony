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


def handle_data(ts, serial, name, origin, data):
    position = data[2:5]

    if name in ("dancer/left-wrist", "dancer/right-wrist", "dancer/wand"):
        vec = wand.calculate_pointing(name, position)
        if vec is not None:
            print(ts, vec)


def main(args):
    data = {}

    for path in os.listdir(args.folder):
        if path.endswith('.csv'):
            serial = os.path.basename(path).rsplit('.', 1)[0]
            lines = file(os.path.join(args.folder, path), 'r').readlines()
            header = lines[0]
            lines = lines[1:]
            name, origin = re.match("^(.*)@\((.*)\):", header).groups()
            origin = [float(p) for p in origin.split(',')]
            data[serial] = (name, origin, lines)

    sch = sched.scheduler(time.time, time.sleep)

    for serial, (name, origin, lines) in data.items():
        for line in lines:
            ts, data = line.split(',', 1)
            ts = float(ts)
            data = [float(p) for p in data.split(',')]
            sch.enter(ts, 1, handle_data, (ts, serial, name, origin, data))

    sch.run()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('folder', metavar='FOLDER', help='log folder to read from')

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