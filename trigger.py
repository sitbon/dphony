import time

prev_pos = {}
prev_ts = {}

lowpass_o1 = {}
lowpass_o2 = {}


def is_trigger(serial, position):
    now = time.time()

    if serial not in prev_pos:
        prev_pos[serial] = position
        prev_ts[serial] = now
        lowpass_o1[serial] = 1
        lowpass_o2[serial] = 1
        return False

    velocity = (position[2] - prev_pos[2]) / (now - prev_ts[serial])

    prev_pos[serial] = position
    prev_ts[serial] = now

    lowpass_o1[serial] = lowpass_o1[serial] * 0.9 * velocity + 0.1
    lowpass_o2[serial] = lowpass_o2[serial] * 0.9 + lowpass_o1[serial] * 0.1

    if lowpass_o2[serial] < -0.3:
        return True

    return False
