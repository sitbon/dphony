import time

prev_pos = {}
prev_ts = {}
prev_v = {}

lowpass_o1 = {}
lowpass_o2 = {}


def is_trigger(velocity):
    return velocity < -0.3


def velocity_update(now, serial, position):

    if serial not in prev_pos:
        prev_pos[serial] = tuple(position)
        prev_ts[serial] = now
        lowpass_o1[serial] = 0
        lowpass_o2[serial] = 0
        return 0, 0

    velocity = (position[2] - prev_pos[serial][2]) / (now - prev_ts[serial])

    prev_pos[serial] = position
    prev_ts[serial] = now

    if (velocity > 3) or (velocity < -3):
        if serial not in prev_v:
            return 0, 0

        return prev_v[serial]
    elif serial in prev_v:
        if abs(prev_v[serial][0] - velocity) > 0.5:
            return prev_v[serial]

    # if velocity > 3.0:
    #     velocity = 3.0
    # elif velocity < -3.0:
    #     velocity = -3.0

    lowpass_o1[serial] = lowpass_o1[serial] * 0.7 + velocity * 0.3
    lowpass_o2[serial] = lowpass_o2[serial] * 0.6 + lowpass_o1[serial] * 0.4

    prev_v[serial] = velocity, lowpass_o2[serial]

    return velocity, lowpass_o2[serial]


zwin = {}


def zwin_trigger(serial):
    zmin_idx, zmin = min(enumerate(zwin[serial]), key=lambda p: p[1])
    zmax_idx, zmax = max(enumerate(zwin[serial]), key=lambda p: p[1])
    dz = zmax - zmin

    if zmax_idx < zmin_idx and zmax >= 2 and zmin <= 1.7 and dz >= 0.7:
        return True

    return False


def zwin_update(serial, z):
    if serial not in zwin:
        zwin[serial] = []

    zwin[serial].append(z)
    zwin[serial] = zwin[serial][-8:]
