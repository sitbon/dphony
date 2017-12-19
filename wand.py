import math
import time
sensor_pos = {}
threshold = None
wrist_data = []
on = False
on_pre = None

wrist_dist_lp1 = 0
wrist_dist_lp2 = 0

def calculate_pointing(name, position):
    global on
    global on_pre
    global sensor_pos
    global threshold
    global wrist_data
    global wrist_dist_lp1
    global wrist_dist_lp2
    sensor_pos[name] = position

    if len(sensor_pos) != 3:
        return

    left_wrist_pos = sensor_pos["dancer/left-wrist"]
    right_wrist_pos = sensor_pos["dancer/right-wrist"]
    wand_pos = sensor_pos["dancer/wand"]

    # calculate distance
    wrist_dist = distance(left_wrist_pos, right_wrist_pos)

    wrist_dist_lp1 = wrist_dist_lp1 * 0.9 + wrist_dist * 0.1
    wrist_dist_lp2 = wrist_dist_lp2 * 0.9 + wrist_dist_lp1 * 0.1
    wrist_dist = wrist_dist_lp2

    if not on:
        # print(wrist_dist)
        if wrist_dist <= 0.3:
            if on_pre is None:
                on_pre = time.time()
            else:
                elapsed = time.time() - on_pre
                if elapsed >= 0.3:
                    on = True
        elif on_pre is not None:
            on_pre = None
    else:
        if wrist_dist > 0.5 and (time.time() - on_pre) >= 3.5:
            on = False
            on_pre = None

    if on:
        wrist_avg = ((left_wrist_pos[0] + right_wrist_pos[0]) / 2, ((left_wrist_pos[1] + right_wrist_pos[1]) / 2),
                     ((left_wrist_pos[2] + right_wrist_pos[2]) / 2))
        wand_vect = (wand_pos[0] - wrist_avg[0], wand_pos[1] - wrist_avg[1], wand_pos[2] - wrist_avg[2])
        return wand_vect


def distance(p1, p2):
    return math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2 + (p2[2] - p1[2])**2)
