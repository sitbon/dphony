import math
import time
sensor_pos = {}
threshold = None
wrist_data = []

def calculate_pointing(name, position):
    global sensor_pos
    global threshold
    global wrist_data
    sensor_pos[name] = position

    if len(sensor_pos) != 3:
        return

    left_wrist_pos = sensor_pos["dancer/left-wrist"]
    right_wrist_pos = sensor_pos["dancer/right-wrist"]
    wand_pos = sensor_pos["dancer/wand"]

    # calculate distance
    wrist_dist = distance(left_wrist_pos, right_wrist_pos)
    wrist_data.append(wrist_dist)

    if len(wrist_data) >= 50:
        wrist_mean = sum(wrist_data) / 50
        del wrist_data[0]

        if wrist_mean > 0.2000:
            threshold = None

        elif wrist_mean <= 0.2000 and threshold and time.time() > threshold:
            wrist_avg = ((left_wrist_pos[0] + right_wrist_pos[0]) / 2, ((left_wrist_pos[1] + right_wrist_pos[1]) / 2), ((left_wrist_pos[2] + right_wrist_pos[2]) / 2))
            wand_vect = (wand_pos[0] - wrist_avg[0], wand_pos[1] - wrist_avg[1], wand_pos[2] - wrist_avg[2])
            #if wand_vect != None:
            print (wand_vect)
            return wand_vect
        elif wrist_mean <= 0.2000 and not threshold:
            threshold = time.time() + 0.5

def distance(p1, p2):
    return math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2 + (p2[2] - p1[2])**2)
