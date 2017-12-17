sensor_pos = {"left-wrist": (None, None, None),
              "right-wrist": (None, None, None),
              "wand": (None, None, None)}

def calculate_pointing(name, position):

    if "left-wrist" in name:
        sensor_pos["left-wrist"] = (position[0], position[1], position[2])

    elif "right-wrist" in name:
        sensor_pos["right-wrist"] = (position[0], position[1], position[2])

    elif "wand" in name:
        sensor_pos["wand"] = (position[0], position[1], position[2])


    if sensor_pos["left-wrist"][2] != None and sensor_pos["right-wrist"][2] != None and sensor_pos["wand"][2] != None:
        wrist_dist = distance(sensor_pos["left-wrist"], sensor_pos["right-wrist"])

        if wrist_dist <= 0.0889:
            wrist_avg = ((sensor_pos["left-wrist"][0] + sensor_pos["right-wrist"][0]) / 2, (sensor_pos["left-wrist"][1] + sensor_pos["right-wrist"][1] / 2), (sensor_pos["left-wrist"][2] + sensor_pos["right-wrist"][2] / 2))
            wand_vect = (sensor_pos["wand"][0] - wrist_avg[0], sensor_pos["wand"][1] - wrist_avg[1], sensor_pos["wand"][2] - wrist_avg[2])
            return wand_vect

    else:
        return None

def distance(p1, p2):
    return math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2 + (p2[2] - p1[2])**2)
