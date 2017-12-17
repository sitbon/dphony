sensor_pos = {}

def calculate_pointing(name, position):
    global sensor_pos
    sensor_pos[name] = position

    if len(sensor_pos) != 3:
        return

    left_wrist_pos = sensor_pos["dancer/left-wrist"]
    right_wrist_pos = sensor_pos["dancer/right-wrist"]
    wand_pos = sensor_pos["dancer/wand"]

    # calculate distance
    wrist_dist = distance(left_wrist_pos, right_wrist_pos)

    if wrist_dist <= 0.0889:
        wrist_avg = ((left_wrist_pos[0] + right_wrist_pos[0]) / 2, ((left_wrist_pos[1] + right_wrist_pos[1]) / 2), ((left_wrist_pos[2] + right_wrist_pos[2]) / 2))
        wand_vect = (wand_pos[0] - wrist_avg[0], wand_pos[1] - wrist_avg[1], wand_pos[2] - wrist_avg[2])
        return wand_vect
