def match_timestamps(base_stamps, other_stamps):
    # try to find the cloest timestamp from other stamps for each base_stamp
    start = 0
    matches = []
    for base_stamp in base_stamps:
        while start + 1 < len(base_stamps) and other_stamps[start + 1] < base_stamp:
            start += 1
        diff1 = abs(base_stamp - other_stamps[start])
        diff2 = abs(base_stamp - other_stamps[start+1])
        if diff1 < diff2:
            pass
        else:
            start += 1
        matches.append(start)
    return matches
