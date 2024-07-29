import numpy as np
import pathlib

def read_lidar(filename, col_num):
    return np.fromfile(filename, dtype=np.float32).reshape((-1, col_num))

def read_vlp(filename):
    return read_lidar(filename, 4)

def read_sick(filename):
    return read_lidar(filename, 2)

def read_calib(filename):
    with open(filename) as f:
        lines = f.readlines()
        R = np.array(lines[-2][3:].strip().split(), dtype=np.float64).reshape(3, 3)
        T = np.array(lines[-1][3:].strip().split(), dtype=np.float64)
    return R, T

def merge_two_scans(left_scan, right_scan, R_left, T_left, R_right, T_right):
    left_transformed = np.dot(left_scan[:, :3], R_left) + T_left
    left_transformed = np.hstack([left_transformed, left_scan[:, -1:]])
    right_transformed = np.dot(right_scan[:, :3], R_right) + T_right
    right_transformed = np.hstack([right_transformed, right_scan[:, -1:]])
    merged_points = np.vstack((left_transformed, right_transformed))
    return merged_points.astype(np.float32)

def merge_dataset(left_stamps, right_stamps):
    # timestamp is in nanosecond, i.e., 1e-9 second
    # lidar scan frequency is at 10 Hz, i.e., 100 millisecond or 0.1 second
    i = 0
    j = 0 # the file of first stamp does not exist
    matches = []
    while i < len(left_stamps):
        left_stamp = left_stamps[i]
        right_stamp = right_stamps[j]
        if left_stamp < right_stamp:
            while i < len(left_stamps) - 1:
                if left_stamps[i + 1] > right_stamp:
                    break
                else:
                    i += 1
            curr_left = left_stamps[i]
            next_left = left_stamps[i + 1]
            diff1 = right_stamp - curr_left
            diff2 = next_left - right_stamp
            if diff1 < diff2:
                left_stamp = curr_left
                assert diff1 < 5e8
            else:
                left_stamp = next_left
                i += 1
                assert diff2 < 5e8
            assert diff1 < 1e9 and diff2 < 1e9
        elif left_stamp > right_stamp:
            while j < len(right_stamps) - 1:
                if left_stamp < right_stamps[j + 1]:
                    break
                else:
                    j += 1
            curr_right = right_stamps[j]
            next_right = right_stamps[j + 1]
            diff1 = left_stamp - curr_right
            diff2 = next_right - left_stamp
            if diff1 < diff2:
                right_stamp = curr_right
                assert diff1 < 5e8
            else:
                right_stamp = next_right
                j += 1
                assert diff2 < 5e8
            assert diff1 < 1e9 and diff2 < 1e9
        else:
            pass
        matches.append((left_stamp, right_stamp))
        i += 1
        j += 1
    return matches

def read_stamps(filename):
    return np.loadtxt(filename, dtype=np.int64)

def process_vlp(dataroot):
    left_stamps = read_stamps(f'{dataroot}/sensor_data/VLP_left_stamp.csv')
    right_stamps = read_stamps(f'{dataroot}/sensor_data/VLP_right_stamp.csv')
    pathlib.Path(f'{dataroot}/sensor_data/VLP_merged').mkdir(parents=True, exist_ok=True)
    matches = merge_dataset(left_stamps, right_stamps)
    R_left, T_left = read_calib(f'{dataroot}/calibration/Vehicle2LeftVLP.txt')
    print('R_left:')
    print(R_left)
    print('T_left:')
    print(T_left)
    R_right, T_right = read_calib(f'{dataroot}/calibration/Vehicle2RightVLP.txt')
    print('R_right:')
    print(R_right)
    print('T_right:')
    print(T_right)
    with open(f'{dataroot}/sensor_data/VLP_merged_stamp.csv', 'w') as f:
        for match in matches:
            left_scan = read_vlp(f'{dataroot}/sensor_data/VLP_left/{match[0]}.bin')
            right_scan = read_vlp(f'{dataroot}/sensor_data/VLP_right/{match[1]}.bin')
            merged_scan = merge_two_scans(left_scan, right_scan, R_left, T_left, R_right, T_right)
            avg_stamp = (match[0] + match[1]) // 2
            merged_scan.tofile(f'{dataroot}/sensor_data/VLP_merged/{avg_stamp}.bin')
            f.write(f'{avg_stamp}\n')

SICK_ANGLES = np.arange(-5., 185.5, 0.6667)
SIN_SA = np.sin(SICK_ANGLES)
COS_SA = np.cos(SICK_ANGLES)

def sick_2d_2_3d(scan):
    xs = scan[:, 0] * COS_SA
    ys = scan[:, 0] * SIN_SA
    scans = np.stack([xs, ys, np.zeros_like(xs), scan[:, 1]]).T
    return scans

def process_sick(dataroot):
    back_stamps = read_stamps(f'{dataroot}/sensor_data/SICK_back_stamp.csv')
    middle_stamps = read_stamps(f'{dataroot}/sensor_data/SICK_middle_stamp.csv')
    matches = merge_dataset(back_stamps, middle_stamps)
    matches = np.array(matches)
    R_back, T_back = read_calib(f'{dataroot}/calibration/Vehicle2BackSick.txt')
    print('R_back:')
    print(R_back)
    print('T_back:')
    print(T_back)
    R_middle, T_middle = read_calib(f'{dataroot}/calibration/Vehicle2MiddleSick.txt')
    print('R_middle:')
    print(R_middle)
    print('T_middle:')
    print(T_middle)
    with open(f'{dataroot}/sensor_data/SICK_merged_stamp.csv', 'w') as f:
        for match in matches:
            back_scan = sick_2d_2_3d(read_sick(f'{dataroot}/sensor_data/SICK_back/{match[0]}.bin'))
            middle_scan = sick_2d_2_3d(read_sick(f'{dataroot}/sensor_data/SICK_middle/{match[1]}.bin'))
            merged_scan = merge_two_scans(back_scan, middle_scan, R_back, T_back, R_middle, T_middle)
            avg_stamp = (match[0] + match[1]) // 2
            merged_scan.tofile(f'{dataroot}/sensor_data/SICK_merged/{avg_stamp}.bin')
            f.write(f'{avg_stamp}\n')    

if __name__ == '__main__':
    process_sick('../urban39')
    # process_vlp('../urban39')
