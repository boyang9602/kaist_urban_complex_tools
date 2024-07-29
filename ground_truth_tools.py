import numpy as np
from kaist_urban_complex import KUCSchema
from tools import match_timestamps
import sys
def rt2coor(poses, start_coor):
    coors = [start_coor]
    for pose in poses:
        coors.append(pose[:, :-1] @ coors[-1] + pose[:, -1])
    return coors

def load_poses(filename):
    content = np.loadtxt(filename, [('stamp', np.int64), ('pose', np.float64, (3, 4))], delimiter=',')
    stamps = content['stamp']
    poses = content['pose']
    return stamps, poses

def load_coors(filename):
    content = np.loadtxt(filename, 
                         [('stamp', np.int64), 
                          ('pose', np.float64, (3,)), 
                          ('quaternion', np.float64, (4,))], 
                         delimiter=',')
    return content['stamp'], content['pose'], content['quaternion']


if __name__ == '__main__':
    dataroot = sys.argv[1]
    stamps, poses = load_poses(f'{dataroot}/global_pose.csv')
    # coors = rt2coor(poses, P0)
    coors = poses[:, :, -1]
    schema = KUCSchema(dataroot)
    imus = schema.imu_schemes(version=2)
    imu_idxs = match_timestamps(stamps, [imu.timestamp for imu in imus])
    if len(imu_idxs) == len(stamps):
        pass
    else:
        raise "Cannot fully match IMUs and poses"
    with open(f'{dataroot}/global_coors.csv', 'w') as f:
        for stamp, coor, imu_idx in zip(stamps, coors, imu_idxs):
            imu = imus[imu_idx]
            q = [imu.qx, imu.qy, imu.qz, imu.qw]
            f.write(f'{stamp},{",".join([str(i) for i in coor])},{",".join([str(i) for i in q])}\n')
