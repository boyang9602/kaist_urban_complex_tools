import sys
import pathlib
import numpy as np
from datetime import datetime
from ground_truth_tools import load_coors
from tools import match_timestamps
from kaist_urban_complex import KUCSchema

from record_msg import pypcd

def to_timestamp(sensor_time):
  date_sec, nano_sec = sensor_time.split('.')
  time_sec = datetime.strptime(date_sec, '%Y-%m-%d %H:%M:%S')
  return datetime.timestamp(time_sec) + float(nano_sec)*1e-9

def convert(filepath, outpath, timestamp, compression='binary_compressed'):
    # read
    if filepath[-4:] == '.bin':
        scan = np.fromfile(filepath, dtype=np.float32)
    elif filepath[-4:] == '.txt':
        scan = np.loadtxt(filepath)
    else:
        raise "Unsupported file extension. It has to be 'txt' or 'bin'"
    scan = scan.reshape(-1, 4)
    intensities = (scan[:, -1] * 255).astype(np.uint8)
    timestamps = np.ones_like(intensities, dtype=np.float64) * timestamp
    md = {'version': .7,
          'fields': ['x', 'y', 'z', 'intensity', 'timestamp'],
          'count': [1, 1, 1, 1, 1],
          'width': len(scan),
          'height': 1,
          'viewpoint': [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
          'points': len(scan),
          'size': [4, 4, 4, 1, 8],
          'type': ['F', 'F', 'F', 'U', 'F'],
          'data': compression}
    pc_data = np.hstack([scan[:, :3], intensities.reshape(-1, 1), timestamps.reshape(-1, 1)])
    pc = pypcd.PointCloud(md, pc_data)
    # save
    pc.save(f'{outpath}')

if __name__ == '__main__':
    outpath = pathlib.Path(sys.argv[1]) / 'pcds'
    outpath.mkdir(parents=True, exist_ok=True)
    pose_stamps, poses, quaternions  = load_coors(f'{sys.argv[1]}/global_coors.csv')
    schema = KUCSchema(sys.argv[1])
    _, _, merged = schema.vlp_schemes()
    pose_idxs = match_timestamps([vlp.timestamp for vlp in merged], pose_stamps)
    with open('aligned_poses.txt', 'w') as f:
        for idx, (pose_idx, vlp) in enumerate(zip(pose_idxs, merged)):
            convert(f'{vlp.data_folder}/{vlp.timestamp}.bin', f'{outpath}/{idx}.pcd', vlp.timestamp)
            s = f'{idx} {pose_stamps[pose_idx]} '
            for coor in poses[pose_idx]:
                s += f'{coor} '
            for q in quaternions[pose_idx]:
                s += f'{q} '
            f.write(f'{s[:-1]}\n')
