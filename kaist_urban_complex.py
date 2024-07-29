import numpy as np

from sensor import *
import heapq
from lidar_process import process_sick, process_vlp
import os

class KUCSchema:
    def __init__(self, dataroot=None) -> None:
        self.dataroot = dataroot

    def _read_stamp_files(self, path, dtype=np.int64):
        return np.loadtxt(path, dtype, delimiter=',')

    def vlp_schemes(self):
        # 3D Lidar
        if not os.path.exists(f'{self.dataroot}/sensor_data/VLP_merged_stamp.csv'):
            process_vlp(self.dataroot)
        merged_stamps = self._read_stamp_files(f'{self.dataroot}/sensor_data/VLP_merged_stamp.csv')
        left_stamps = self._read_stamp_files(f'{self.dataroot}/sensor_data/VLP_left_stamp.csv')
        right_stamps = self._read_stamp_files(f'{self.dataroot}/sensor_data/VLP_right_stamp.csv')
        merged_lidars = [VLP(stamp, f'{self.dataroot}/sensor_data/VLP_merged') for stamp in merged_stamps]
        left_lidars = [VLP(stamp, f'{self.dataroot}/sensor_data/VLP_left') for stamp in left_stamps]
        right_lidars = [VLP(stamp, f'{self.dataroot}/sensor_data/VLP_right') for stamp in right_stamps]
        return left_lidars, right_lidars, merged_lidars

    def sick_schemes(self):
        # 2D Lidar
        if not os.path.exists(f'{self.dataroot}/sensor_data/SICK_merged_stamp.csv'):
            process_sick(self.dataroot)
        merged_stamps = self._read_stamp_files(f'{self.dataroot}/sensor_data/SICK_merged_stamp.csv')
        back_stamps = self._read_stamp_files(f'{self.dataroot}/sensor_data/SICK_back_stamp.csv')
        middle_stamps = self._read_stamp_files(f'{self.dataroot}/sensor_data/SICK_middle_stamp.csv')
        merged_lidars = [SICK(stamp, f'{self.dataroot}/sensor_data/SICK_merged') for stamp in merged_stamps]
        back_lidars = [SICK(stamp, f'{self.dataroot}/sensor_data/SICK_back') for stamp in back_stamps]
        middle_lidars = [SICK(stamp, f'{self.dataroot}/sensor_data/SICK_middle') for stamp in middle_stamps]
        return back_lidars, middle_lidars, merged_lidars

    def stereo_schemes(self):
        # stereo images
        stamps = self._read_stamp_files(f'{self.dataroot}/sensor_data/stereo_stamp.csv')
        stereos = [Stereo(stamp, f'{self.dataroot}/image') for stamp in stamps]
        return stereos

    def altimeter_schemes(self):
        dtype = [('stamp', 'i8'), ('altitude', 'f4')]
        stamps_n_data = self._read_stamp_files(f'{self.dataroot}/sensor_data/altimeter.csv', dtype=dtype)
        altimeters = [Altimeter(stamp, data=altitude) for stamp, altitude in stamps_n_data]
        return altimeters

    def encoder_schemes(self):
        dtype = [('stamp', 'i8'), ('left_count', 'i4'), ('right_count', 'i4')]
        stamps_n_data = self._read_stamp_files(f'{self.dataroot}/sensor_data/encoder.csv', dtype=dtype)
        encoders = [Encoder(stamp, data=data) for stamp, data in \
                    zip(stamps_n_data['stamp'], stamps_n_data[[pair[0] for pair in dtype[1:]]])]
        return encoders
    
    def fog_schemes(self):
        dtype = [('stamp', 'i8'), ('delta_roll', 'g'), ('delta_pitch', 'g'), ('delta_yaw', 'g')]
        stamps_n_data = self._read_stamp_files(f'{self.dataroot}/sensor_data/fog.csv', dtype=dtype)
        fogs = [Fog(stamp, data=data) for stamp, data in \
                zip(stamps_n_data['stamp'], stamps_n_data[[pair[0] for pair in dtype[1:]]])]
        return fogs

    def gps_schemes(self):
        dtype = [('stamp', 'i8'), ('latitude', 'f4'), ('longitude', 'f4'), ('altitude', 'f4'), \
                 ('position_covariance', 'f4', (9,))]
        stamps_n_data = self._read_stamp_files(f'{self.dataroot}/sensor_data/gps.csv', dtype=dtype)
        gps = [Gps(stamp, data=data) for stamp, data in \
                zip(stamps_n_data['stamp'], stamps_n_data[[pair[0] for pair in dtype[1:]]])]
        return gps
    
    def vrs_gps_schemes(self, version=1):
        dtype = [('stamp', 'i8'), ('latitude', 'g'), ('longitude', 'g'), \
                 ('utm_x', 'g'), ('utm_y', 'g'), ('altitude', 'g'), \
                 ('fix_state', 'i1'), ('num_satellites', 'i1'), ('horizontal_precision', 'g'), \
                 ('latitude_std', 'g'), ('longitude_std', 'g'), ('altitude_std', 'g'), \
                 ('heading_validate_flag', 'i1'), ('magnetic_global_heading', 'i1'), ('speed_in_knot', 'g'), \
                 ('speed_in_km', 'g'), ('GNVTG_mode', 'U1')]
        if version == 1:
            pass
        elif version == 2:
            dtype.append(('ortometric_altitude', 'g'))
        else:
            raise "Version has to be 1 or 2!"

        stamps_n_data = self._read_stamp_files(f'{self.dataroot}/sensor_data/vrs_gps.csv', dtype=dtype)
        vrs_gps = [VrsGps(stamp, data=data) for stamp, data in \
                zip(stamps_n_data['stamp'], stamps_n_data[[pair[0] for pair in dtype[1:]]])]
        return vrs_gps
    
    def imu_schemes(self, version=1):
        dtype = [('stamp', 'i8'), ('qx', 'g'), ('qy', 'g'), ('qz', 'g'), ('qw', 'g'), \
                 ('ex', 'g'), ('ey', 'g'), ('ez', 'g')]
        if version == 1:
            pass
        elif version == 2:
            dtype += [('gx', 'g'), ('gy', 'g'), ('gz', 'g'), \
                        ('ax', 'g'), ('ay', 'g'), ('az', 'g'), \
                        ('mx', 'g'), ('my', 'g'), ('mz', 'g')]
        else:
            raise "Version has to be 1 or 2!"
        
        stamps_n_data = self._read_stamp_files(f'{self.dataroot}/sensor_data/xsens_imu.csv', dtype=dtype)
        imus = [IMU(stamp, data=data) for stamp, data in \
                zip(stamps_n_data['stamp'], stamps_n_data[[pair[0] for pair in dtype[1:]]])]
        return imus

class KUC(object):
  """KAIST Urban Complex dataset

  Args:
      object (_type_): _description_
  """
  def __init__(self, kuc_schema, sensor_of_interests, version_info, lidar_mode=1) -> None:
    self._kuc_schema = kuc_schema
    self.sensor_data_lists = []
    self.sensor_data = None
    self.sensor_of_interests = sensor_of_interests
    assert lidar_mode in [0, 1, 2], "lidar_mode has to be \n\t0: original, \n\t1: merged, \n\t2: both"
    self.lidar_mode = lidar_mode
    self.version_info = version_info
    self.read_messages()

  def __iter__(self):
    for message in self.sensor_data:
      yield message

  def __enter__(self):
    return self

  def __exit__(self, exc_type, exc_value, traceback):
    pass

  def read_messages(self):
    for sensor_name in self.sensor_of_interests:
        sensor_schemes = getattr(self._kuc_schema, f'{sensor_name}_schemes')
        if sensor_name in ['vlp', 'sick']:
            data1, data2, merged_data = sensor_schemes()
            if self.lidar_mode == 0:
                self.sensor_data_lists += [data1, data2]
            elif self.lidar_mode == 1:
                self.sensor_data_lists.append(merged_data)
            else:
                self.sensor_data_lists += [data1, data2, merged_data]
        elif sensor_name in ['imu', 'vrs_gps']:
            data = sensor_schemes(self.version_info[sensor_name])
            self.sensor_data_lists.append(data)
        else:
            data = sensor_schemes()
            self.sensor_data_lists.append(data)
    # sort by timestamp
    self.sensor_data = heapq.merge(*self.sensor_data_lists, key=lambda x: x.timestamp)
