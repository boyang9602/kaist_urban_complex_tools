#!/usr/bin/env python3

###############################################################################
# Copyright 2022 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################
'''Generate apollo record file by kitti raw sensor data.'''

from sensor import *
from cyber_record.record import Record
from record_msg.builder import (
  PointCloudBuilder,
  IMUBuilder,
  GnssBestPoseBuilder)
from kaist_urban_complex import KUCSchema, KUC
import sys

LOCALIZATION_TOPIC = '/apollo/localization/pose'
TF_TOPIC= '/tf'
IMU_TOPIC = '/apollo/sensor/gnss/imu'
GNSS_BEST_POSE_TOPIC = '/apollo/sensor/gnss/best_pose'
VLP_TOPIC = '/apollo/sensor/velodyne/compensator/PointCloud2'

def dataset_to_record(kuc, record_root_path):
  """Construct record message and save it as record

  Args:
      kuc (_type_): KUC
      record_root_path (str): record file saved path
  """
  pc_builder = PointCloudBuilder()
  imu_builder = IMUBuilder()
  gnss_builder = GnssBestPoseBuilder()

  channel_name = None

  with Record(record_root_path, mode='w') as record:
    for sensor in kuc:
      t = sensor.timestamp
      t_sec = t * 1e-9
      if isinstance(sensor, IMU):
        channel_name = IMU_TOPIC
        linear_acceleration = [sensor.ax, sensor.ay, sensor.az]
        angular_velocity = [sensor.gx, sensor.gy, sensor.gz]
        pb_msg = imu_builder.build(linear_acceleration, angular_velocity, t_sec)
      elif isinstance(sensor, VrsGps):
        channel_name = GNSS_BEST_POSE_TOPIC
        pb_msg = gnss_builder.build(sensor.latitude, sensor.longitude, sensor.altitude, 0, t_sec,
                                    latitude_std_dev = sensor.latitude_std, 
                                    longitude_std_dev = sensor.longitude_std,
                                    height_std_dev = sensor.altitude_std)
      elif isinstance(sensor, VLP):
        channel_name = VLP_TOPIC
        pb_msg = pc_builder.build_nuscenes(f'{sensor.data_folder}/{t}.bin', 'velodyne', t_sec)
      else:
        raise "Not implemented yet."
      record.write(channel_name, pb_msg, t)

def convert_dataset(dataset_path, record_path, version_info, lidar_mode=1):
  """Generate apollo record file by KITTI dataset

  Args:
      dataset_path (str): KAIST dataset path
      record_path (str): record file saved path
  """
  kuc_schema = KUCSchema(dataroot=dataset_path)
  kuc = KUC(kuc_schema, ['vlp', 'imu', 'vrs_gps'], version_info, lidar_mode=lidar_mode)

  print("Start to convert scene, Pls wait!")
  dataset_to_record(kuc, record_path)
  print("Success! Records saved in '{}'".format(record_path))

if __name__ == '__main__':
  from dataset_config import version_info
  datasets_root = sys.argv[1]
  dataset_name = sys.argv[2]
  output = sys.argv[3]
  convert_dataset(f'{datasets_root}/{dataset_name}', output, version_info[dataset_name], lidar_mode=1)