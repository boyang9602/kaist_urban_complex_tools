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

# This file is modified from adataset https://github.com/ApolloAuto/apollo/tree/ffa0765b2b7f6b831d4c2ede6834d6d5ba2f77f6/modules/tools/adataset

class Sensor(object):
  def __init__(self, timestamp, data_folder = None, data = None) -> None:
    # images, point clouds are stored as separate files in a folder
    # GPS, IMU, etc are stored in a single file 
    # if data provided, it should be a list of a single item
    self.timestamp = timestamp
    self.data_folder = data_folder
    self.data = data
    self.parse()

  def parse(self):
    raise NotImplementedError("Must override!")


class Lidar(Sensor):
  def __init__(self, timestamp, data_folder = None, data = None) -> None:
    super().__init__(timestamp, data_folder, data)

  def parse(self):
    pass

class VLP(Lidar):
  pass

class SICK(Lidar):
  pass

class Camera(Sensor):
  def __init__(self, timestamp, data_folder = None, data = None) -> None:
    super().__init__(timestamp, data_folder, data)

  def parse(self):
    pass
  
class Stereo(Sensor):
  def __init__(self, timestamp, data_folder = None, data = None) -> None:
    super().__init__(timestamp, data_folder, data)
    self.left_camera = Camera(timestamp, f'{data_folder}/stereo_left/{timestamp}.png')
    self.right_camera = Camera(timestamp, f'{data_folder}/stereo_right/{timestamp}.png')

  def parse(self):
    pass

class Altimeter(Sensor):
  def __init__(self, timestamp, data_folder = None, data = None) -> None:
    super().__init__(timestamp, data_folder, data)
    self.altitude = data

  def parse(self):
    pass

class Encoder(Sensor):
  def __init__(self, timestamp, data_folder = None, data = None) -> None:
    super().__init__(timestamp, data_folder, data)
    self.left_count = data[0]
    self.right_count = data[1]

  def parse(self):
    pass

class Fog(Sensor):
  def __init__(self, timestamp, data_folder = None, data = None) -> None:
    super().__init__(timestamp, data_folder, data)
    self.delta_roll = data[0]
    self.delta_pitch = data[1]
    self.delta_yaw = data[2]

  def parse(self):
    pass

class Gps(Sensor):
  def __init__(self, timestamp, data_folder = None, data = None) -> None:
    super().__init__(timestamp, data_folder, data)
    self.latitude = data[0]
    self.longitude = data[1]
    self.altitude = data[2]
    self.position_covariance = data[3]

  def parse(self):
    pass

class VrsGps(Sensor):
  def __init__(self, timestamp, data_folder = None, data = None) -> None:
    super().__init__(timestamp, data_folder, data)
    self.latitude = data[0]
    self.longitude = data[1]
    self.utm_x = data[2]
    self.utm_y = data[3]
    self.altitude = data[4]
    self.fix_state = data[5]
    self.num_satellites = data[6]
    self.horizontal_precision = data[7]
    self.latitude_std = data[8]
    self.longitude_std = data[9]
    self.altitude_std = data[10]
    self.heading_validate_flag = data[11]
    self.magnetic_global_heading = data[12]
    self.speed_in_knot = data[13]
    self.speed_in_km = data[14]
    self.GNVTG_mode = data[15]
    self.ortometric_altitude = data[16] if len(data) == 17 else None

  def parse(self):
    pass

class IMU(Sensor):
  def __init__(self, timestamp, data_folder = None, data = None) -> None:
    super().__init__(timestamp, data_folder, data)
    self.qx = data[0]
    self.qy = data[1]
    self.qz = data[2]
    self.qw = data[3]
    self.ex = data[4]
    self.ey = data[5]
    self.ez = data[6]
    if len(data) > 7:
      self.gx = data[7]
      self.gy = data[8]
      self.gz = data[9]
      self.ax = data[10]
      self.ay = data[11]
      self.az = data[12]
      self.mx = data[13]
      self.my = data[14]
      self.mz = data[15]

  def parse(self):
    pass
