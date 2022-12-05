#!/usr/bin/env python3

import rclpy, numpy as np

from rclpy.node import Node

from geometry_msgs.msg import Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from tf_transformations import euler_from_quaternion

try:  # for ROS2 run and launch compatibility  
  from .utils import CoordinateConverter
  from .rangefinder import build_pixel_rangefinder

except ImportError: # for python3 run compatibility
  from utils import CoordinateConverter
  from rangefinder import build_pixel_rangefinder


class LidarSimulator(Node):

  def __init__(self):
    super().__init__('lidar_simulator')
    self.variable_init()
    self.connection_init()
  
  
  def variable_init(self):
    self.hfov = 57*np.pi/180.0 # [rad] (57 [degrees])
    self.n_h_scans = 57
    self.map_resolution = 0.01 # [m/pix]
    self.view_depth = 4.0 # [m]
    self.std_error = 0.02 # [m]

    self.view_depth_pix = self.view_depth / self.map_resolution # [pix]
    self.converter = None
    self.mapimg = np.array([])
    self.h_beam_angles = np.linspace(self.hfov/2.0, -self.hfov/2.0, self.n_h_scans)

    self.lidar_fov = np.pi
    self.lidar_n_h_scans = 181

    self.laserScan = LaserScan()
    self.laserScan.header.frame_id = "base_link"
    self.laserScan.angle_min = -self.lidar_fov/2.0
    self.laserScan.angle_max = self.lidar_fov/2.0
    self.laserScan.angle_increment = 1.0*np.pi/180.0
    self.laserScan.time_increment = 0.00001
    self.laserScan.scan_time = 0.001*181
    self.laserScan.range_min = 0.0
    self.laserScan.range_max = self.view_depth
    self.laserScan.intensities = []
  

  def connection_init(self):
    self.pub_scan = self.create_publisher(LaserScan, '/scan', 10)

    self.sub_map = self.create_subscription(OccupancyGrid, 'map', self.set_map, 10)
    self.sub_real_pose = self.create_subscription(Pose, '/real_pose', self.new_pose, 10)


  def new_pose(self, pose):
    if len(self.mapimg) == 0:
      return None

    x, y = self.converter.metric2pixel(pose.position.x, pose.position.y)
    
    if y < 0 or self.mapimg.shape[0] <= y or x < 0 or self.mapimg.shape[1] <= x:
      scans = self.view_depth * np.ones(self.lidar_n_h_scans)
      self.send_laser_scan(scans)
      return None

    roll, pitch, yaw = euler_from_quaternion((pose.orientation.x,
                                                pose.orientation.y,
                                                pose.orientation.z,
                                                pose.orientation.w))

    robot_pose = (x, y, yaw)
    pixel_lidar, distance_sensor = build_pixel_rangefinder(self.mapimg,
                                                            robot_pose,
                                                            self.hfov,
                                                            self.n_h_scans,
                                                            self.view_depth_pix)

    distance_sensor = self.map_resolution * np.array(distance_sensor) # [m]
    distance_sensor = np.random.normal(distance_sensor, self.std_error)
    distance_sensor[distance_sensor > self.view_depth-0.2] = self.view_depth # filter pix to meter conversion errors

    scans = np.zeros(self.lidar_n_h_scans)
    out_of_fov_beams = int(self.lidar_n_h_scans/2) - int(self.n_h_scans/2)
    scans[:out_of_fov_beams] = self.view_depth
    scans[-out_of_fov_beams:] = self.view_depth
    scans[out_of_fov_beams:-out_of_fov_beams] = distance_sensor

    self.send_laser_scan(scans)


  def send_laser_scan(self, scans):
    self.laserScan.header.stamp =  self.get_clock().now().to_msg() 
    self.laserScan.ranges = list(scans)
    self.pub_scan.publish(self.laserScan)


  def set_map(self, occupancy_grid):
    width, height = occupancy_grid.info.width, occupancy_grid.info.height
    self.map_resolution = occupancy_grid.info.resolution
    self.mapimg = 100 - np.array(occupancy_grid.data).reshape((height, width))
    self.converter = CoordinateConverter(0.0, self.mapimg.shape[0] * self.map_resolution, self.map_resolution)
    self.view_depth_pix = self.view_depth / self.map_resolution # [pix]


def main(args=None):
    rclpy.init(args=args)
    lidar_simulator = LidarSimulator()
    rclpy.spin(lidar_simulator)


if __name__ == '__main__':
  main()
