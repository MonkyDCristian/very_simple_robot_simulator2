#!/usr/bin/env python3

import signal
import threading
import rclpy
import numpy as np

from os import path
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from PIL import Image as PILImage

from geometry_msgs.msg import Pose, Quaternion, Point
from std_msgs.msg import Header
from nav_msgs.msg import MapMetaData, OccupancyGrid
from tf_transformations import euler_from_quaternion, quaternion_from_euler

from world_state_gui import WorldStateGUI


class WorldSimulator(Node):

  def __init__(self):
    super().__init__('world_state_gui')
    self.variable_init()
    self.comunication_init()
    self.update_map()

    """
    if rospy.has_param('/world_state_gui/map_file'):
      yaml_file = rospy.get_param('/world_state_gui/map_file')
      rospy.loginfo('Loading map file: %s' % (yaml_file))
      
      if path.isfile(yaml_file):
        self.wsg.load_map(yaml_file)
        self.update_map()
     
      else:
        rospy.logerr('Map file [%s] does not exist' % (yaml_file))
    """


  def variable_init(self):
    self.wsg = WorldStateGUI(self)

  
  def comunication_init(self):
    self.pub_map_metadata = self.create_publisher(MapMetaData, 'map_metadata', 1)
    self.pub_map = self.create_publisher(OccupancyGrid, 'map', 1)
    self.pub_initial_pose = self.create_publisher(Pose, 'initial_pose', 1)

    self.subscription = self.create_subscription(Pose, 'real_pose', self.update_robot_pose, 1)
    self.subscription  # prevent unused variable warning


  def send_initial_pose(self, robot_pose, metric = False):
    if metric:
      x, y, yaw = robot_pose

    else:
      x, y = self.wsg.gui_converter.pixel2metric(robot_pose[0], robot_pose[1])
      yaw = robot_pose[2]
    
    quat = quaternion_from_euler(0.0, 0.0, yaw)
    pix_pose = Pose(Point(x, y, 0.0), Quaternion(*quat))
    self.pub_initial_pose.publish(pix_pose)


  def update_robot_pose(self, pose):
    quat = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    roll, pitch, yaw = euler_from_quaternion(quat)
    self.wsg.update_robot_pose(yaw, pose)
  

  def update_map(self):
    background_image = self.wsg.update_map()

    if self.wsg.map_resolution != self.wsg.gui_resolution:
      factor = self.wsg.gui_resolution / self.wsg.map_resolution
      width = int(np.ceil(factor * background_image.size[0]))
      height = int(np.ceil(factor * background_image.size[1]))
      background_image = background_image.resize((width, height), resample = PILImage.NEAREST)

    width, height = background_image.size
    
    map_point = Point()
    map_point.x, map_point.y, map_point.z = 0.0, height * self.wsg.map_resolution, 0.0
    
    map_quat = Quaternion()
    map_quat.x, map_quat.y, map_quat.z, map_quat.w = quaternion_from_euler(0.0, 0.0, 0.0)

    map_pose = Pose()
    map_pose.position = map_point
    map_pose.orientation = map_quat

    map_metadata = MapMetaData()
    map_metadata.map_load_time = self.get_clock().now().to_msg()
    map_metadata.resolution = self.wsg.map_resolution
    map_metadata.width, map_metadata.height = width, height
    map_metadata.origin = map_pose
    
    self.pub_map_metadata.publish(map_metadata)

    og_header = Header()
    og_header.stamp = self.get_clock().now().to_msg()
    og_header.frame_id = 'map_frame'
    
    og_data = np.array(background_image)
    og_data = (100 - (og_data / 255.0) * 100).astype(np.uint8)
    og_data = og_data.reshape(height * width)
    
    occupancy_grid = OccupancyGrid()
    occupancy_grid.header = og_header
    occupancy_grid.info = map_metadata
    occupancy_grid.data = og_data.tolist()
    self.pub_map.publish(occupancy_grid)


  def reset_state(self):
    self.send_initial_pose([float('inf'), float('inf'), 0.0], True)


  def sigint_handler(self, signum, frame):
    self.get_logger().info('world_state_gui is shutting down')
    self.wsg.sigint_handler(signum, frame)
    rclpy.shutdown()


def main(args=None):
  rclpy.init(args=args)
  
  world_simulator = WorldSimulator()
  signal.signal(signal.SIGINT, world_simulator.sigint_handler)
  signal.signal(signal.SIGTERM, world_simulator.sigint_handler)
  
  x = threading.Thread(target=world_simulator.wsg.mainloop)
  x.start()

  rclpy.spin(world_simulator)


if __name__ == '__main__':
  main()  
