import rclpy, cv2, numpy as np

from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from geometry_msgs.msg import Pose, Quaternion, Point
from std_msgs.msg import Header
from nav_msgs.msg import MapMetaData, OccupancyGrid
from tf_transformations import euler_from_quaternion, quaternion_from_euler

try:  # for ROS2 run and launch compatibility  
  from .pygame_gui import PyGameGUI

except ImportError: # for python3 run compatibility
  from pygame_gui import PyGameGUI


class WorldSimulator(Node):

    def __init__(self):
      super().__init__('world_simulator')
      self.variables_init()
      self.connections_init()
      self.update_map()

    
    def variables_init(self):
      self.pg_gui = PyGameGUI()
      
      self.timer_cb_group = MutuallyExclusiveCallbackGroup()
      self.sub1_cb_group = MutuallyExclusiveCallbackGroup()
      self.sub2_cb_group = MutuallyExclusiveCallbackGroup()


      timer_period = 0.05  # seconds = 20 hz
      self.timer = self.create_timer(timer_period, self.update_gui, callback_group=self.timer_cb_group)


    def connections_init(self):
      self.pub_map_metadata = self.create_publisher(MapMetaData, 'map_metadata', 10)
      self.pub_map = self.create_publisher(OccupancyGrid, 'map', 10)
      self.pub_initial_pose = self.create_publisher(Pose, 'initial_pose', 10)

      self.subscription = self.create_subscription(Pose, 'real_pose', self.update_robot_pose, 10, callback_group=self.sub1_cb_group)
      self.subscription = self.create_subscription(String, 'load_map', self.load_new_map, 10, callback_group=self.sub2_cb_group)


    def update_gui(self):
      self.pg_gui.update_screen()
     
      if self.pg_gui.map.changed:
        self.update_map()
        self.pg_gui.map.changed = False
      
      if self.pg_gui.robot.pose_changed:
        self.send_initial_pose()
        self.pg_gui.robot.pose_changed = False


    def update_robot_pose(self, pose):
      if not self.pg_gui.robot.pose_changing and not self.pg_gui.robot.pose_changed:
        quat = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        roll, pitch, yaw = euler_from_quaternion(quat)
      
        self.pg_gui.robot.x, self.pg_gui.robot.y = self.pg_gui.map.converter.metric2pixel(pose.position.x, pose.position.y)
        self.pg_gui.robot.yaw = yaw


    def send_initial_pose(self):
      x, y = self.pg_gui.map.converter.pixel2metric(self.pg_gui.robot.x, self.pg_gui.robot.y)
      yaw = self.pg_gui.robot.yaw

      robot_pose = self.create_pose(x=x, y=y, yaw=yaw)
      self.pub_initial_pose.publish(robot_pose)


    def reset_state(self):
      self.pg_gui.robot.x, self.pg_gui.robot.y = self.pg_gui.map.converter.metric2pixel(1.0, 1.0)
      self.pg_gui.robot.yaw = 0.0
      
      self.send_initial_pose(self)
    

    def update_map(self):
      map_metadata = MapMetaData()
      map_metadata.map_load_time = self.get_clock().now().to_msg()
      map_metadata.resolution = self.pg_gui.map.resolution
      map_metadata.width, map_metadata.height = self.pg_gui.map.width, self.pg_gui.map.height
      map_metadata.origin = self.create_pose(x=-1.0, y=self.pg_gui.map.height*self.pg_gui.map.resolution -1.0 , roll=np.pi)
      
      self.pub_map_metadata.publish(map_metadata)

      og_header = Header()
      og_header.stamp = self.get_clock().now().to_msg()
      og_header.frame_id = 'odom'
      
      og_data = cv2.cvtColor(self.pg_gui.map.np_map, cv2.COLOR_RGB2GRAY)
      og_data = (100 - (og_data / 255.0) * 100).astype(np.uint8)
      og_data = og_data.reshape(self.pg_gui.map.width * self.pg_gui.map.height)
      
      occupancy_grid = OccupancyGrid()
      occupancy_grid.header = og_header
      occupancy_grid.info = map_metadata
      occupancy_grid.data = og_data.tolist()
      
      self.pub_map.publish(occupancy_grid)
    

    def load_new_map(self, msg):
      yaml_file = msg.data
      self.pg_gui.map.load_new_map(yaml_file)
      self.pg_gui.setup_screen()
      self.update_map()


    def create_pose(self, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
      point = Point()
      point.x, point.y, point.z = x, y, z
      
      quat = Quaternion()
      quat.x, quat.y, quat.z, quat.w = quaternion_from_euler(roll, pitch, yaw)

      pose = Pose()
      pose.position = point
      pose.orientation = quat

      return pose


def main(args=None):
    rclpy.init(args=args)
    
    world_simulator = WorldSimulator()
    
    executor = MultiThreadedExecutor()
    executor.add_node(world_simulator)

    try:
      world_simulator.get_logger().info('Beginning simulation, shut down with CTRL-C')
      executor.spin()
   
    except KeyboardInterrupt:
      world_simulator.get_logger().info('Keyboard interrupt, shutting down.\n')
    
    world_simulator.destroy_node()

    
if __name__ == '__main__':
    main()
