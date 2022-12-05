#!/usr/bin/env python3

import rclpy, numpy as np

from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist, Pose, Quaternion, Point, TransformStamped
from nav_msgs.msg import Odometry

from tf2_ros import TransformBroadcaster #***
from tf_transformations import euler_from_quaternion, quaternion_from_euler


class KobukiSimulator(Node):
  def __init__(self, initial_x = 1.0, initial_y = 1.0, initial_yaw = 0.0):
    super().__init__('kobuki_simulator')
    self.variable_init(initial_x, initial_y, initial_yaw)
    self.connection_init()
  

  def variable_init(self, initial_x, initial_y, initial_yaw):
    # params
    self.initial_x, self.initial_y, self.initial_yaw = initial_x, initial_y, initial_yaw
    self.simulate_ground_friction = True
    self.reset = False
    
    self.timer_cb_group = MutuallyExclusiveCallbackGroup()
    self.sub1_cb_group = MutuallyExclusiveCallbackGroup()
    self.sub2_cb_group = MutuallyExclusiveCallbackGroup()

    self.timer_period = 0.2  # seconds = 5 hz
    self.timer = self.create_timer(self.timer_period, self.main_loop, callback_group=self.timer_cb_group)
    self.last_time = self.get_clock().now()
    
    # set real pose data
    self.real_pose = self.create_pose(x=self.initial_x, y=self.initial_y, yaw=self.initial_yaw)
    self.robot_speed = self.create_twist()

    self.tf_broadcaster = TransformBroadcaster(self)
    
    self.odom_tf = TransformStamped()
    self.odom_tf.header.frame_id = 'odom'
    self.odom_tf.child_frame_id = 'base_link'

    # set odom data
    self.odom = Odometry()
    self.odom.header.frame_id = 'odom'
    self.odom.child_frame_id = 'base_link'
    self.odom.pose.pose = self.create_pose()


  def connection_init(self):
    self.pub_odom = self.create_publisher(Odometry, "odom", 10)
    self.pub_real_pose = self.create_publisher(Pose, "real_pose", 10)

    self.sub_cmd_vel = self.create_subscription(Twist, 'cmd_vel', self.move, 10, callback_group=self.sub1_cb_group)
    self.sub_init_pose = self.create_subscription(Pose, 'initial_pose', self.set_initial_pose, 10, callback_group=self.sub2_cb_group)


  def set_initial_pose(self, initial_pose):
    self.real_pose = initial_pose
    self.pub_real_pose.publish(self.real_pose)


  def main_loop(self):

    if self.reset:
      point = Point()
      point.x, point.y, point.z = 0.0, 0.0, 0.0
      self.odom.pose.pose.position = point
      self.reset = False
    
    vx, vy, vyaw = self.get_robot_speed()
    
    dt = self.timer_period

    self.update_real_pose(vx, vy, vyaw, dt)
    self.update_odom(vx, vy, vyaw, dt)
      

  def update_real_pose(self, vx, vy, vyaw, dt):
    x, y, yaw = self.update_pose(vx, vy, vyaw, dt, self.real_pose)
    self.real_pose = self.create_pose(x = x, y = y, yaw = yaw)
    self.pub_real_pose.publish(self.real_pose)

  
  def update_odom(self, vx, vy, vyaw, dt):
    x, y, yaw = self.update_pose(vx, vy, vyaw, dt, self.odom.pose.pose)
    
    # first, we'll publish the transform over tf
    self.odom_tf.header.stamp = self.get_clock().now().to_msg()
    self.send_br_tf(self.odom_tf, x, y, yaw)

    # set the position and velocity 
    self.odom.pose.pose = self.create_pose(x = x, y = y, yaw = yaw)
    self.odom.twist.twist = self.create_twist(l_x=vx, l_y=vy, a_z=vyaw)
    self.odom.header.stamp =  self.get_clock().now().to_msg()
    
    self.pub_odom.publish(self.odom)
  
  def send_br_tf(self, t, x, y, yaw):
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = 0.0

    q = quaternion_from_euler(0, 0, yaw)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    
    self.tf_broadcaster.sendTransform(t)


  def update_pose(self, vx, vy, vyaw, dt, pose):
    quat = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    roll, pitch, yaw = euler_from_quaternion(quat)

    delta_x = (vx * np.cos(yaw + self.initial_yaw) - vy * np.sin(yaw + self.initial_yaw)) * dt
    delta_y = (vx * np.sin(yaw + self.initial_yaw) + vy * np.cos(yaw + self.initial_yaw)) * dt
    
    if self.simulate_ground_friction:
      delta_yaw = 0.9 * vyaw * dt
    
    else:
      delta_yaw = vyaw * dt

    x, y = pose.position.x + delta_x, pose.position.y + delta_y
    yaw = yaw + delta_yaw

    return x, y, yaw


  def move(self, twist): #***
    if np.isnan(twist.linear.x) or np.isnan(twist.angular.z):
      self.get_logger().info(f'Invalid speed command received: lin.x: {str(twist.linear.x)}, ang.z: {str(twist.angular.z)}')
      return
    
    # movement is restricted to x and yaw
    self.robot_speed = self.create_twist(l_x=twist.linear.x, a_z=twist.angular.z )


  def create_pose(self, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
    point = Point()
    point.x, point.y, point.z = x, y, z
    
    quat = Quaternion()
    quat.x, quat.y, quat.z, quat.w = quaternion_from_euler(roll, pitch, yaw)

    pose = Pose()
    pose.position = point
    pose.orientation = quat

    return pose
  

  def create_twist(self, l_x=0.0, l_y=0.0, l_z=0.0, a_x=0.0, a_y=0.0, a_z=0.0):
    twist = Twist()
    twist.linear.x, twist.linear.y, twist.linear.z = l_x, l_y, l_z
    twist.angular.x, twist.angular.y, twist.angular.z = a_x, a_y, a_z
    return twist


  def get_robot_speed(self):
    return self.robot_speed.linear.x, self.robot_speed.linear.y, self.robot_speed.angular.z


def main(args=None):
    rclpy.init(args=args)
    
    kobuki_simulator = KobukiSimulator()

    executor = MultiThreadedExecutor()
    executor.add_node(kobuki_simulator)
    
    kobuki_simulator.get_logger().info('Initializing Kobuki Simulator')
    executor.spin()
   
    kobuki_simulator.destroy_node()


if __name__ == '__main__':
  main()