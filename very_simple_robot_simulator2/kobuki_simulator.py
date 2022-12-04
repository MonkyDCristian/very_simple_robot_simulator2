#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Twist, Pose, Quaternion, Vector3, Point
from nav_msgs.msg import Odometry
from tf import TransformBroadcaster
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np

class KobukiSimulator(object):

  def __init__(self, initial_x = 1.0, initial_y = 1.0, initial_yaw = 0.0):
    rospy.loginfo('Initializing Kobuki Simulator')
    rospy.on_shutdown(self.shutdown)
    self.variable_init(initial_x, initial_y, initial_yaw)
    self.connection_init()
  

  def variable_init(self, initial_x, initial_y, initial_yaw):
    # params
    self.initial_x, self.initial_y, self.initial_yaw = initial_x, initial_y, initial_yaw
    self.simulate_ground_friction = True
    self.reset = False

    self.real_pose_publish_rate = 5.0 # [Hz]
    self.rate = rospy.Rate(self.real_pose_publish_rate)
    
    # set real pose data
    quat = quaternion_from_euler(0.0, 0.0, self.initial_yaw)
    self.real_pose = Pose(Point(self.initial_x, self.initial_y, 0.0), Quaternion(*quat))
    self.current_speed = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
    
    self.odom_broadcaster = TransformBroadcaster()

    # set odom data
    self.odom = Odometry()
    self.odom.header.frame_id = 'odom'
    self.odom.child_frame_id = 'base_link'
    
    odom_quat = quaternion_from_euler(0.0, 0.0, 0.0)
    self.odom.pose.pose = Pose(Point(0.0, 0.0, 0.0), Quaternion(*odom_quat))


  def connection_init(self):
    self.pub_odom = rospy.Publisher('odom', Odometry, queue_size = 10)
    self.pub_real_pose = rospy.Publisher('real_pose', Pose, queue_size = 1)

    rospy.Subscriber('cmd_vel', Twist, self.move)
    rospy.Subscriber('initial_pose', Pose, self.set_initial_pose)


  def set_initial_pose(self, initial_pose):
    if initial_pose.position.x == float('inf') and initial_pose.position.y == float('inf'):
      quat = quaternion_from_euler(0.0, 0.0, self.initial_yaw)
      initial_pose = Pose(Point(self.initial_x, self.initial_y, 0.0), Quaternion(*quat))
      self.reset = True

    self.real_pose = initial_pose
    self.pub_real_pose.publish(self.real_pose)


  def main_loop(self):
    # Depends on incoming speed
    current_time = rospy.Time.now()
    last_time = rospy.Time.now()
    
    while not rospy.is_shutdown():
      
      if self.reset:
        self.odom.pose.pose.position = Point(0.0, 0.0, 0.0)
        self.reset = False
      
      vx, vy, vyaw = self.get_current_speed()
      
      current_time = rospy.Time.now()
      dt = (current_time - last_time).to_sec()

      self.update_real_pose(vx, vy, vyaw, dt)
      self.update_odom(vx, vy, vyaw, dt)

      last_time = current_time
      self.rate.sleep()
  

  def update_real_pose(self, vx, vy, vyaw, dt):
    x, y, yaw = self.update_pose(vx, vy, vyaw, dt, self.real_pose)

    quat = quaternion_from_euler(0, 0, yaw)
    self.real_pose = Pose(Point(x, y, 0.0), Quaternion(*quat))
    
    self.pub_real_pose.publish(self.real_pose)

  
  def update_odom(self, vx, vy, vyaw, dt):
    x, y, yaw = self.update_pose(vx, vy, vyaw, dt, self.odom.pose.pose)
    
    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = quaternion_from_euler(0.0, 0.0, yaw)

    # first, we'll publish the transform over tf
    self.odom_broadcaster.sendTransform((x, y, 0.0), odom_quat, rospy.Time.now(), 'base_link', 'odom')

    # set the position and velocity 
    self.odom.pose.pose = Pose(Point(x, y, 0.0), Quaternion(*odom_quat))
    self.odom.twist.twist = Twist(Vector3(vx, vy, 0.0), Vector3(0.0, 0.0, vyaw))

    self.odom.header.stamp = rospy.Time.now()
    self.pub_odom.publish(self.odom)


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


  def move(self, twist):
    if np.isnan(twist.linear.x) or np.isnan(twist.angular.z):
      rospy.logwarn(f'Invalid speed command received: lin.x: {str(twist.linear.x)}, ang.z: {str(twist.angular.z)}')
      return
    # movement is restricted to x and yaw
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    self.current_speed = twist


  def velocity_state(self, state):
    rospy.loginfo(f'Current subscriptor: {state.data}')
    if state.data == 'idle':
      self.current_speed = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))


  def get_current_speed(self):
    return self.current_speed.linear.x, self.current_speed.linear.y, self.current_speed.angular.z


  def shutdown(self):
    rospy.loginfo('Stopping Kobuki Simulator')
    # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
    self.current_speed = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
    # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
    rospy.sleep(1)


if __name__ == '__main__':
  rospy.init_node('kobuki_simulator')
  kobuki_sim = KobukiSimulator()
  kobuki_sim.main_loop()