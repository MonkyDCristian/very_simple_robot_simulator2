import rclpy

from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from pygame_gui import PyGameGUI

class WorldSimulator(Node):

    def __init__(self):
        super().__init__('base_node')
        self.variables_init()
        self.connections_init()

    
    def variables_init(self):
      self.pgg = PyGameGUI()
      
      self.timer_cb_group = MutuallyExclusiveCallbackGroup()
      self.sub_cb_group = MutuallyExclusiveCallbackGroup()

      timer_period = 0.05  # seconds = 20 hz
      self.timer = self.create_timer(timer_period, self.pgg.update_screen, callback_group=self.timer_cb_group)


    def connections_init(self):
      # code your/s publisher/s
      #self.publisher = self.create_publisher(String, 'topic', 10)

      # code your/s subscriber/s
      #self.subscription = self.create_subscription(String, 'topic', self.callback, 10, callback_group=self.sub_cb_group)
      pass
  

def main(args=None):
    rclpy.init(args=args)
    
    world_simulator = WorldSimulator()
    
    executor = MultiThreadedExecutor()
    executor.add_node(world_simulator)

    try:
      world_simulator.get_logger().info('Beginning client, shut down with CTRL-C')
      executor.spin()
   
    except KeyboardInterrupt:
      world_simulator.get_logger().info('Keyboard interrupt, shutting down.\n')
    
    world_simulator.destroy_node()

    
if __name__ == '__main__':
    main()
