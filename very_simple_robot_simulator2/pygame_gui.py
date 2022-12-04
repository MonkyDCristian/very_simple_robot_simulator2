
import pygame, sys, cv2, numpy as np

from PIL import Image
from pygame.locals import MOUSEBUTTONUP, MOUSEBUTTONDOWN, MOUSEMOTION, KEYDOWN, QUIT
from utils import CoordinateConverter

class Robot(object):
  def __init__(self, x = 30, y = 200, yaw = 0, radius = 30):
    self.radius = radius
    self.x, self.y = x, y
    self.yaw = yaw
   
    self.cir_color = (0, 0, 255) # blue
    self.lin_color = (255, 255, 255) # white


  def draw(self, surface):
    pygame.draw.circle(surface, self.cir_color, (self.x, self.y), self.radius)
    pygame.draw.line(surface, self.lin_color, (self.x, self.y), (self.dir_x, self.dir_y), 2)
  
  @property
  def yaw(self):
    return self._yaw

  @yaw.setter
  def yaw(self, value):
    self._yaw = value
    self.dir_x, self.dir_y = self.x + self.radius*np.cos(-self._yaw), self.y + self.radius*np.sin(-self._yaw)
  

class Map(object):
  def __init__(self, width = 500, height = 290, wall_thick = 3, resolution = 0.01):
    self.np_map = self.make_map(height, width, wall_thick)
    
    self.height, self.width =  self.np_map.shape[:2]
    self.wall_thick = wall_thick
    
    self.pil_map =  Image.fromarray(cv2.cvtColor(self.np_map, cv2.COLOR_BGR2RGB))
    self.pg_map = pygame.image.fromstring(self.pil_map.tobytes(), self.pil_map.size, self.pil_map.mode)
    
    self.map_resolution = resolution
    self.map_converter = CoordinateConverter(0.0, self.height * self.map_resolution, self.map_resolution)
  

  def make_map(self, height, width, wall_thick):
    np_map = 255 * np.ones((height, width), dtype = np.uint8)
    
    horizontal_wall = np.zeros((wall_thick, width), dtype = np.uint8)
    vertical_wall = np.zeros((height + 2 * wall_thick, wall_thick), dtype = np.uint8)
   
    np_map = np.concatenate((horizontal_wall, np_map, horizontal_wall), axis = 0)
    np_map = np.concatenate((vertical_wall, np_map, vertical_wall), axis = 1)
    
    return np_map
  

  def update_map(self, display_surf):
    new_map = pygame.surfarray.array3d(display_surf)
    new_map = new_map.transpose([1, 0, 2])
    self.np_map = cv2.cvtColor(new_map, cv2.COLOR_RGB2BGR)
  
  
  def draw(self, surface):
    surface.blit(self.pg_map, (0, 0))


class PyGameGUI(object):
  def __init__(self):
    pygame.init()
    self.variables_init()
    self.setup_screen()
    pygame.display.set_caption("SIM")

    
  def variables_init(self, resolution = 0.01):
    self.colors = {"BLUE":(0, 0, 255), "RED":(255, 0, 0), "GREEN":(0, 255, 0),
                    "BLACK":(0, 0, 0), "WHITE":(255, 255, 255)}
    
    self.map = Map()
    
    self.gui_resolution = resolution # [m/pix]
    self.gui_converter = CoordinateConverter(0.0, self.map.height * self.gui_resolution, self.gui_resolution)

    self.robot_diameter = 0.355 # [m]
    self.robot_radio = self.robot_diameter / 2.0 # [m]
    self.robot_radio_pix = int(self.robot_radio/self.gui_resolution)
    
    self.robot = Robot(radius = self.robot_radio_pix)

    self.state = "idle_mode"
    self.run = {"idle_mode":self.idle_mode, "set_robot_mode": self.set_robot_mode, 
                "set_robot_pose_mode": self.set_robot_pose_mode, "set_robot_yaw_mode": self.set_robot_yaw_mode,
                "add_wall_mode": self.add_wall_mode, "write_wall_mode": self.write_wall_mode, 
                "delete_wall_mode": self.delete_wall_mode}
    
    self.get_dist = lambda p1, p2: np.linalg.norm(np.array(p1) - np.array(p2))

    self.font = pygame.font.Font('freesansbold.ttf', 16)
    self.text_active = False

    self.wall_list = []
    self.wall_dist_threshold = 15 # pix
    self.map_changed = False


  def setup_screen(self):
    self.display_surf = pygame.display.set_mode((self.map.width, self.map.height))
    self.map.draw(self.display_surf)

  
  def update_screen(self):
    self.run[self.state]()

    self.map.draw(self.display_surf)
    
    if len(self.wall_list) > 0:
      self.draw_walls()
      
    if self.map_changed:
      self.map.update_map(self.display_surf)
      self.map_changed = False
    
    self.robot.draw(self.display_surf)
    
    if self.text_active:
      self.display_surf.blit(self.text, self.text_rect)
    
    pygame.display.update()
  

  def idle_mode(self):

    for event in pygame.event.get():              
      if event.type == QUIT:
          pygame.quit()
          sys.exit()

      if event.type == KEYDOWN:
    
        if pygame.key.name(event.key) == "p":
          self.state = "set_robot_mode"
          pygame.mouse.set_cursor(pygame.SYSTEM_CURSOR_HAND)
        
        elif pygame.key.name(event.key) == "w":
          self.state = "add_wall_mode"
          pygame.mouse.set_cursor(pygame.SYSTEM_CURSOR_CROSSHAIR)

        elif pygame.key.name(event.key) == "d":
          self.state = "delete_wall_mode"
          pygame.mouse.set_cursor(pygame.SYSTEM_CURSOR_NO)

  
  def set_robot_mode(self):
    for event in pygame.event.get():
      
      if event.type == MOUSEBUTTONDOWN and event.button == 1:
        self.mouse_start_pos = event.pos
        self.text_active = True
        
        
        if self.get_dist(event.pos, (self.robot.x, self.robot.y)) < self.robot.radius:
          self.state = "set_robot_pose_mode"
          self.set_robot_pose_text()
        
        else:
          self.state = "set_robot_yaw_mode"
          self.create_text('%.2f [rad]' % self.robot.yaw)
        
        break
    

  def set_robot_pose_mode(self):
    events = pygame.event.get()
    evnts_types =list(map(lambda event: event.type, events))

    if MOUSEBUTTONUP not in evnts_types:
      move_events = list(filter(lambda event: event.type == MOUSEMOTION, events))
      
      if len(move_events) > 0:
        mouse_end_pos = move_events[-1].pos
        delta_x, delta_y = mouse_end_pos[0] - self.mouse_start_pos[0], mouse_end_pos[1] - self.mouse_start_pos[1]
        
        self.robot.x += delta_x
        self.robot.y += delta_y
        self.robot.dir_x += delta_x
        self.robot.dir_y += delta_y
        
        self.mouse_start_pos = mouse_end_pos
        self.set_robot_pose_text()

    else:
      self.back_to_idle()
  

  def set_robot_pose_text(self):
    x_m, y_m = self.gui_converter.pixel2metric(self.robot.x, self.robot.y)
    self.create_text('(%.3f, %.3f) [m]' % (x_m, y_m))


  def set_robot_yaw_mode(self):
    events = pygame.event.get()
    evnts_types =list(map(lambda event: event.type, events))

    if MOUSEBUTTONUP not in evnts_types:
      move_events = list(filter(lambda event: event.type == MOUSEMOTION, events))
      
      if len(move_events) > 0:
        mouse_end_pos = move_events[-1].pos
        self.robot.yaw = np.arctan2(-(mouse_end_pos[1] - self.robot.y), mouse_end_pos[0] - self.robot.x)
        self.create_text('%.2f [rad]' % self.robot.yaw)

    else:
      self.back_to_idle()
  
  
  def add_wall_mode(self):
    for event in pygame.event.get():
      
      if event.type == MOUSEBUTTONDOWN and event.button == 1:
        self.mouse_start_pos = event.pos
        self.wall_list.append([self.mouse_start_pos, self.mouse_start_pos])
        self.text_active = True

        self.state = "write_wall_mode"
        self.create_text('0 [m]')
        break


  def write_wall_mode(self):  
    events = pygame.event.get()
    evnts_types =list(map(lambda event: event.type, events))

    if MOUSEBUTTONUP not in evnts_types:
      move_events = list(filter(lambda event: event.type == MOUSEMOTION, events))
     
      if len(move_events) > 0:
        mouse_end_pos = move_events[-1].pos
        delta_x = abs(mouse_end_pos[0]-self.mouse_start_pos[0])*self.gui_converter.resolution
        delta_y = abs(mouse_end_pos[1]-self.mouse_start_pos[1])*self.gui_converter.resolution
        self.create_text('%.3f [m]' % np.hypot(delta_x, delta_y))
        
        self.wall_list[-1][1] = mouse_end_pos
    
    else:
      self.map_changed = True
      self.back_to_idle()


  def delete_wall_mode(self):
    if len(self.wall_list) > 0:
      for event in pygame.event.get():
        
        if event.type == MOUSEBUTTONDOWN and event.button == 1:
          mouse_pos = event.pos
        
          for index, wall in enumerate(self.wall_list):
            wall_x = np.array((wall[0][0], wall[1][0]))
            wall_y = np.array((wall[0][1], wall[1][1]))

            if np.min(wall_x) - self.wall_dist_threshold < mouse_pos[0] < np.max(wall_x) + self.wall_dist_threshold and \
               np.min(wall_y) - self.wall_dist_threshold < mouse_pos[1] < np.max(wall_y) + self.wall_dist_threshold:
              
              if wall[1][0] == wall[0][0] and wall[1][1] == wall[0][1]:
                dist = self.get_dist(wall[0], mouse_pos)
              
              elif (wall[1][0] - wall[0][0]) < (wall[1][1] - wall[0][1]):
                m = (wall[1][0] - wall[0][0])/(wall[1][1] - wall[0][1]) # m = (x2-x1)/(y2-y1) x = y*m**(-1) + n
                n = wall[1][0] - m*wall[1][1] # n = x2 - m*y2
                x = m*mouse_pos[1] + n
               
                dist = self.get_dist((x, mouse_pos[1]), mouse_pos)

              else:
                m = (wall[1][1] - wall[0][1])/(wall[1][0] - wall[0][0]) # m = (y2-y1)/(x2-x1); y = m*x + n
                n = wall[1][1] - m*wall[1][0] # n = y2 - m*x2 
                y = m*mouse_pos[0] + n
                
                dist = self.get_dist((mouse_pos[0], y), mouse_pos)
              

              if dist < self.wall_dist_threshold:
                self.wall_list.pop(index)
                self.map_changed = True
                break
          
          self.back_to_idle()
          break

    else:
      self.back_to_idle()
    
  
  def create_text(self, text):
    self.text = self.font.render(text, True, self.colors["RED"], self.colors["WHITE"])
    self.text_rect = self.text.get_rect()
    self.text_rect.center = (self.map.width//2, 30)


  def draw_walls(self):
    for wall_pts in self.wall_list:
          pygame.draw.line(self.display_surf, self.colors["BLACK"], wall_pts[0], wall_pts[1], self.map.wall_thick)


  def back_to_idle(self):
    self.state = "idle_mode"
    self.text_active = False
    pygame.mouse.set_cursor(pygame.SYSTEM_CURSOR_ARROW)
  

def main():
    FPS = 20
    FramePerSec = pygame.time.Clock()
    
    pgg = PyGameGUI()

    while True:
      pgg.update_screen()
      FramePerSec.tick(FPS)

   
if __name__ == '__main__':
    main()