import copy
import yaml
import cv2
import numpy as np

from PIL import Image as PILImage, ImageDraw as PILImageDraw, ImageTk
from tkinter import *
from tkinter import filedialog as tkFileDialog

from utils import CoordinateConverter
from os import path


GRAPH_FONT=None
#GRAPH_FONT=('Helvetica', 20)

class CanvasMode(object):

  def __init__(self, canvas):
    pass

  def click1(self, event):
    pass

  def click1_motion(self, event):
    pass

  def click1_off(self, event):
    pass

  def double_click1(self, event):
    pass


class IdleMode(CanvasMode):
  pass


class SetRobotPoseMode(CanvasMode):

  def __init__(self, canvas, converter, set_initial_pose_cb):
    self.canvas = canvas
    self.width = self.canvas.pilimage.size[0]

    self.converter = converter
    self.set_initial_pose_cb = set_initial_pose_cb
    self.x, self.y, self.yaw = 0, 0, 0

  def click1(self, event):
    canvasx, canvasy = self.canvas.canvasx(event.x), self.canvas.canvasy(event.y)
    self.x, self.y, self.yaw = self.get_current_pose()

    current_tag = self.canvas.itemcget(CURRENT, 'tags').split(' ')[0]
    
    r_coords = self.canvas.coords('robot')
    radius = (r_coords[2] - r_coords[0])/2

    if current_tag.startswith('robot'):
      self.x, self.y = canvasx, canvasy
      r_coords = [self.x+radius, self.y+radius, self.x-radius, self.y-radius]
      self.canvas.coords('robot', *r_coords)
      
      d_coords = self.canvas.coords('robot_direction')
      deltax, deltay = self.x - d_coords[0], self.y - d_coords[1]
      d_coords[0] += deltax
      d_coords[1] += deltay
      d_coords[2] += deltax
      d_coords[3] += deltay
      self.canvas.coords('robot_direction', *d_coords)
    
      x_m, y_m = self.converter.pixel2metric(self.x, self.y)
      data_str = '(%.3f, %.3f) [m]' % (x_m, y_m)

    else:
      self.set_robot_direction(r_coords, radius, canvasx, canvasy)
      data_str = '%.2f [rad]' % self.yaw
   
    i = self.canvas.create_text(self.width/2, 20, text = data_str, fill = 'blue', tags = 'indicator_text', font = GRAPH_FONT)
    r = self.canvas.create_rectangle(self.canvas.bbox(i), fill = 'white', outline = 'white', tags = 'indicator_bg')
    self.canvas.tag_lower(r, i)

  def click1_motion(self, event):
    canvasx, canvasy = self.canvas.canvasx(event.x), self.canvas.canvasy(event.y)
    
    current_tag = self.canvas.itemcget(CURRENT, 'tags').split(' ')[0]

    r_coords = self.canvas.coords('robot')

    if current_tag.startswith('robot'):
      deltax, deltay = canvasx - self.x, canvasy - self.y
      r_coords[0] += deltax
      r_coords[1] += deltay
      r_coords[2] += deltax
      r_coords[3] += deltay
      self.canvas.coords('robot', *r_coords)
     
      d_coords = self.canvas.coords('robot_direction')
      d_coords[0] += deltax
      d_coords[1] += deltay
      d_coords[2] += deltax
      d_coords[3] += deltay
      self.canvas.coords('robot_direction', *d_coords)
      
      self.x, self.y = canvasx, canvasy
      
      x_m, y_m = self.converter.pixel2metric(self.x, self.y)
      data_str = '(%.3f, %.3f) [m]' % (x_m, y_m)
      
    else:
      radius = (r_coords[2] - r_coords[0])/2
      self.set_robot_direction(r_coords, radius, canvasx, canvasy)
      data_str = '%.2f [rad]' % self.yaw

    self.canvas.itemconfig('indicator_text', text = data_str)
    self.canvas.coords('indicator_bg', self.canvas.bbox('indicator_text'))

  def click1_off(self, event):
    self.set_initial_pose_cb([self.x, self.y, self.yaw])
    self.canvas.delete('indicator_text')
    self.canvas.delete('indicator_bg')

  def get_current_pose(self):
    r_coords = self.canvas.coords('robot')
    radius = (r_coords[2] - r_coords[0])/2
    x, y = r_coords[0] + radius, r_coords[1] + radius
    
    d_coords = self.canvas.coords('robot_direction')
    yaw = np.arctan2(-(d_coords[3] - d_coords[1]), d_coords[2] - d_coords[0])
    return x, y, yaw
  
  def set_robot_direction(self, r_coords, radius, canvasx, canvasy):
    x, y = r_coords[0] + radius, r_coords[1] + radius
    self.yaw = np.arctan2(-(canvasy - y), canvasx - x)
    x1, y1 = int(x + radius * np.cos(self.yaw)), int(y - radius * np.sin(self.yaw))
    coords = [x, y, x1, y1]
    self.canvas.coords('robot_direction', *coords)


class AddWallMode(CanvasMode):

  def __init__(self, canvas, converter, id_offset = 0):
    self.canvas = canvas

    self.converter = converter
    self.width = self.canvas.pilimage.size[0]
    self.id_offset = id_offset
    self.x, self.y = 0, 0
    self.current_tag = ''

  def click1(self, event):
    self.x, self.y = self.canvas.canvasx(event.x), self.canvas.canvasy(event.y)
    self.current_tag = 'wall_' + str(self.id_offset)
    self.canvas.create_line(self.x, self.y, self.x, self.y, width = 3, fill = 'black', tags = self.current_tag)

    i = self.canvas.create_text(self.width/2, 20, text = '0.0 [m]', fill = 'blue', tags = 'indicator_text', font = GRAPH_FONT)
    r = self.canvas.create_rectangle(self.canvas.bbox(i), fill = 'white', outline = 'white', tags = 'indicator_bg')
    self.canvas.tag_lower(r, i)

  def click1_motion(self, event):
    canvasx, canvasy  = self.canvas.canvasx(event.x), self.canvas.canvasy(event.y)
    deltax, deltay = canvasx - self.x, canvasy - self.y
    coords = self.canvas.coords(self.current_tag)
    coords[2] += deltax
    coords[3] += deltay
    self.canvas.coords(self.current_tag, *coords)
    self.x, self.y = canvasx, canvasy
   
    width, height = abs(coords[2]-coords[0])*self.converter.resolution, abs(coords[3]-coords[1])*self.converter.resolution
    
    wall_legth_str = '%.3f [m]' % np.hypot(width, height)
    self.canvas.itemconfig('indicator_text', text = wall_legth_str)
    self.canvas.coords('indicator_bg', self.canvas.bbox('indicator_text'))

  def click1_off(self, event):
    self.id_offset += 1
    self.canvas.delete('indicator_text')
    self.canvas.delete('indicator_bg')

  def reset(self):
    self.id_offset = 0


class DeleteWallMode(CanvasMode):

  def __init__(self, canvas):
    self.canvas = canvas

  def click1(self, event):
    current_tag = self.canvas.itemcget(CURRENT, 'tags').split(' ')[0]
    if current_tag.startswith('wall_'):
      self.canvas.delete(current_tag)


class WorldStateGUI(Frame):

  def __init__(self, node, width = 500, height = 290, resolution = 0.01):
    self.wall_thick = 3
    self.width, self.height = width + 2 * self.wall_thick, height + 2 * self.wall_thick
    self.gui_resolution = resolution # [m/pix]
    self.map_resolution = resolution # [m/pix]
    self.robot_diameter = 0.355 # [m]
    self.robot_radio = self.robot_diameter / 2.0 # [m]
    self.robot_radio_pix = int(self.robot_radio / self.gui_resolution)
    self.gui_converter = CoordinateConverter(0.0, self.height * self.gui_resolution, self.gui_resolution)
    self.map_converter = CoordinateConverter(0.0, self.height * self.map_resolution, self.map_resolution)
    self.node = node

    self.root = Tk()
    self.root.geometry('%dx%d' % (self.width, self.height))
    self.root.title('World State')
    self.root.resizable(False, False)
    
    Frame.__init__(self, self.root, width = self.width, height = self.height)
    self.grid(row = 0, column = 0)

    menubar = Menu(self.master)
    self.master.config(menu = menubar)
    fileMenu = Menu(menubar)
    fileMenu.add_command(label = "Open map ...", command = self.open_map)
    fileMenu.add_command(label = "Save map ...", command = self.save_map)
    fileMenu.add_command(label = "Exit", command = self.on_exit)
    menubar.add_cascade(label = "File", menu = fileMenu)
    toolsMenu = Menu(menubar)
    toolsMenu.add_command(label = "Reset", command = node.reset_state)
    menubar.add_cascade(label = "Tools", menu = toolsMenu)

    npimage = self.add_margin(255 * np.ones((height, width), dtype = np.uint8))
    self.canvas = Canvas(self, width = self.width, height = self.height, bg = '#FFFFFF')
    self.canvas.pack(side = LEFT, expand = True, fill = BOTH)
    self.canvas.pilimage = PILImage.fromarray(npimage)
    self.canvas.bgimage = ImageTk.PhotoImage(self.canvas.pilimage)
    self.canvas.create_image(0, 0, anchor = NW, image = self.canvas.bgimage, tags = 'backgroundimg')

    self.canvas.bind('<ButtonPress-1>', self.click1)
    self.canvas.bind('<ButtonRelease-1>', self.click1_off)
    self.canvas.bind('<B1-Motion>', self.click1_motion)
    self.canvas.bind('<Key>', self.key_pressed)
    self.canvas.configure(cursor = 'left_ptr')
    self.canvas.focus_set()

    self.statem = dict()
    self.statem['idle_mode'] = IdleMode(self.canvas)
    self.statem['set_robot_pose_mode'] = SetRobotPoseMode(self.canvas, self.gui_converter, self.node.send_initial_pose)
    self.statem['add_wall_mode'] = AddWallMode(self.canvas, self.gui_converter)
    self.statem['delete_wall_mode'] = DeleteWallMode(self.canvas)
    self.cstate = 'idle_mode'


  def add_margin(self, image):
    height, width = image.shape[:2]
    horizontal_wall = np.zeros((self.wall_thick, width), dtype = np.uint8)
    vertical_wall = np.zeros((height + 2 * self.wall_thick, self.wall_thick), dtype = np.uint8)
    image = np.concatenate((horizontal_wall, image, horizontal_wall), axis = 0)
    image = np.concatenate((vertical_wall, image, vertical_wall), axis = 1)
    return image


  def open_map(self):
    yamlfile = tkFileDialog.askopenfilename(title = 'Load Map', filetypes = [ ('YAML', ('*.yaml')) ])
    if len(yamlfile) == 0:
      return
    self.load_map(yamlfile)
    self.node.update_map()


  def save_map(self):
    outfile = tkFileDialog.asksaveasfile(title = 'Save map', filetypes = [('YAML', ('*.yaml'))], defaultextension = '.yaml')
    
    if outfile is None or len(outfile.name) == 0:
      return

    filebasename = path.splitext(outfile.name)[0]
    map_image = copy.copy(self.canvas.pilimage)
    map_image = map_image.convert('RGB')
    draw = PILImageDraw.Draw(map_image)
    itemList = self.canvas.find_all()
    for item in itemList:
      tag = self.canvas.gettags(item)[0]
      
      if tag.startswith('wall_'):
        coords = self.canvas.coords(item)
        params = ['fill', 'width']
        opt = dict()
        for p in params:
          opt[p] = self.canvas.itemcget(item, p)
        draw.line(coords, fill = opt['fill'], width = int(float(opt['width'])))
    
    if self.map_resolution != self.gui_resolution:
      factor = self.gui_resolution / self.map_resolution
      width = int(np.ceil(factor * map_image.size[0]))
      height = int(np.ceil(factor * map_image.size[1]))
      map_image = map_image.resize((width, height), resample = PILImage.NEAREST)
    
    map_image.save(filebasename + '.pgm')
    data = {
             'image' : path.basename(filebasename) + '.pgm',
             'resolution' : self.map_converter.resolution,
             'origin' : [self.map_converter.metric_zero_x, self.map_converter.metric_zero_y, 0.0],
             'occupied_thresh' : 0.65,
             'free_thresh' : 0.196,
             'negate' : 0
           }
    with open(filebasename + '.yaml', 'w') as fp:
      yaml.dump(data, fp)


  def click1(self, event):
    self.statem[self.cstate].click1(event)


  def click1_motion(self, event):
    self.statem[self.cstate].click1_motion(event)


  def click1_off(self, event):
    self.statem[self.cstate].click1_off(event)
    if self.cstate == 'add_wall_mode' or self.cstate == 'delete_wall_mode':
      self.node.update_map()
    if self.cstate != 'idle_mode':
      self.cstate = 'idle_mode'
      self.canvas.config(cursor = 'left_ptr')


  def key_pressed(self, event):
    if event.keysym == 'w' and self.cstate != 'add_wall_mode':
      self.cstate = 'add_wall_mode'
      self.canvas.config(cursor = 'pencil')

    elif event.keysym == 'w' and self.cstate == 'add_wall_mode':
      self.cstate = 'idle_mode'
      self.canvas.config(cursor = 'left_ptr')

    elif event.keysym == 'd' and self.cstate != 'delete_wall_mode':
      self.cstate = 'delete_wall_mode'
      self.canvas.config(cursor = 'X_cursor')

    elif event.keysym == 'd' and self.cstate == 'delete_wall_mode':
      self.cstate = 'idle_mode'
      self.canvas.config(cursor = 'left_ptr')

    elif event.keysym == 'p' and self.cstate != 'set_robot_pose_mode':
      self.cstate = 'set_robot_pose_mode'
      self.canvas.config(cursor = 'hand1')

    elif event.keysym == 'p' and self.cstate == 'set_robot_pose_mode':
      robot_pose = self.get_current_pose()
      self.node.send_initial_pose(robot_pose)
      self.cstate = 'idle_mode'
      self.canvas.config(cursor = 'left_ptr')


  def get_current_pose(self):
    r_coords = self.canvas.coords('robot')
    radius = (r_coords[2] - r_coords[0])/2
    x, y = r_coords[0] + radius, r_coords[1] + radius
    d_coords = self.canvas.coords('robot_direction')
    yaw = np.arctan2(-(d_coords[3] - d_coords[1]), d_coords[2] - d_coords[0])
    return x, y, yaw


  def update_robot_pose(self, yaw, pose):
    if self.cstate != 'set_robot_pose_mode':
      x, y = self.gui_converter.metric2pixel(pose.position.x, pose.position.y)
      x1, y1 = int(x + self.robot_radio_pix * np.cos(yaw)), int(y - self.robot_radio_pix * np.sin(yaw))

      if len(self.canvas.find_withtag('robot')) == 0:
        self.canvas.create_oval(x-self.robot_radio_pix,
                                 y-self.robot_radio_pix,
                                 x+self.robot_radio_pix,
                                 y+self.robot_radio_pix,
                                 outline = 'red',
                                 fill = 'red',
                                 tags = 'robot')
        
        self.canvas.create_line(x, y, x1, y1, fill = 'white', width = 2, tags = 'robot_direction')
      
      else:
        coords = [x-self.robot_radio_pix, y-self.robot_radio_pix, x+self.robot_radio_pix, y+self.robot_radio_pix]
        self.canvas.coords('robot', *coords)
        
        coords = [x, y, x1, y1]
        self.canvas.coords('robot_direction', *coords)


  def load_map(self, yaml_file):
    with open(yaml_file) as fd:
      metadata = yaml.safe_load(fd)

    if not path.isabs(metadata['image']):
      map_path = path.dirname(yaml_file)
      map_filename = path.basename(metadata['image'])
      map_file = path.join(map_path, map_filename)
    else:
      map_file = metadata['image']

    self.map_resolution = metadata['resolution'] # [m/pix]
    self.map_converter = CoordinateConverter(metadata['origin'][0], metadata['origin'][1], self.map_resolution)
    self.gui_converter = CoordinateConverter(metadata['origin'][0], metadata['origin'][1], self.gui_resolution)

    for st_name, st_object in self.statem.items():
      if hasattr(st_object, 'converter'):
        st_object.converter = self.gui_converter

    npimage = cv2.imread(map_file, cv2.IMREAD_GRAYSCALE)
    pilimage = PILImage.fromarray(npimage)
    
    if self.map_resolution != self.gui_resolution:
      factor = self.map_resolution / self.gui_resolution
      width = int(np.ceil(factor * pilimage.size[0]))
      height = int(np.ceil(factor * pilimage.size[1]))
      pilimage = pilimage.resize((width, height), resample = PILImage.NEAREST)
    
    self.width, self.height = pilimage.size
    bgimage = ImageTk.PhotoImage(pilimage)

    self.canvas.delete('robot')
    self.canvas.delete('robot_direction')
    self.canvas.delete('backgroundimg')
    self.root.geometry('%dx%d' % (self.width, self.height))
    # keep a reference to the image to avoid the image being garbage collected
    self.canvas.pilimage = pilimage
    self.canvas.bgimage = bgimage
    self.canvas.config(width = self.width, height = self.height)
    self.canvas.create_image(0, 0, anchor = NW, image = bgimage, tags = 'backgroundimg')

    # update map width 
    self.statem['set_robot_pose_mode'].width = self.canvas.pilimage.size[0]
    self.statem['add_wall_mode'].width = self.canvas.pilimage.size[0]


  def update_map(self):
    background_image = self.canvas.pilimage.copy()
    draw = PILImageDraw.Draw(background_image)
    item_list = self.canvas.find_all()
    for item in item_list:
      tag = self.canvas.gettags(item)[0]
      if tag.startswith('wall_'):
        coords = self.canvas.coords(item)
        params = ['fill', 'width']
        opt = dict()
        for p in params:
          opt[p] = self.canvas.itemcget(item, p)
        draw.line(coords, fill = opt['fill'], width = int(float(opt['width'])))
    
    return background_image


  def mainloop(self):
    self.root.mainloop()


  def sigint_handler(self, signum, frame):
    self.root.quit()
    self.root.update()


  def on_exit(self):
    self.quit()
