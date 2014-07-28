#!/usr/bin/env python

# readLogData.py - code to read Aerospace Robotics SLAMbot project log file
# 
# Copyright (C) 2014 Michael Searing
# 
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

GPL = "breezySLAM_XBEE.py Copyright (C) 2014 Michael Searing \n\
This program comes with ABSOLUTELY NO WARRANTY; for details type [tbd]. \n\
This is free software, and you are welcome to redistribute it \n\
under certain conditions; type [tbd] for details."
print(GPL)

import sys
print("Python {}.{}.{}".format(*sys.version_info[0:3]))

from breezyslam.algorithms import RMHC_SLAM
from breezyslam.components import Laser

if sys.version_info[0] < 3: # 2.x
  print("Getting Python 2 modules")
  import Tkinter as tk
  from tkMessageBox import askokcancel
else: # 3.x
  print("Getting Python 3 modules")
  import tkinter as tk
  from tkinter.messagebox import askokcancel
  def raw_input(inStr): return input(inStr)

import tkFont

import numpy as np
from scipy.ndimage.interpolation import rotate

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2TkAgg
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

from sys import exit

# note that Data.saveImage() imports PIL and subprocess for map image saving and viewing

logfilename = 'data_24JUL14.log'

# User preferences (default to True)
USE_ODOMETRY = False
INTERNAL_MAP = True

# Macros (here's me sort of wishing this were C++...)
def float2int(x):
  return int(0.5 + x)

# Packet constants (shared with Arduino)
DIST_MAX = 6000; # maximum distance
ANG_MIN = 0
ANG_MAX = 360; # maximum scan angle

# Map constants
MAP_SIZE_M = 14 # size of region to be mapped [m]
VIEW_SIZE_M = 6 # default size of region to be shown in display [m]
INSET_SIZE_M = 3 # size of relative map
MAP_RES_PIX_PER_M = 100 # number of pixels of data per meter [pix/m]
MAP_DEPTH = 10 # depth of data points on map (levels of certainty)

# GUI constants
DATA_RATE = 50 # minimum time between updating data from lidar [ms]
MAP_RATE = 250 # minimum time between updating map [ms]

# Robot constants
REV_2_TICK = 1000.0/3 # encoder ticks per wheel revolution
WHEEL_DIAMETER = 58.2
WHEEL_BASE = 177.0 # length of treads [mm]
WHEEL_TRACK = 190.0 # separation of treads [mm]

# Laser constants
SCAN_SIZE = 360 # number of points per scan
SCAN_RATE_HZ = 1980.0/360 # 1980points/sec * scan/360points [scans/sec]
SCAN_DETECTION_ANGLE = ANG_MAX
SCAN_DISTANCE_NO_DETECTION_MM = DIST_MAX
SCAN_DETECTION_MARGIN = 0
LASER_OFFSET_MM = 35 # this value is negative what it should be # update() returns LIDAR unit position

# BreezySLAM map constants
MAP_SIZE_PIXELS = MAP_SIZE_M*MAP_RES_PIX_PER_M # number of pixels across the entire map
MAP_QUALITY = 50
HOLE_WIDTH_MM = 200
RANDOM_SEED = 0xabcd

# Map constants (derived)
INSET_SIZE_PIX = INSET_SIZE_M*MAP_RES_PIX_PER_M # number of pixels across the inset map
MAP_CENTER_PIX = MAP_SIZE_PIXELS/2 # indices of map center []
MM_2_PIX = MAP_RES_PIX_PER_M/1000.0 # [pix/mm]
DEG_2_RAD = np.pi/180.0
ROBOT_DEPTH = MAP_DEPTH + 1 # value of robot in map storage

# Robot constants (derived)
REV_2_MM = np.pi*WHEEL_DIAMETER # circumference [mm]
TICK_2_MM = REV_2_MM/REV_2_TICK # [mm/tick]
TREAD_ERROR = 0.90 # continuous tread slips this much relative to slip of wheels (experimental)
ANGULAR_FLUX = TREAD_ERROR*((WHEEL_BASE/WHEEL_TRACK)**2+1); # tread slippage [] # our math assumed wheels, hence TREAD_ERROR
ROT_2_MM = np.pi*WHEEL_TRACK # circumference of wheel turning circle [mm]
ROT_2_DEG = 360.0 # [deg]
REV_2_DEG = ROT_2_DEG*(REV_2_MM/ROT_2_MM) # degrees of rotation per wheel revolution [deg]
TICK_2_DEG = REV_2_DEG/REV_2_TICK * 1.0/ANGULAR_FLUX # [deg/tick]

class Root(tk.Tk): # Tkinter window, inheriting from Tkinter module
  # init draws and displays window, creates data matrix, and calls updateData and updateMap
  # updateData calls getScanData, and updates the slam object and data matrix with this data, and loops to itself
  # updateMap draws a new map, using whatever data is available, and loops to itself
  # getScanData pulls LIDAR data directly from the serial port and does preliminary processing
  # resetAll recreates slam object to remove previous data it and wipes current data matrix

  def __init__(self):
    tk.Tk.__init__(self) # explicitly initialize base class and create window
    self.protocol("WM_DELETE_WINDOW", self.closeWin) # control what happens when a window is closed externally (e.g. by the 'x')
    self.geometry('+100+100') # position windows 100,100 pixels from top-left corner
    self.wm_title("Aerospace Robotics LIDAR Viewer") # name window
    self.lower() # bring terminal to front

    # Initialize objects
    self.statusStr = tk.StringVar() # status of serThread
    self.statusStr.set("Initializing")
    self.resetting = False
    self.fileIndex = 0
    self.fileContents = [[int(el) for el in rawline.split(' ')] for rawline in dataFile.read().strip().split('\n')]

    # Start loops
    self.initUI() # create all the pretty stuff in the Tkinter window
    self.restartAll(rootInit=True) # contains all initialization that needs to be re-done in case of a soft reset
    print("Each pixel is " + str(round(1000.0/MAP_RES_PIX_PER_M,1)) + "mm, or " + str(round(1000.0/MAP_RES_PIX_PER_M/25.4,2)) + "in.")
    self.lift() # bring tk window to front if initialization finishes

  def initUI(self):
    # current (and only) figure
    self.fig = plt.figure(figsize=(9, 5), dpi=131.2) # create matplotlib figure (dpi calculated from $ xrandr)
    gs = gridspec.GridSpec(1,3) # layout of plots in figure

    # plot color settings
    cmap = plt.get_cmap("binary") # opposite of "gray"
    cmap.set_over("red") # robot is set to higher than MAP_DEPTH

    # subplot 1 (stationary map)
    self.ax1 = plt.subplot(gs[0,:2]) # add plot 1 to figure
    self.ax1.set_title("Region Map") # name and label plot
    self.ax1.set_xlabel("X Position [m]")
    self.ax1.set_ylabel("Y Position [m]")
    dummyInitMat = np.zeros((2,2), dtype=np.uint8) # need to plot something to setup tkinter before dependent objects
    self.myImg1 = self.ax1.imshow(dummyInitMat, interpolation="none", cmap=cmap, vmin=0, vmax=MAP_DEPTH, # plot data
              extent=[-MAP_SIZE_M/2, MAP_SIZE_M/2, -MAP_SIZE_M/2, MAP_SIZE_M/2]) # extent sets labels by matching limits to edges of matrix
    self.ax1.set_xlim(-VIEW_SIZE_M/2, VIEW_SIZE_M/2) # pre-zoom image to defined default VIEW_SIZE_MM
    self.ax1.set_ylim(-VIEW_SIZE_M/2, VIEW_SIZE_M/2)

    # colorbar
    self.cbar = self.fig.colorbar(self.myImg1, orientation="vertical") # create colorbar

    # subplot 2 (relative map)
    self.ax2 = plt.subplot(gs[0,2]) # add plot 2 to figure
    self.ax2.set_title("Robot Environs") # name and label plot
    self.ax2.set_xlabel("", family="monospace")
    self.myImg2 = self.ax2.imshow(dummyInitMat, interpolation="none", cmap=cmap, vmin=0, vmax=MAP_DEPTH, # plot data
              extent=[-INSET_SIZE_M/2, INSET_SIZE_M/2, -INSET_SIZE_M/2, INSET_SIZE_M/2])

    # turn figure data into matplotlib draggable canvas
    self.canvas = FigureCanvasTkAgg(self.fig, master=self) # tkinter interrupt function
    self.canvas.draw()
    self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=1) # put figure at top of window

    # add matplotlib toolbar for easy navigation around map
    NavigationToolbar2TkAgg(self.canvas, self).update() # tkinter interrupt function
    self.canvas._tkcanvas.pack(side="top", fill=tk.BOTH, expand=1) # actually goes on bottom...not sure why

    # bind keyboard inputs to functions
    self.bind('<Escape>', lambda event: self.closeWin()) # tkinter interrupt function
    self.bind('r', lambda event: self.restartAll()) # tkinter interrupt function

    # create buttons
    tk.Button(self, text="Quit (esc)", command=self.closeWin).pack(side="left", padx=5, pady=5) # tkinter interrupt function
    tk.Button(self, text="Restart (r)", command=self.restartAll).pack(side=tk.LEFT, padx = 5) # tkinter interrupt function
    monospaceFont = tkFont.Font(family="Courier", size=12)
    tk.Label(self, textvariable=self.statusStr, font=monospaceFont).pack(side="left", padx=5, pady=5)
    tk.Button(self, text="Save Map", command=self.saveImage).pack(side=tk.LEFT, padx=5) # tkinter interrupt function

  def restartAll(self, rootInit=False, funcStep=0): # two-part soft reset function
    if funcStep == 0:
      if rootInit: # don't need to stop currently running loops, so go right to second half of restart
        self.restartAll(rootInit=rootInit, funcStep=1)
      else: # reset called during program
        self.resetting = True # stop currently running loops
        self.after(2000, lambda: self.restartAll(rootInit=rootInit, funcStep=1)) # release processor to let it wrap things up elsewhere
      return

    elif funcStep == 1:
      if not rootInit: # reset called during program
        self.resetting = False
        self.fileIndex = 0
      self.data = Data()
      self.slam = Slam()
      self.updateData(init=True) # pull data from queue, put into data matrix
      self.updateMap() # draw new data matrix

  def closeWin(self):
    if askokcancel("Quit?", "Are you sure you want to quit?"):
      print("Closing program")
      self.quit() # kills interpreter (necessary for some reason)

  def saveImage(self): # function prototype until data is initialized
    self.data.saveImage()

  def updateData(self, init=False):
    self.dataInit = init
    if self.fileIndex < len(self.fileContents): # ready to pull new scan data
      self.getScanData() # pull data from log file

      # update robot position
      if init: self.slam.prevEncPos = self.slam.currEncPos # set both values the first time through
      self.data.getRobotPos(self.slam.updateSlam(self.data.currScan), init=init) # send data to slam to do stuff

      self.statusStr.set('Scan = {0:4}'.format(self.fileIndex))

      if init: init = False # initial data gathered successfully

    if not self.resetting: self.after(DATA_RATE, lambda: self.updateData(init=init)) # tkinter interrupt function

  def updateMap(self):
    if not self.dataInit: # wait until first data update to update map
      self.data.drawMap(self.slam.breezyMap if INTERNAL_MAP else None) # draw points, using updated slam, on the data matrix # 16ms
      self.data.drawInset() # new relative map
      self.data.drawRobot(self.data.matrix, self.data.robot_pix) # new robot position

      self.myImg1.set_data(self.data.matrix) # 15ms
      self.myImg2.set_data(self.data.insetMatrix) # 15ms
      self.ax2.set_xlabel('X = {0:6.1f}; Y = {1:6.1f};\nHeading = {2:6.1f}'.format(*self.data.robot_rel))
      self.canvas.draw() # 400ms

    if not self.resetting: self.after(MAP_RATE, lambda: self.updateMap()) # tkinter interrupt function

  def getScanData(self):
    scan = self.fileContents[self.fileIndex]
    self.slam.currEncPos = scan[0:3]
    self.data.currScan = scan[3:]
    self.fileIndex += 1


class Slam(RMHC_SLAM): # new
  # updateSlam takes LIDAR data and uses BreezySLAM to calculate the robot's new position
  # getVelocities takes the encoder data (lWheel, rWheel, timeStamp) and finds change in translational and angular position and in time

  def __init__(self):
    laser = Laser(SCAN_SIZE, \
                  SCAN_RATE_HZ, \
                  SCAN_DETECTION_ANGLE, \
                  SCAN_DISTANCE_NO_DETECTION_MM, \
                  SCAN_DETECTION_MARGIN, \
                  LASER_OFFSET_MM)
    RMHC_SLAM.__init__(self, \
                       laser, \
                       MAP_SIZE_PIXELS, \
                       MAP_SIZE_M, \
                       MAP_QUALITY, \
                       HOLE_WIDTH_MM, \
                       RANDOM_SEED)
    self.prevEncPos = () # robot encoder data
    self.currEncPos = () # left wheel [ticks], right wheel [ticks], timestamp [ms]

    if INTERNAL_MAP: self.breezyMap = bytearray(MAP_SIZE_PIXELS**2) # initialize map array for BreezySLAM's internal mapping

  def updateSlam(self, scan): # 15ms
    # note that breezySLAM switches the x- and y- axes (their x is forward, 0deg; y is right, +90deg)
    scan[:] = [scan[i-180] for i in range(SCAN_SIZE)]
    self.update(scan, self.getVelocities() if USE_ODOMETRY else None) # update slam information using particle filtering on LIDAR scan data
    x, y, theta = self.getpos()

    if INTERNAL_MAP: self.getmap(self.breezyMap)

    return (y, x, theta)

  def getVelocities(self):
    dLeft, dRight, dt = [curr - prev for (curr,prev) in zip(self.currEncPos, self.prevEncPos)]
    self.prevEncPos = self.currEncPos

    # overflow correction:
    if dLeft > 2**15: dLeft -= 2**16-1 # signed short
    elif dLeft < -2**15: dLeft += 2**16-1

    if dRight > 2**15: dRight -= 2**16-1 # signed short
    elif dRight < -2**15: dRight += 2**16-1

    if dt < -2**15: dt += 2**16-1 # unsigned short # time always increases, so only check positive overflow

    dxy = TICK_2_MM * (dLeft + dRight)/2 # forward change in position
    dtheta = TICK_2_DEG * (dLeft - dRight)/2 # positive theta is clockwise

    return dxy, dtheta, dt/1000.0 # [mm], [deg], [s]


class Data():
  # init creates data matrix and information vectors for processing
  # drawPoints adds scan data to data matrix
  # drawInset adds scan data to inset matrix
  # drawRobot adds robot position to data matrix, generally using slam to find this position
  # saveImage uses PIL to write an image file from the data matrix

  def __init__(self):    
    self.matrix = np.zeros((MAP_SIZE_PIXELS, MAP_SIZE_PIXELS), dtype=np.uint8) # initialize data matrix
    self.insetMatrix = np.zeros((INSET_SIZE_PIX, INSET_SIZE_PIX), dtype=np.uint8) # initialize inset matrix
    self.currScan = [] # current scan data
    self.robot_init = () # x [mm], y [mm], th [deg], defined from lower-left corner of map
    self.robot_rel = () # robot location relative to start position
    self.robot_pix = () # robot pixel location, relative to upper-left (0,0) of image matrix

    self.robotSprite = np.array([[0,0,0,0,0,1,0,0,0,0,0], # shape of robot on map
                                 [0,0,0,0,0,1,0,0,0,0,0],
                                 [0,0,0,0,1,1,1,0,0,0,0],
                                 [0,0,0,0,1,1,1,0,0,0,0],
                                 [0,0,0,1,1,0,1,1,0,0,0],
                                 [0,0,0,1,1,0,1,1,0,0,0],
                                 [0,0,1,1,0,0,0,1,1,0,0],
                                 [0,0,1,1,0,0,0,1,1,0,0],
                                 [0,1,1,0,0,0,0,0,1,1,0],
                                 [0,1,1,1,1,1,1,1,1,1,0],
                                 [1,1,1,1,1,1,1,1,1,1,1],
                                 [0,0,0,0,0,0,0,0,0,0,0],
                                 [0,0,0,0,0,0,0,0,0,0,0]])

  def getRobotPos(self, curr_pos, init=False):
    if init: self.robot_init = curr_pos
    self.robot_rel = tuple([curr-init for (curr,init) in zip(curr_pos,self.robot_init)]) # displacement from start position (center of map)
    xpix = float2int(curr_pos[0]*MM_2_PIX) # robot wrt map center (mO) + mO wrt matrix origin (xO)  = robot wrt xO [pix]
    ypix = MAP_SIZE_PIXELS-float2int(curr_pos[1]*MM_2_PIX) # y is negative because pixels increase downwards (+y_mm = -y_pix = -np rows)
    self.robot_pix = (xpix, ypix, self.robot_rel[2])

  def drawMap(self, breezyMap): # 7ms
    if INTERNAL_MAP: # get data from internal BreezySLAM map
      dataMat = 255-np.flipud(np.resize(np.array(breezyMap, dtype=np.uint8),(MAP_SIZE_PIXELS,MAP_SIZE_PIXELS)).T) # 7ms
      self.matrix = (MAP_DEPTH/255.0*dataMat).astype(np.uint8) # 8ms

    else: # use scan data directly
      for (dist,ang) in zip(self.currScan, range(len(SCAN_SIZE))):
        # pixel location of scan point # point wrt robot + robot wrt x0 = point wrt x0
        x_pix = float2int( MAP_CENTER_PIX + ( self.robot_rel[0] + dist*np.sin((ang+self.robot_rel[2])*DEG_2_RAD) )*MM_2_PIX )
        y_pix = float2int( MAP_CENTER_PIX - ( self.robot_rel[1] + dist*np.cos((ang+self.robot_rel[2])*DEG_2_RAD) )*MM_2_PIX )
        try:
          self.matrix[y_pix, x_pix] += self.matrix[y_pix, x_pix] < MAP_DEPTH # increment value at location if below maximum value
        except IndexError:
          print("scan out of bounds")

  def drawInset(self):
    x, y = self.robot_pix[0:2] # indices of center of robot in main map
    raw = int(INSET_SIZE_PIX*0.75) # half the size of the main map segment to capture
    rad = INSET_SIZE_PIX/2 # half the size of the final segment

    mapChunk = self.matrix[y-raw:y+raw, x-raw:x+raw]
    self.insetMatrix = rotate(mapChunk, self.robot_rel[2], output=np.uint8, reshape=False)[raw-rad:raw+rad, raw-rad:raw+rad]
    self.drawRobot(self.insetMatrix, (INSET_SIZE_PIX/2,INSET_SIZE_PIX/2,0))

  def drawRobot(self, mapObject, pos):
    robotMat = rotate(self.robotSprite, -pos[2], output=np.uint8)
    hgt = (robotMat.shape[0]-1)/2 # indices of center of robot
    wid = (robotMat.shape[1]-1)/2
    mapObject[pos[1]-hgt:pos[1]+hgt+1, pos[0]-wid:pos[0]+wid+1][robotMat.astype(bool)] = ROBOT_DEPTH

  def saveImage(self):
    from PIL import Image # don't have PIL? sorry (try pypng)

    imgData = self.matrix # get map
    robot = imgData > MAP_DEPTH # save location of robot
    imgData = MAP_DEPTH - imgData # invert colors to make 0 black and MAP_DEPTH white
    GB = np.where(robot, 0, 255.0/MAP_DEPTH*imgData).astype(np.uint8) # scale map data and assign to red, green, and blue layers
    R = np.where(robot, 255, GB).astype(np.uint8) # add robot path to red layer

    im = Image.fromarray(np.dstack((R,GB,GB))) # create image from depth stack of three layers
    filename = str(MAP_RES_PIX_PER_M)+"_pixels_per_meter.png" # filename is map resolution
    im.save(filename) # save image
    print("Image saved to " + filename)

    import subprocess

    subprocess.call(["eog", filename]) # open with eye of gnome


if __name__ == '__main__':
  dataFile = open(logfilename,'r')
  root = Root() # create Tkinter window, containing entire App
  root.mainloop() # start Tkinter loop
  dataFile.close()

