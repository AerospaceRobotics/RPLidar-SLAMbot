#!/usr/bin/env python

# baseStationMain.py - base station code for Aerospace Robotics SLAMbot project
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
This program comes with ABSOLUTELY NO WARRANTY. \n\
This is free software, and you are welcome to redistribute it \n\
under certain conditions; please cite the source."
print(GPL)

from sys import version_info
print("Python {}.{}.{}".format(*version_info[0:3]))
pythonSeries = version_info[0]


# User preferences
TALK_TO_XBEE = False
USE_ODOMETRY = True
INTERNAL_MAP = True
LOG_ALL_DATA = True
LOGFILE_NAME = "test" # do not include .log extension here

# Laser constants (shared with Arduino)
DIST_MIN = 100; # minimum distance
DIST_MAX = 6000; # maximum distance
ANG_MIN = 0; # minimum scan angle
ANG_MAX = 360; # maximum scan angle

# Map constants
MAP_SIZE_M = 16.0 # size of region to be mapped [m]
INSET_SIZE_M = 2.0 # size of relative map
MAP_RES_PIX_PER_M = 100 # number of pixels of data per meter [pix/m]

# Robot constants
REV_2_TICK = 1000.0/3 # encoder ticks per wheel revolution
WHEEL_DIAMETER = 58.2
WHEEL_BASE = 177.0 # length of treads [mm]
WHEEL_TRACK = 190.0 # separation of treads [mm]

# Robot constants (derived)
from numpy import pi as PI
REV_2_MM = PI*WHEEL_DIAMETER # circumference [mm]
MM_2_TICK = REV_2_TICK/REV_2_MM # [ticks/mm]
TICK_2_MM = REV_2_MM/REV_2_TICK # [mm/tick]
TREAD_ERROR = 0.90 # continuous tread slips this much relative to slip of wheels (experimental)
ANGULAR_FLUX = TREAD_ERROR*((WHEEL_BASE/WHEEL_TRACK)**2+1); # tread slippage [] # our math assumed wheels, hence TREAD_ERROR
ROT_2_MM = PI*WHEEL_TRACK # circumference of wheel turning circle [mm]
ROT_2_DEG = 360.0 # [deg]
REV_2_DEG = ROT_2_DEG*(REV_2_MM/ROT_2_MM) # degrees of rotation per wheel revolution [deg]
DEG_2_TICK = REV_2_TICK/REV_2_DEG * ANGULAR_FLUX # [ticks/deg]
TICK_2_DEG = REV_2_DEG/REV_2_TICK * 1.0/ANGULAR_FLUX # [deg/tick]


def main():
  # app window setup
  root = tk.Tk() # create tkinter window
  root.wm_title("Aerospace Robotics LIDAR Viewer") # name window
  root.geometry('+100+100') # position windows 100,100 pixels from top-left corner
  root.lower() # bring terminal to front

  # helper object setup
  data = Data()
  slam = Slam()
  queues = [Queue(), Queue(), Queue()]
  serThread = SerialThread(*queues)

  # create main app
  app = App(root, data, slam, queues, serThread)

  # Start loops
  app.serThread.start() # begin fetching data from serial port and processing it
  app.updateData() # pull data from queue, put into data matrix
  app.updateMap() # draw new data matrix
  app.autosendCommand() # check for user input and automatically send it

  # Bring GUI to front # app.master = root
  app.master.lift() # bring tk window to front if initialization finishes
  app.master.focus_force() # make tk window active one (so you don't need to click on it for hotkeys to work)
  app.master.mainloop() # start Tkinter GUI loop


# used in Root class
if pythonSeries == 2:
  import Tkinter as tk
  from tkFont import Font
  from tkMessageBox import askokcancel
  from Queue import Queue
elif pythonSeries == 3:
  import tkinter as tk
  from tkinter.font import Font
  from tkinter.messagebox import askokcancel
  from queue import Queue
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2TkAgg
from matplotlib.gridspec import GridSpec
paddedStr = lambda inStr, length: '{0: <{width}s}'.format(inStr, width=length)[0:length]
# GUI constants
CMD_RATE = 100 # minimum time between auto-send commands [ms]
DATA_RATE = 50 # minimum time between updating data from lidar [ms]
MAP_RATE = 500 # minimum time between updating map [ms]
MAX_TX_TRIES = 3 # max number of times to try to resend command before giving up
VIEW_SIZE_M = 8.0 # default size of region to be shown in display [m]
# Protocol constants
NUM_SAMP = 370 # number of serial packets needed for 1 scan (guesstimate)


# used in Slam class
from breezyslam.algorithms import RMHC_SLAM
from breezyslam.components import Laser
# Laser constants
SCAN_SIZE = 360 # number of points per scan
SCAN_RATE_HZ = 1980.0/360 # 1980points/sec * scan/360points [scans/sec]
SCAN_DETECTION_ANGLE = ANG_MAX
SCAN_DISTANCE_NO_DETECTION_MM = DIST_MAX
SCAN_DETECTION_MARGIN = 0
LASER_OFFSET_MM = 35 # this value is negative what it should be # update() returns LIDAR unit position
# BreezySLAM map constants
MAP_SIZE_PIXELS = int(MAP_SIZE_M*MAP_RES_PIX_PER_M) # number of pixels across the entire map
# MAP_SIZE_M = MAP_SIZE_M
MAP_QUALITY = 50
HOLE_WIDTH_MM = 200
RANDOM_SEED = 0xabcd


# used in Data class # note that Data.saveImage() imports PIL and subprocess for map image saving and viewing
import numpy as np # for array processing and matplotlib display
from scipy.ndimage.interpolation import rotate
float2int = lambda x: int(0.5 + x)
# Map constants (derived)
INSET_SIZE_PIX = int(INSET_SIZE_M*MAP_RES_PIX_PER_M) # number of pixels across the inset map
MAP_CENTER_PIX = int(MAP_SIZE_PIXELS/2) # indices of map center []
MM_2_PIX = MAP_RES_PIX_PER_M/1000.0 # [pix/mm]
DEG_2_RAD = np.pi/180.0
MAP_DEPTH = 5 # depth of data points on map (levels of certainty)
ROBOT_DEPTH = MAP_DEPTH + 1 # value of robot in map storage


# used in SerialThread class
if pythonSeries == 2: from Queue import Empty as QueueEmpty
elif pythonSeries == 3: from queue import Empty as QueueEmpty
import sys
import threading # allow serial checking to happen on top of tkinter interface things
from serial import Serial
from serial.serialutil import SerialException
from serial.tools import list_ports # get computer's port info
from time import time, sleep
from struct import unpack # parse incoming serial data
from math import copysign
if pythonSeries == 3: raw_input = lambda inStr: input(inStr)
bits2mask = lambda bitList: sum([2**el for el in bitList])
# Patience constants
SER_READ_TIMEOUT = 1 # time to wait for data from Arduino before connection considered lost [s]
MAX_RX_TRIES = 8 # max number of times to try to tell LIDAR to start before giving up # 1s/try
# Packet constants (shared with Arduino)
ENC_FLAG = '\xFE' # encoder data flag (&thorn)
SCN_FLAG = '\xFF' # scan data flag (&yuml)
BUF_LEN = 20 # points per transmit packet []
PKT_SIZE = 4 # length of scan data packet [bytes]
ENC_SIZE = 8 # length of encoder data packet [bytes]
DFAC = 0.5; # distance resolution factor [1/mm]
AFAC = 8.0; # angle resolution factor [1/deg]
XBEE_BAUD = 125000 # maximum baud rate allowed by Arduino and XBee [hz] # 250k=0x3D090, 125k=0x1E848


class App:
  # init            creates all objects, calls initUI, and starts all loops (including serial thread)
  # initUI          draws all elements in the GUI
  # resetAll        restarts all objects that store map data, allowing history to be wiped without hard reset
  # closeWin        first prompts the user if they really want to close, then ends serial thread and tkinter
  # saveImage       tells data object to capture the current map and save as a png
  # removeMarkers   deletes all temporary markers on the main map (old robot position)
  # drawMarker      draws robot on main map using matplotlib marker
  # getScanData     pulls LIDAR data directly from the serial port and does preliminary processing
  # updateData      calls getScanData, and updates the slam object and data matrix with this data, and loops to itself
  # updateMap       draws a new map, using whatever data is available, and loops to itself
  # sendCommand     triggered by request to manually send the command in the text box
  # autoSendCommand loop to control sending of commands, including automatic retries and continuous drive commands

  def __init__(self, master, data, slam, queues, serThread):
    self.master = master # root tk window
    self.master.protocol("WM_DELETE_WINDOW", self.closeWin) # control what happens when a window is closed externally (e.g. by the 'x')

    # Initialize serial object, prompting user for input if required
    self.statusQueue = queues[0] # status of serial thread # FIFO queue by default
    self.RXQueue = queues[1] # data from serial to root # FIFO queue by default
    self.TXQueue = queues[2] # data from root to serial # FIFO queue by default
    self.serThread = serThread # initialize thread object, getting user input if required

    # Initialize root variables
    self.statusStr = tk.StringVar() # status of serThread
    self.resetting = False # are we in the process of soft restarting?
    self.paused = False # should the loops be doing nothing right now?
    self.markers = [] # current matplotlib markers
    self.points = [] # current scan data
    self.cmds = {'v':{'prop':"speed 0..255",  'func':lambda x: str(x)                 }, # send motor speed as given
                 'w':{'prop':"forward mm",    'func':lambda x: str(int(MM_2_TICK*x))  }, # convert mm to ticks
                 'a':{'prop':"left deg",      'func':lambda x: str(int(DEG_2_TICK*x)) }, # convert degrees to ticks
                 's':{'prop':"reverse mm",    'func':lambda x: str(int(MM_2_TICK*x))  }, # convert mm to ticks
                 'd':{'prop':"right deg",     'func':lambda x: str(int(DEG_2_TICK*x)) }} # convert degrees to ticks
    self.sendingCommand = False # are we trying to manually send a command?

    # Initialize 
    self.data = data # handle map data
    self.slam = slam # do slam processing
    self.initUI() # create all the pretty stuff in the Tkinter window

  def initUI(self, io=True):
    # current (and only) figure
    self.fig = plt.figure(figsize=(9, 5), dpi=131.2) # create matplotlib figure (dpi calculated from $ xrandr)
    gs = GridSpec(1,3) # layout of plots in figure

    # plot color settings
    cmap = plt.get_cmap("binary") # opposite of "gray"
    cmap.set_over("red") # robot map value is set to higher than maximum map value

    # subplot 1 (stationary map)
    self.ax1 = plt.subplot(gs[0,:2]) # add plot 1 to figure
    self.ax1.set_title("Region Map") # name and label plot
    self.ax1.set_xlabel("X Position [mm]")
    self.ax1.set_ylabel("Y Position [mm]")
    self.myImg1 = self.ax1.imshow(self.data.getMapMatrix(), interpolation="none", cmap=cmap, vmin=0, vmax=MAP_DEPTH, # plot data
              extent=[-MAP_SIZE_M/2, MAP_SIZE_M/2, -MAP_SIZE_M/2, MAP_SIZE_M/2]) # extent sets labels by matching limits to edges of matrix
    self.ax1.set_xlim(-VIEW_SIZE_M/2, VIEW_SIZE_M/2) # pre-zoom image to defined default MAP_SIZE_M
    self.ax1.set_ylim(-VIEW_SIZE_M/2, VIEW_SIZE_M/2)

    # colorbar
    self.cbar = self.fig.colorbar(self.myImg1, orientation="vertical") # create colorbar

    # subplot 2 (relative map)
    self.ax2 = plt.subplot(gs[0,2]) # add plot 2 to figure
    self.ax2.set_title("Robot Environs") # name and label plot
    self.ax2.set_xlabel("", family="monospace")
    self.myImg2 = self.ax2.imshow(self.data.getInsetMatrix(), interpolation="none", cmap=cmap, vmin=0, vmax=MAP_DEPTH, # plot data
              extent=[-INSET_SIZE_M/2, INSET_SIZE_M/2, -INSET_SIZE_M/2, INSET_SIZE_M/2])
    self.drawMarker(self.ax2, (0,0,0), temporary=False) # draw permanent robot at center of inset map

    # turn figure data into matplotlib draggable canvas
    self.canvas = FigureCanvasTkAgg(self.fig, master=self.master) # tkinter interrupt function
    self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=1) # put figure at top of window

    # add matplotlib toolbar for easy navigation around map
    NavigationToolbar2TkAgg(self.canvas, self.master).update() # tkinter interrupt function
    self.canvas._tkcanvas.pack(side="top", fill=tk.BOTH, expand=1) # actually goes on bottom...not sure why

    # bind keyboard inputs to functions
    self.master.bind('<Escape>', lambda event: self.closeWin()) # tkinter interrupt function
    self.master.bind('<Shift-R>', lambda event: self.restartAll()) # tkinter interrupt function
    self.master.bind('<Return>', lambda event: self.sendCommand()) # tkinter interrupt function

    # create buttons
    tk.Button(self.master, text="Quit (esc)", command=self.closeWin).pack(side="left", padx=5, pady=5) # tkinter interrupt function
    tk.Button(self.master, text="Restart (R)", command=self.restartAll).pack(side=tk.LEFT, padx = 5) # tkinter interrupt function
    if io:
      tk.Label(self.master, text="Command: ").pack(side="left")
      self.entryBox = tk.Entry(master=self.master, width=15)
      self.entryBox.pack(side="left", padx = 5)
      tk.Button(self.master, text="Send (enter)", command=self.sendCommand).pack(side=tk.LEFT, padx=5) # tkinter interrupt function
    monospaceFont = Font(family="Courier", weight='bold', size=12)
    tk.Label(self.master, textvariable=self.statusStr, font=monospaceFont).pack(side="left", padx=5, pady=5)
    tk.Button(self.master, text="Save Map", command=self.saveImage).pack(side=tk.LEFT, padx=5) # tkinter interrupt function

  def restartAll(self, funcStep=0): # two-part initialization to be re-done at soft reset
    if funcStep == 0:
      self.resetting = True # stop currently running loops
      self.master.after(2000, lambda: self.restartAll(funcStep=1)) # let processor wrap things up elsewhere # should be smarter
      return

    elif funcStep == 1:
      self.resetting = False
      self.data = Data()
      self.slam = Slam()
      self.updateData() # pull data from queue, put into data matrix
      self.updateMap() # draw new data matrix

  def closeWin(self):
    self.paused = True
    self.statusStr.set(paddedStr("Paused",len(self.statusStr.get()))) # keep length of label constant
    if askokcancel("Quit?", "Are you sure you want to quit?"):
      self.statusStr.set(paddedStr("Stopping",len(self.statusStr.get()))) # keep length of label constant
      print("Shutting down LIDAR")
      self.serThread.stop() # tell serial thread to stop running
      print("Closing program")
      self.master.quit() # kills interpreter (necessary for some reason)
    else:
      self.paused = False

  def saveImage(self): # function prototype until data is initialized
    self.statusStr.set(paddedStr("Saving. Close image to resume.", len(self.statusStr.get()))) # keep length of label constant
    self.updateMap(loop=False) # make sure we save the newest map
    self.data.saveImage()

  def removeMarkers(self):
    for i in range(len(self.markers)):
      self.markers[i].remove()
      del self.markers[i]

  def drawMarker(self, ax, pos, temporary=True):
    marker = ax.plot(pos[0]/1000, pos[1]/1000, markersize=8, color='red', marker=(3,1,-pos[2]), markeredgewidth=0, aa=False)
    if temporary: self.markers.extend(marker) # marker is a list of matplotlib.line.Line2D objects

  def getScanData(self, repeat=False):
    self.points = [] # wipe old data before writing new data
    while True:
      # Make sure there's actually data to get
      try:
        queueItem = self.RXQueue.get_nowait()
      except QueueEmpty: # something's wrong
        self.statusStr.set(paddedStr("RXQueue empty. Send 'l' iff LIDAR stopped.", len(self.statusStr.get())))
        return # stop because no data to read
      else:
        if isinstance(queueItem[0], float): # scan data
          self.points.append(queueItem)
        elif isinstance(queueItem[1], int): # encoder data
          self.slam.currEncPos = queueItem
          break # encoder data signals new scan
        else:
          print("RXQueue broken (something weird happened...)")

    if repeat: self.getScanData()

  def updateData(self, init=True):
    self.dataInit = init
    if not self.paused:
      try: # do this before anything else
        status = self.statusQueue.get_nowait()
      except QueueEmpty:
        pass
      else:
        length = len(self.statusStr.get())
        self.statusStr.set(paddedStr(status, length) if length != 0 else status)

      if self.RXQueue.qsize() > (2 if init else 1)*NUM_SAMP: # ready to pull new scan data
        # pull data from serial thread via RXQueue
        # ignore first scan (we're assuming it's not a complete scan, so not reliable)
        self.getScanData(repeat=init) # 2ms

        # update robot position
        if init: self.slam.prevEncPos = self.slam.currEncPos # set both values the first time through
        self.data.getRobotPos(self.slam.updateSlam(self.points), init=init) # send data to slam to do stuff # 15ms

        if init: init = False # initial data gathered successfully

    if not self.resetting: self.master.after(DATA_RATE, lambda: self.updateData(init=init))
    else: self.statusStr.set(paddedStr("Restarting", len(self.statusStr.get()))) # if loop ends, we're restarting

  def updateMap(self, loop=True):
    if not self.paused and not self.dataInit: # wait until first data update to update map
      # create maps
      if INTERNAL_MAP: self.data.drawBreezyMap(self.slam.getBreezyMap()) # draw map using slam data # 16ms
      else: self.data.drawMap(self.points) # draw map using scan points
      self.data.drawInset() # new relative map # 6ms

      # send maps to image object
      self.myImg1.set_data(self.data.getMapMatrix()) # 20ms
      self.myImg2.set_data(self.data.getInsetMatrix()) # 0.4ms

      # finishing touches
      self.removeMarkers() # delete old robot position from map
      self.drawMarker(self.ax1, self.data.get_robot_rel()) # add new robot position to map # 0.2ms
      self.ax2.set_xlabel('X = {0:6.1f}; Y = {1:6.1f};\nHeading = {2:6.1f}'.format(*self.data.get_robot_rel()))

      # refresh the figure
      self.canvas.draw() # 200ms
    if loop and not self.resetting: self.master.after(MAP_RATE, self.updateMap)

  def sendCommand(self):
    self.sendingCommand = True

  def autosendCommand(self, numTries=0, wantACK=False, strIn='', strOut=''):
    # note that ACK-checking only applies to value-setting commands in 'vwasd'

    # expecting ACK from Arduino
    if wantACK:
      gotACK = self.serThread.getACK() # has the serial thread received an ACK?

      # either ACK received or command resent too many times
      if gotACK or numTries >= MAX_TX_TRIES:
        numTries, wantACK = 0, False # reset stuff
        self.entryBox.delete(0,"end") # clear box if we're done sending command
        self.serThread.resetACK() # tell serial thread that we got the ACK

      # no ACK received and not resent too many times
      else:
        numTries += 1 # keep track of how many times we're resending command
        self.TXQueue.put(strOut) # send last command
        self.entryBox.delete(0,"end")
        self.entryBox.insert(0,"try {0}: {1}".format(numTries, strIn)) # tell box what we're doing

    # there's a new command to send (that isn't empty)
    elif self.entryBox.get(): # string in box isn't empty
      strIn = self.entryBox.get() # get new command

      # auto-send continuous drive commands (capitalized normal commands)
      if strIn in 'WASD':
        self.TXQueue.put(strIn[0])

      # manual-send
      elif self.sendingCommand:
        command = strIn[0]
        if command in list(self.cmds): # we're giving the robot a value for a command
          try:
            num = int(strIn[1:])
          except ValueError:
            self.entryBox.delete(1,"end")
            self.entryBox.insert(1,"[{}]".format(self.cmds[command]['prop'])) # prompt user with proper command format
            self.entryBox.selection_range(1,'end')
          else:
            wantACK = True # start resending the command if it isn't received
            strOut = command + self.cmds[command]['func'](num) # re-create command with converted values
            self.TXQueue.put(strOut)
        else: # otherwise send only first character
          self.TXQueue.put(command)
          self.entryBox.delete(0,"end") # clear box

      else:
        pass # wait until manual-send for other commands
    
    self.sendingCommand = False # reset manual sending
    self.master.after(CMD_RATE, lambda: self.autosendCommand(numTries=numTries, wantACK=wantACK, strIn=strIn, strOut=strOut))


class Slam(RMHC_SLAM):
  # init          creates the BreezySLAM objects needed for mapping
  # getBreezyMap  returns BreezySLAM's current internal map
  # updateSlam    takes LIDAR data and uses BreezySLAM to calculate the robot's new position
  # getVelocities uses encoder data to return robot position deltas (forward, angular, and time)

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

    self.breezyMap = bytearray(MAP_SIZE_PIXELS**2) # initialize map array for BreezySLAM's internal mapping

  def getBreezyMap(self):
    return self.breezyMap

  def updateSlam(self, points): # 15ms
    distVec = [0 for i in range(SCAN_SIZE)]

    for point in points: # create breezySLAM-compatible data from raw scan data
      dist = point[0]
      index = int(point[1])
      if not 0 <= index < SCAN_SIZE: continue
      distVec[index] = int(dist)

    # note that breezySLAM switches the x- and y- axes (their x is forward, 0deg; y is right, +90deg)
    if LOG_ALL_DATA: dataFile.write(' '.join((str(el) for el in list(self.currEncPos)+distVec)) + '\n')
    distVec = [distVec[i-180] for i in range(SCAN_SIZE)]
    self.update(distVec, self.getVelocities() if USE_ODOMETRY else None) # update slam information using particle filtering on LIDAR scan data // 10ms
    x, y, theta = self.getpos()

    self.getmap(self.breezyMap) # write internal map to breezyMap

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

    if abs(dxy) > 50 or abs(dtheta) > 20: # encoder data is messed up
      return copysign(2, dxy), copysign(1, dtheta), 1/SCAN_RATE_HZ
    return dxy, dtheta, dt/1000.0 # [mm], [deg], [s]


class Data:
  # init            creates data matrix and information vectors for processing
  # getMapMatrix    returns the map matrix
  # getInsetMatrix  returns the inset map matrix
  # get_robot_rel   returns the robot's position in mm relative to where is started
  # getRobotPos     populates robot position information needed by other methods of Data
  # drawBreezyMap   adds the BreezySLAM internal map to the map matrix
  # drawMap         adds scan data to map matrix
  # drawInset       adds scan data to inset matrix
  # drawRobot       adds robot position to data matrix, in the form of an arrow of red pixels
  # drawPath        draws portion of the robot's trajectory in the form of red dots on the desired object
  # saveImage       uses PIL to write an image file from the data matrix

  def __init__(self):
    print("Each pixel is " + str(round(1000.0/MAP_RES_PIX_PER_M,1)) + "mm, or " + str(round(1000.0/MAP_RES_PIX_PER_M/25.4,2)) + "in.")
    self.mapMatrix = np.zeros((MAP_SIZE_PIXELS, MAP_SIZE_PIXELS), dtype=np.uint8) # initialize data matrix
    self.insetMatrix = np.zeros((INSET_SIZE_PIX, INSET_SIZE_PIX), dtype=np.uint8) # initialize inset matrix
    self.trajectory = [] # robot location history, in pixels
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

  def getMapMatrix(self):
    return self.mapMatrix

  def getInsetMatrix(self):
    return self.insetMatrix

  def get_robot_rel(self):
    return self.robot_rel

  def getRobotPos(self, curr_pos, init=False):
    if init: self.robot_init = curr_pos
    self.robot_rel = tuple([curr-init for (curr,init) in zip(curr_pos,self.robot_init)]) # displacement from start position (center of map)
    xpix = float2int(curr_pos[0]*MM_2_PIX) # robot wrt map center (mO) + mO wrt matrix origin (xO)  = robot wrt xO [pix]
    ypix = MAP_SIZE_PIXELS-float2int(curr_pos[1]*MM_2_PIX) # y is negative because pixels increase downwards (+y_mm = -y_pix = -np rows)
    self.robot_pix = (xpix, ypix, self.robot_rel[2])
    self.trajectory.append(self.robot_pix)

  def drawBreezyMap(self, breezyMap): # 7ms
    if breezyMap: # get data from internal BreezySLAM map
      dataMat = 255-np.flipud(np.resize(np.array(breezyMap, dtype=np.uint8),(MAP_SIZE_PIXELS,MAP_SIZE_PIXELS)).T) # 7ms
      self.mapMatrix = (MAP_DEPTH/255.0*dataMat).astype(np.uint8) # 8ms

  def drawMap(self, points):
    for (dist, ang) in points:
      # pixel location of scan point # point wrt robot + robot wrt x0 = point wrt x0
      x_pix = float2int( MAP_CENTER_PIX + ( self.robot_rel[0] + dist*np.sin((ang+self.robot_rel[2])*DEG_2_RAD) )*MM_2_PIX )
      y_pix = float2int( MAP_CENTER_PIX - ( self.robot_rel[1] + dist*np.cos((ang+self.robot_rel[2])*DEG_2_RAD) )*MM_2_PIX )
      try:
        self.mapMatrix[y_pix, x_pix] += self.mapMatrix[y_pix, x_pix] < MAP_DEPTH # increment value at location if below maximum value
      except IndexError:
        print("scan out of bounds")

  def drawInset(self):
    x, y = self.robot_pix[0:2] # indices of center of robot in main map
    raw = int(INSET_SIZE_PIX*0.75) # half the size of the main map segment to capture
    rad = INSET_SIZE_PIX/2 # half the size of the final segment

    mapChunk = self.mapMatrix[y-raw:y+raw, x-raw:x+raw]
    s = slice(raw-rad, raw+rad, 1) # region of rotated chunk that we want
    self.insetMatrix = rotate(mapChunk, self.robot_rel[2], output=np.uint8, order=1, reshape=False)[s,s]

  def drawRobot(self, mapObject, pos):
    robotMat = rotate(self.robotSprite, -pos[2])
    hgt = (robotMat.shape[0]-1)/2 # indices of center of robot
    wid = (robotMat.shape[1]-1)/2
    x = slice(pos[0]-wid, pos[0]+wid+1, 1) # columns
    y = slice(pos[1]-hgt, pos[1]+hgt+1, 1) # rows
    mapObject[y,x][robotMat.astype(bool)] = ROBOT_DEPTH

  def drawPath(self, mapObject, inSlice):
    for x, y, theta in self.trajectory[inSlice]:
      mapObject[y,x] = ROBOT_DEPTH

  def saveImage(self):
    from PIL import Image # don't have PIL? sorry (try pypng)

    imgData = np.array(self.mapMatrix) # copy map data
    self.drawRobot(imgData, self.robot_pix)
    self.drawPath(imgData, slice(None,-1,None)) # draw all values except last one # same as ":-1"
    robot = imgData > MAP_DEPTH # save location of robot
    imgData = MAP_DEPTH - imgData # invert colors to make 0 black and MAP_DEPTH white
    GB = np.where(robot, 0, 255.0/MAP_DEPTH*imgData).astype(np.uint8) # scale map data and assign to red, green, and blue layers
    R = np.where(robot, 255, GB).astype(np.uint8) # add robot path to red layer

    im = Image.fromarray(np.dstack((R,GB,GB))) # create image from depth stack of three layers
    filename = str(MAP_RES_PIX_PER_M)+"_pixels_per_meter.png" # filename is map resolution
    im.save(filename) # save image
    print("Image saved to " + filename)

    import subprocess # used to display the image (not necessary for save)

    subprocess.call(["eog", filename]) # open with eye of gnome


class SerialThread(threading.Thread):
  # init defines objects and prompts user for port information if necessary
  # connectToPort attempts to establish serial port connection
  # talkToXBee allows direct communication with the XBee if optional flag is set
  # waitForResponse attempts to establish contact with the Arduino
  # stop ends serial communication with the Arduino
  # writeCmd sends data to the Arduino
  # getACK returns whether we have received an ACK from the Arduino
  # resetACK resets boolean indicating that Arduino has received a command
  # run is the main loop, which handles all serial communication

  def __init__(self, statusQueue, RXQueue, TXQueue):
    super(SerialThread, self).__init__() # nicer way to initialize base class (only works with new-style classes)
    self.statusQueue = statusQueue
    self.RXQueue = RXQueue
    self.TXQueue = TXQueue
    self._stop = threading.Event() # create flag
    self.gotACK = False

    self.connectToPort() # initialize serial connection with XBee
    if TALK_TO_XBEE: self.talkToXBee() # optional (see function)
    self.waitForResponse() # start LIDAR and make sure Arduino is sending stuff back to us

    self.ser.timeout = SER_READ_TIMEOUT

  def connectToPort(self):
    # first select a port to use
    portList = sorted([port[0] for port in list_ports.comports() if 'USB' in port[0] or 'COM' in port[0]])
    if not portList: # list empty
      sys.exit("Check your COM ports for connected devices and compatible drivers.")
    elif len(portList) == 1: # if there's only one device connected, use it
      portName = portList[0]
    else: # query user for which port to use
      print("Available COM ports:")
      print(portList)
      portRequested = raw_input("Please select a port number [%s]: " % ', '.join([str(el[-1]) for el in portList]))
      portName = (portList[0][0:-1] + portRequested) if portRequested is not "" else portList[0]

    # then try to connect to it
    try:
      self.ser = Serial(portName, baudrate=XBEE_BAUD)
    except SerialException:
      sys.exit("Invalid port.")
    else:
      sleep(1) # give time to connect to serial port
      print("Successfully connected to: %s" % portName)

  def talkToXBee(self): # allow direct interaction with XBee (Arduino code needs modification)
    if raw_input("Configure this (Arduino's) XBee before mapping? [y/N]: ").lower() == 'y':
      while True:
        inputStr = raw_input("Enter XBee command: ")
        sendStr = inputStr if (inputStr == "+++" or inputStr == '') else inputStr + '\x0D'
        self.ser.write(sendStr)
        tstart = time()
        while time() < tstart + 1: # give XBee 1 sec to respond
          if self.ser.inWaiting(): print(self.ser.read())
        if inputStr.lower() == "atcn": # we've told the XBee to exit command mode, so we should, too...
          self.ser.write('q') # tells Arduino to stop talking to XBee (shouldn't affect computer XBee...)
          break
      sys.exit("Please restart XBee and then re-run this code.") # XBee has to power cycle for change to take effect

  def waitForResponse(self):
    tryCount = 0
    while True:
      self.ser.write('l') # tell robot to start lidar
      self.ser.flushInput() # wipe the receiving buffer
      print("Command sent to robot... waiting for response... check robot power...")
      sleep(1) # give time for Arduino to send something

      if self.ser.inWaiting(): break # until we see something...

      tryCount += 1
      if tryCount >= MAX_RX_TRIES: sys.exit("No response received from Arduino.") # only try for MAX_RX_TRIES seconds before giving up

    print("Data received... live data processing commencing...")

  def stop(self, try1=True):
    if try1:
      self._stop.set() # set stop flag to True
      sleep(0.2) # give serial reading loop time to finish current point before flushInput()
      # prevents: SerialException: device reports readiness to read but returned no data (device disconnected?)
    self.ser.write('o') # tell robot to turn lidar off
    self.ser.flushInput() # empty input serial buffer
    sleep(0.5) # give time to see if data is still coming in
    if self.ser.inWaiting(): self.stop(try1=False)

  def writeCmd(self, outBytes):
    self.ser.write(outBytes)

  def getACK(self):
    return self.gotACK

  def resetACK(self):
    self.gotACK = False

  def run(self):
    MASK1 = bits2mask(range(0,4)) # 0b00001111
    MASK2 = bits2mask(range(4,8)) # 0b11110000

    tstart = time()
    lagged, missed, total, scans = 0,0,0,0
    while not self._stop.isSet(): # pulls and processes all incoming and outgoing serial data
      # relay commands from root to Arduino
      try:
        self.writeCmd(self.TXQueue.get_nowait())
      except QueueEmpty:
        pass

      # check if data in buffer is too old to be useful (300 packets)
      if self.ser.inWaiting() > 300*PKT_SIZE:
        lagged += self.ser.inWaiting()/PKT_SIZE
        self.ser.flushInput()

      # pull serial data from computer buffer (XBee)
      # in order sent (bytes comma-separated):  dist[0:7], dist[8:12] ang[0:3], ang[4:12]
      pointLine = self.ser.read(PKT_SIZE) # distance and angle (blocking)
      if len(pointLine) < PKT_SIZE: # timeout occurs
        self.statusQueue.put("ser.read() timeout. Send 'l' iff LIDAR stopped.")
        continue # try again

      if time() > tstart + 1.0: # report status of serial thread to root every second
        tstart += 1.0
        self.statusQueue.put("{:4} lagged, {:2} errors in {:4} points, {:2} scans.".format(lagged,missed,total,scans))
        lagged, missed, total, scans = 0,0,0,0


      # check for command ACK
      if pointLine == ENC_FLAG*PKT_SIZE:
        self.gotACK = True # ACK from Arduino
        continue # move to the next point

      # check for encoder data packet
      if pointLine[0:2] == ENC_FLAG*2:
        pointLine += self.ser.read(ENC_SIZE-PKT_SIZE) # read more bytes to complete longer packet
        self.RXQueue.put(unpack('<2hH',pointLine[2:])) # little-endian 2 signed shorts, 1 unsigned short
        scans += 1
        continue # move to the next point

      # check for lidar data packet
      if pointLine[-1] == SCN_FLAG:
        byte1, byte2, byte3 = unpack('<3B',pointLine[0:-1]) # little-endian 3 bytes
        distCurr = (byte1 | (byte2 & MASK1) << 8)/DFAC # 12 least-significant (sent first) bytes12 bits
        angleCurr = (byte3 << 4 | (byte2 & MASK2) >> 4)/AFAC # 4 most-significant (sent last) bytes2 bits, 8 byte1 bits
        if DIST_MIN < distCurr < DIST_MAX and angleCurr <= ANG_MAX: # data matches what was transmitted
          self.RXQueue.put((distCurr, angleCurr))
          total += 1
        else: # invalid point received (communication error)
          while self.ser.read(1) != SCN_FLAG: pass # delete current packet up to and including SCN_FLAG byte
          missed += 1
        continue # move to the next point


if __name__ == '__main__':
  if LOG_ALL_DATA: dataFile = open(LOGFILE_NAME+'.log','w')
  main()
  if LOG_ALL_DATA: dataFile.close()
