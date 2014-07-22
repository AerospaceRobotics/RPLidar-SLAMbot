#!/usr/bin/env python

# breezySLAM_XBee.py - base station code for Aerospace Robotics SLAMbot project
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

import sys
print("Python {}".format('.'.join([str(el) for el in sys.version_info[0:3]])))

GPL = "breezySLAM_XBEE.py Copyright (C) 2014 Michael Searing \n\
This program comes with ABSOLUTELY NO WARRANTY; for details type [tbd]. \n\
This is free software, and you are welcome to redistribute it \n\
under certain conditions; type [tbd] for details."
print(GPL)

from breezyslam.algorithms import RMHC_SLAM
from breezyslam.components import Laser
from breezyslam.robots import WheeledRobot

if sys.version_info[0] < 3: # 2.x
  print("Getting Python 2 modules")
  import Tkinter as tk
  from tkMessageBox import askokcancel
  import Queue as queue
else: # 3.x
  print("Getting Python 3 modules")
  import tkinter as tk
  from tkinter.messagebox import askokcancel
  import queue
  def raw_input(inStr): return input(inStr)

import time
import threading # allow serial checking to happen on top of tkinter interface things

import serial
from serial.tools import list_ports # get computer's port info
import struct # parse incoming serial data
import numpy as np
from scipy.ndimage.interpolation import rotate

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2TkAgg
import matplotlib.pyplot as plt

# note that Data.saveImage() imports PIL and subprocess for map image saving and viewing

TALK_TO_XBEES = True if len(sys.argv) > 1 else False # non-blocking user input (yay)

USE_ODOMETRY = True
INTERNAL_MAP = True

# Macros (here's me sort of wishing this were C++...)
def float2int(x):
  return int(0.5 + x)

# Packet constants (shared with Arduino)
ENC_FLAG = '\xFE' # encoder data flag (&thorn)
SCN_FLAG = '\xFF' # scan data flag (&yuml)
BUF_LEN = 20 # points per transmit packet []
PKT_SIZE = 4 # length of scan data packet [bytes]
ENC_SIZE = 8 # length of encoder data packet [bytes]
DFAC = 0.5; # distance resolution factor [1/mm]
AFAC = 8.0; # angle resolution factor [1/deg]
XBEE_BAUD = 250000 # maximum baud rate allowed by Arduino and XBee [hz]
DIST_MIN = 100; # minimum distance
DIST_MAX = 6000; # maximum distance
ANG_MIN = 0; # minimum scan angle
ANG_MAX = 360; # maximum scan angle

# Protocol constants
MASK = 0b0000111111111111 # bits 0 .. 11
NUM_SAMP = 360 # number of serial packets needed for 1 scan (guesstimate)

# Map constants
VIEW_SIZE_M = 4 # default size of region to be shown in display [m]
MAP_SIZE_M = 16 # size of region to be mapped [m]
MAP_RES_PIX_PER_M = 100 # number of pixels of data per meter [pix/m]
MAP_DEPTH = 10 # depth of data points on map (levels of certainty)

# GUI constants
CMD_RATE = 100 # minimum time between auto-send commands [ms]
DATA_RATE = 50 # minimum time between updating data from lidar [ms]
MAP_RATE = 500 # minimum time between updating map [ms]
MAX_RX_TRIES = 8 # max number of times to try to tell LIDAR to start before giving up
MAX_TX_TRIES = 3 # max number of times to try to resend command before giving up

# Robot constants
REV_2_TICK = 1000.0/3 # encoder ticks per wheel revolution
WHEEL_DIAMETER = 58.2
WHEEL_BASE = 177.0 # length of treads [mm]
WHEEL_TRACK = 190.0 # separation of treads [mm]

# Laser constants
SCAN_SIZE = 361 # number of points per scan
SCAN_RATE_HZ = 1980.0/360 # 1980points/sec * scan/360points [scans/sec]
SCAN_ANGLE_MIN_DEG = ANG_MIN
SCAN_ANGLE_MAX_DEG = ANG_MAX
SCAN_DISTANCE_NO_DETECTION_MM = DIST_MAX
SCAN_DETECTION_MARGIN = 0
LASER_OFFSET_MM = -35

# BreezySLAM map constants
MAP_SIZE_PIXELS = MAP_SIZE_M*MAP_RES_PIX_PER_M # number of pixels across the map
MAP_SCALE_MM_PER_PIXEL = int(1000/MAP_RES_PIX_PER_M)
MAP_QUALITY = 50
HOLE_WIDTH_MM = 100
RANDOM_SEED = 0xabcd

# Map constants (derived)
VIEW_SIZE_MM = 1000*VIEW_SIZE_M
MAP_SIZE_MM = MAP_SIZE_M*1000 # size of data matrix [mm]
MAP_CENTER_PIX = MAP_SIZE_PIXELS/2 # indices of map center []
MM_2_PIX = MAP_RES_PIX_PER_M/1000.0 # [pix/mm]
DEG_2_RAD = np.pi/180.0
ROBOT_DEPTH = MAP_DEPTH + 1 # value of robot in map storage

# Robot constants (derived)
REV_2_MM = np.pi*WHEEL_DIAMETER # circumference [mm]
MM_2_TICK = REV_2_TICK/REV_2_MM # [ticks/mm]
TICK_2_MM = REV_2_MM/REV_2_TICK # [mm/tick]
TREAD_ERROR = 0.89 # continuous tread slips this much relative to slip of wheels (experimental)
ANGULAR_FLUX = TREAD_ERROR*((WHEEL_BASE/WHEEL_TRACK)**2+1); # tread slippage [] # our math assumed wheels, hence TREAD_ERROR
ROT_2_MM = np.pi*WHEEL_TRACK # circumference of wheel turning circle [mm]
ROT_2_DEG = 360.0 # [deg]
REV_2_DEG = ROT_2_DEG*(REV_2_MM/ROT_2_MM) # degrees of rotation per wheel revolution [deg]
DEG_2_TICK = REV_2_TICK/REV_2_DEG * ANGULAR_FLUX # [ticks/deg]
TICK_2_DEG = REV_2_DEG/REV_2_TICK * 1.0/ANGULAR_FLUX # [deg/tick]

print("Each pixel is " + str(round(1000.0/MAP_RES_PIX_PER_M,1)) + "mm, or " + str(round(1000.0/MAP_RES_PIX_PER_M/25.4,2)) + "in.")

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

    self.RXQueue = queue.Queue() # data from serial to root # FIFO queue by default
    self.TXQueue = queue.Queue() # data from root to serial # FIFO queue by default
    self.serThread = SerialThread(self.RXQueue, self.TXQueue) # initialize thread object, getting user input if required
    self.initUI() # create all the pretty stuff in the Tkinter window
    self.resetting = False
    self.restartAll(rootInit=True) # contains all initialization that needs to be re-done in case of a soft reset
    self.lift() # bring tk window to front if initialization finishes

  def initUI(self):
    self.fig = plt.figure(figsize=(8, 5), dpi=131) # create matplotlib figure
    self.ax = self.fig.add_subplot(111) # add plot to figure
    self.ax.set_title("RPLIDAR Plotting") # name and label plot
    self.ax.set_xlabel("X Position [mm]")
    self.ax.set_ylabel("Y Position [mm]")

    # if INTERNAL_MAP: cmap = plt.get_cmap("gray")
    # else: cmap = plt.get_cmap("binary")
    cmap = plt.get_cmap("binary")
    cmap.set_over("red") # robot is set to higher than MAP_DEPTH
    dummyInitMat = np.zeros((2,2), dtype=np.uint8) # need to plot something to setup tkinter before dependent objects
    self.myImg = self.ax.imshow(dummyInitMat, interpolation="none", cmap=cmap, vmin=0, vmax=MAP_DEPTH, # plot data
              extent=[-MAP_SIZE_MM/2, MAP_SIZE_MM/2, -MAP_SIZE_MM/2, MAP_SIZE_MM/2]) # extent sets labels by matching limits to edges of matrix
    self.ax.set_xlim(-VIEW_SIZE_MM/2, VIEW_SIZE_MM/2) # pre-zoom image to defined default VIEW_SIZE_M
    self.ax.set_ylim(-VIEW_SIZE_MM/2, VIEW_SIZE_MM/2)
    self.cbar = self.fig.colorbar(self.myImg, orientation="vertical") # create colorbar

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
    self.bind('<Return>', lambda event: self.sendCommand(loop=False)) # tkinter interrupt function

    # create buttons
    tk.Button(self, text="Quit (esc)", command=self.closeWin).pack(side="left", padx=5, pady=5) # tkinter interrupt function
    tk.Button(self, text="Restart (r)", command=self.restartAll).pack(side=tk.LEFT, padx = 5) # tkinter interrupt function
    tk.Label(self, text="Command: ").pack(side="left")
    self.entryBox = tk.Entry(master=self)
    self.entryBox.pack(side="left", padx = 5)
    tk.Button(self, text="Send (enter)", command=lambda: self.sendCommand(loop=False)).pack(side=tk.LEFT, padx=5) # tkinter interrupt function
    self.numLost = tk.StringVar() # status of serThread
    tk.Label(self, textvariable=self.numLost).pack(side="left", padx=5, pady=5)
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
      self.data = Data()
      self.slam = Slam()
      self.updateData(init=True) # pull data from queue, put into data matrix
      self.updateMap() # draw new data matrix
      self.sendCommand(loop=True) # check for user input and automatically send it

  def closeWin(self):
    if askokcancel("Quit?", "Are you sure you want to quit?"):
      print("Shutting down LIDAR")
      self.serThread.stop() # tell serial thread to stop running
      print("Closing program")
      self.quit() # kills interpreter (necessary for some reason)

  def saveImage(self): # function prototype until data is initialized
    self.data.saveImage()

  def sendCommand(self, loop=True, resendCount=0): # loop indicates how function is called: auto (True) or manual (False)
    # first, figure out which string to send # note that ACK-checking only applies to value-setting commands in 'vwasd'

    # expecting ACK, but not received yet, and not resent MAX_TX_TRIES times
    if self.serThread.cmdSent and not self.serThread.cmdRcvd and resendCount <= MAX_TX_TRIES-1:
      resend = True
      strIn = self.serThread.sentCmd # get previous command

    else: # not resending last command
      # ACK received, or command resent too many times (failed)
      if (self.serThread.cmdSent and self.serThread.cmdRcvd) or resendCount >= MAX_TX_TRIES:
        self.serThread.cmdSent = False # reset state values
        self.serThread.cmdRcvd = False
        resendCount = 0
      resend = False
      strIn = self.entryBox.get() # get new command

    # then, send it in the proper manner corresponding to the command
    if strIn: # string is not empty
      if not loop: # manual-send mode of function
        self.TXQueue.put(strIn)
        self.entryBox.delete(0,"end") # clear box after manual-send

      elif strIn in 'WASD': # auto-send continuous drive commands (capitalized normal commands)
        self.TXQueue.put(strIn)

      elif resend: # auto-resend command until ACK received
        resendCount += 1 # keep track of how many times we're resending command
        self.TXQueue.put(strIn)

      else:
        pass # wait until manual-send for other commands

    if loop and not self.resetting: self.after(CMD_RATE, lambda: self.sendCommand(resendCount=resendCount)) # tkinter interrupt function

  def updateData(self, init=False):
    if self.RXQueue.qsize() > NUM_SAMP or init: # ready to pull new scan data
      self.getScanData() # pull LIDAR data via serial from robot, write to data object
      if init: self.slam.prevEncPos = self.slam.currEncPos # set both values the first time through

      self.data.robot_mm = self.slam.updateSlam(self.data.points) # send data to slam to do stuff
      if init: self.data.robot_mm0 = self.data.robot_mm # set initial position during initialization

      if INTERNAL_MAP:
        dataMat = 255-np.flipud(np.resize(np.array(self.slam.breezyMap, dtype=np.uint8),(MAP_SIZE_PIXELS,MAP_SIZE_PIXELS)).T) # 7ms
        self.data.matrix = (MAP_DEPTH/255.0*dataMat).astype(np.uint8) # 8ms
      else:
        self.data.drawPoints() # draw points, using updated slam, on the data matrix # 16ms
      self.data.drawRobot() # new robot position

    if not self.resetting: self.after(DATA_RATE, self.updateData) # tkinter interrupt function

  def updateMap(self):
    self.myImg.set_data(self.data.matrix) # 15ms
    self.canvas.draw() # 400ms

    if not self.resetting: self.after(MAP_RATE, lambda: self.updateMap()) # tkinter interrupt function

  def getScanData(self): # 50ms
    while True:
      queueItem = self.RXQueue.get()
      if isinstance(queueItem[0], float): # scan data
        self.data.points.append(queueItem)
      elif isinstance(queueItem[1], int): # encoder data
        self.slam.currEncPos = queueItem
        break # encoder data signals new scan
      elif isinstance(queueItem, str): # serial thread status
        self.numLost.set(queueItem)
      else:
        print("RXQueue broken")


class Slam(RMHC_SLAM):
  # updateSlam takes LIDAR data and uses BreezySLAM to calculate the robot's new position
  # getVelocities takes the encoder data (lWheel, rWheel, timeStamp) and finds change in translational and angular position and in time

  def __init__(self):
    laser = Laser(SCAN_SIZE, \
                  SCAN_RATE_HZ, \
                  SCAN_ANGLE_MIN_DEG, \
                  SCAN_ANGLE_MAX_DEG, \
                  SCAN_DISTANCE_NO_DETECTION_MM, \
                  SCAN_DETECTION_MARGIN, \
                  LASER_OFFSET_MM)
    RMHC_SLAM.__init__(self, \
                       laser, \
                       MAP_SIZE_PIXELS, \
                       MAP_SCALE_MM_PER_PIXEL, \
                       MAP_QUALITY, \
                       HOLE_WIDTH_MM, \
                       RANDOM_SEED)
    self.prevEncPos = () # robot encoder data
    self.currEncPos = () # left wheel [ticks], right wheel [ticks], timestamp [ms]

    if INTERNAL_MAP: self.breezyMap = bytearray(MAP_SIZE_PIXELS**2) # initialize map array for BreezySLAM's internal mapping

  def updateSlam(self, points): # 15ms
    distVec = [0 for i in range(SCAN_SIZE)]

    for point in points: # create breezySLAM-compatible data from raw scan data
      dist = point[0]
      index = float2int(point[1]) # index
      if not 0 <= index <= 360: continue
      distVec[index] = int(dist) if distVec[index] == 0 else int((dist+distVec[index])/2)

    # note that breezySLAM switches the x- and y- axes (their x is forward, 0deg; y is right, +90deg)
    # dataFile.write(' '.join((str(el) for el in list(self.currEncPos)+distVec)) + '\n')
    self.update(distVec, self.getVelocities() if USE_ODOMETRY else None) # update slam information using particle filtering on LIDAR scan data // 10ms
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
  # drawRobot adds robot position to data matrix, generally using slam to find this position
  # saveImage uses PIL to write an image file from the data matrix

  def __init__(self):    
    self.matrix = np.zeros((MAP_SIZE_PIXELS, MAP_SIZE_PIXELS), dtype=np.uint8) # initialize data matrix
    self.points = [] # current scan data
    self.robot_mm0 = () # robot location data
    self.robot_mm = () # x [mm], y [mm], th [deg], defined from lower-left corner of map

    self.robotSprite = np.array([[0,0,0,0,0,1,0,0,0,0,0], # shape of robot on map
                                 [0,0,0,0,0,1,0,0,0,0,0],
                                 [0,0,0,0,0,1,0,0,0,0,0],
                                 [0,0,0,0,1,1,1,0,0,0,0],
                                 [0,0,0,0,1,1,1,0,0,0,0],
                                 [0,0,0,1,1,1,1,1,0,0,0],
                                 [0,0,0,1,1,0,1,1,0,0,0],
                                 [0,0,1,1,0,0,0,1,1,0,0],
                                 [0,0,1,1,0,0,0,1,1,0,0],
                                 [0,1,1,0,0,0,0,0,1,1,0],
                                 [0,1,1,1,1,1,1,1,1,1,0],
                                 [1,1,1,1,1,1,1,1,1,1,1]])

  def drawPoints(self): # 7ms
    xRobot_mm = (self.robot_mm[0] - self.robot_mm0[0]) # displacement from start position (center of map)
    yRobot_mm = (self.robot_mm[1] - self.robot_mm0[1])
    th_deg = self.robot_mm[2] - self.robot_mm0[2]

    for point in self.points:
      dist = point[0]
      ang = point[1]
      x_pix = float2int( MAP_CENTER_PIX + ( xRobot_mm + dist*np.sin((ang+th_deg)*DEG_2_RAD) )*MM_2_PIX ) # pixel location of scan point
      y_pix = float2int( MAP_CENTER_PIX - ( yRobot_mm + dist*np.cos((ang+th_deg)*DEG_2_RAD) )*MM_2_PIX ) # point wrt robot + robot wrt xO
      try:
        self.matrix[y_pix, x_pix] += self.matrix[y_pix, x_pix] < MAP_DEPTH # increment value at location if below maximum value
      except IndexError:
        print("scan out of bounds")

  def drawRobot(self):
    # robot wrt map center (mO) + mO wrt matrix origin (xO)  = robot wrt xO [pix]
    # y is negative because np array rows increase downwards (+y_mm = -y_pix = -np rows)
    xRobot_pix = float2int(MAP_CENTER_PIX + (self.robot_mm[0] - self.robot_mm0[0])*MM_2_PIX)
    yRobot_pix = float2int(MAP_CENTER_PIX - (self.robot_mm[1] - self.robot_mm0[1])*MM_2_PIX)
    th_deg = self.robot_mm[2] - self.robot_mm0[2]

    robotMat = rotate(self.robotSprite, -th_deg, output=int)
    rShape = robotMat.shape # rows, columns

    hgt = (rShape[0]-1)/2 # indices of center of robot
    wid = (rShape[1]-1)/2
    for x in range(-wid, wid+1): # relative to center of robot
      for y in range(-hgt, hgt+1):
        if robotMat[y+hgt, x+wid] == 1: # robot should exist here
          try:
            self.matrix[yRobot_pix+y, xRobot_pix+x] = ROBOT_DEPTH
          except IndexError:
            print("robot out of bounds")

  def saveImage(self):
    from PIL import Image # don't have PIL? sorry (try pypng)

    imgData = MAP_DEPTH - self.matrix # invert colors to make 0 black and MAP_DEPTH white
    robot = imgData < 0 # save location of robot
    imgData[:] = np.where(robot, 0, imgData) # make robot path 0
    GB = (255.0/MAP_DEPTH*imgData).astype(np.uint8) # scale map data and assign to red, green, and blue layers
    R = GB + (255*robot).astype(np.uint8) # add robot path to red layer

    im = Image.fromarray(np.dstack((R,GB,GB))) # create image from depth stack of three layers
    filename = str(MAP_RES_PIX_PER_M)+"_pixels_per_meter.png" # filename is map resolution
    im.save(filename) # save image
    print("Image saved to " + filename)

    import subprocess
    subprocess.call(["eog", filename]) # open with eye of gnome


class SerialThread(threading.Thread):
  def __init__(self, RXQueue, TXQueue):
    super(SerialThread, self).__init__() # nicer way to initialize base class (only works with new-style classes)
    self.RXQueue = RXQueue
    self.TXQueue = TXQueue
    self._stop = threading.Event() # create flag
    self.cmdSent = False
    self.cmdRcvd = False

    self.connectToPort() # initialize serial connection with XBee
    if TALK_TO_XBEES: self.talkToXBee() # optional (see function)
    self.waitForResponse() # start LIDAR and make sure Arduino is sending stuff back to us

    self.start() # put serial data into queue

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
      portName = portList[0][0:-1] + raw_input("Please select a port number [%s]: " % ', '.join([str(el[-1]) for el in portList]))

    # then try to connect to it
    try:
      self.ser = serial.Serial(portName, baudrate=XBEE_BAUD)
    except serial.serialutil.SerialException:
      sys.exit("Invalid port.")
    else:
      time.sleep(1) # give time to connect to serial port
      print("Successfully connected to: %s" % portName)

  def talkToXBee(self): # allow direct interaction with XBee (Arduino code needs modification)
    if raw_input("Configure this (Arduino's) XBee before mapping? [y/N]: ") == 'y':
      while True:
        inputStr = raw_input("Enter XBee command: ")
        sendStr = inputStr if (inputStr == "+++" or inputStr == '') else inputStr + '\x0D'
        self.ser.write(sendStr)
        tstart = time.clock()
        while time.clock() < tstart + 1: # give XBee 1 sec to respond
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
      time.sleep(1) # give time for Arduino to send something

      if self.ser.inWaiting(): break # until we see something...

      tryCount += 1
      if tryCount >= MAX_RX_TRIES: sys.exit("No response received from Arduino.") # only try for MAX_RX_TRIES seconds before giving up

    print("Data received... live data processing commencing...")

  def stop(self, try1 = True):
    if try1:
      self._stop.set() # set stop flag to True
      time.sleep(0.2) # give serial reading loop time to finish current point before flushInput()
      # prevents: SerialException: device reports readiness to read but returned no data (device disconnected?)
    self.ser.write('o') # tell robot to turn lidar off
    self.ser.flushInput() # empty input serial buffer
    time.sleep(0.5) # give time to see if data is still coming in
    if self.ser.inWaiting(): self.stop(try1=False)

  def writeCmd(self, outByte):
    command = outByte[0]
    if command in 'vwasd': # we're giving the robot a value for a command
      self.ser.write(command) # send the command
      try:
        num = float(outByte[1:])

      except IndexError:
        if command == 'v': print("Invalid command. Enter speed for speed set command.")
        else: print("Invalid command. Enter distance/angle for movement commands.")

      except ValueError:
        print("Invalid command. Enter a number after command.")

      else:
        self.cmdSent = True
        self.sentCmd = outByte
        if command == 'v': self.ser.write(str(num)) # send motor speed as given
        elif command in 'ws': self.ser.write(str(int(MM_2_TICK*num))) # convert millimeters to ticks
        elif command in 'ad': self.ser.write(str(int(DEG_2_TICK*num))) # convert degrees to ticks

    else:
      self.ser.write(command) # otherwise send only first character

  def run(self):
    tstart = time.time()
    lagged, missed, total, scans = 0,0,0,0
    while not self._stop.isSet(): # pulls and processes all incoming and outgoing serial data
      if time.time() > tstart + 1.0: # report status of serial thread to root every second
        tstart += 1.0
        self.RXQueue.put("Last sec: {} lagged, {} errors out of {} points, {} scans.".format(lagged,missed,total,scans))
        lagged, missed, total, scans = 0,0,0,0

      # relay commands from root to Arduino
      try:
        self.writeCmd(self.TXQueue.get_nowait())
      except queue.Empty:
        pass

      if self.ser.inWaiting() > 300*PKT_SIZE: # data in buffer is too old to be useful (300 packets old)
        lagged += self.ser.inWaiting()/PKT_SIZE
        self.ser.flushInput()

      # in order sent (bytes comma-separated):                  dist[0:7], dist[8:12] ang[0:3], ang[4:12]
      # in order received (future data chunks comma-separated): dist[0:12]            ang[0:3], ang[4:12]
      pointLine = self.ser.read(PKT_SIZE) # distance and angle (blocking)

      # check for command ACK
      if pointLine == ENC_FLAG*PKT_SIZE:
        self.cmdRcvd = True # ACK from Arduino
        continue # move to the next point

      # check for encoder data packet
      if pointLine[0:2] == ENC_FLAG*2:
        pointLine += self.ser.read(ENC_SIZE-PKT_SIZE) # read more bytes to complete longer packet
        self.RXQueue.put(struct.unpack('<hhH',pointLine[2:ENC_SIZE+1])) # little-endian 2 signed shorts, 1 unsigned short
        scans += 1
        continue # move to the next point

      # check for lidar data packet
      if pointLine[PKT_SIZE-1] == SCN_FLAG:
        bytes12, byte3 = struct.unpack('<HB',pointLine[0:PKT_SIZE-1]) # little-endian 2 bytes and 1 byte
        distCurr = (bytes12 & MASK)/DFAC # 12 least-significant (sent first) bytes12 bits
        angleCurr = ((bytes12 & ~MASK) >> 12 | byte3 << 4)/AFAC # 4 most-significant (sent last) bytes2 bits, 8 byte1 bits
        if DIST_MIN < distCurr < DIST_MAX and angleCurr <= ANG_MAX: # data matches what was transmitted
          self.RXQueue.put((distCurr, angleCurr))
          total += 1
        else: # invalid point received (communication error)
          while self.ser.read(1) != SCN_FLAG: pass # delete current packet up to and including SCN_FLAG byte
          missed += 1
        continue # move to the next point


if __name__ == '__main__':
  # dataFile = open('data.log','w')
  root = Root() # create Tkinter window, containing entire App
  root.mainloop() # start Tkinter loop
  # dataFile.close()