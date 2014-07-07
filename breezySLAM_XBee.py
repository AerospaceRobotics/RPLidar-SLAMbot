#!/usr/bin/env python

# This file is the python base station code 
# for use with the Aerospace Robotics SLAM Rover project. 
# Do whatever you want with this code as long as it doesn't kill people.
# No liability et cetera.
# http://www.aerospacerobotics.com/             June-August 2014
#                     Michael Searing & Bill Warner


from breezyslam.algorithms import CoreSLAM
from breezyslam.robots import WheeledRobot, Laser

import sys
if sys.version_info[0] < 3:
    import Tkinter as tk
else:
    import tkinter as tk

import time # wait for robot to do things that take time
import threading # allow serial checking to happen on top of tkinter interface things
import Queue # deal with sending data across threads

import serial # bind hardware serial
from serial.tools import list_ports # bind hardware serial
import struct # parse incoming serial data
import numpy as np # create map array
from scipy.ndimage.interpolation import rotate

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2TkAgg
import matplotlib.pyplot as plt

# note that Data.saveImage() imports PIL and subprocess for map image saving and viewing


PKT_LEN = 4
NEWLINE = '\xFE' # response indicator (&thorn)
ENDLINE = '\xFF' # end line terminator (&yuml)
DFAC = 0.5; # distance resolution factor [1/mm]
AFAC = 8.0; # angle resolution factor [1/deg]
MASK = 0b0000111111111111
XBEE_BAUD = 250000

# "sizes" all refer to half of the image (radius)
viewSize = 2 # default size of region to be shown in display [m]
roomSize = 12 # size of region to be mapped [m]
rad = roomSize*1000 # size of data matrix [mm]
mapRes = 100 # number of pixels of data per meter [pix/m]
mapSize = roomSize*mapRes # number of pixels on each side of the origin
mm2pix = mapRes/1000.0 # [pix/mm]
deg2rad = np.pi/180.0
maxVal = 10 # depth of data points on map (higher=more sure that it exists)
robotVal = maxVal + 1 # value of robot in map storage
numSamp = 400 # desired number of valid points per scan
# updateHz = 1980points/sec * scan/400points

commandRate = 100 # minimum time between auto-send commands [ms]
dataRate = 50 # minimum time between updating data from lidar [ms]
mapRate = 500 # minimum time between updating map [ms]

# distance and angle conversions into encoder ticks (wheel revolution != robot rotation)
ticksPerRev = 1000.0/3
wheelDia = 58.2 # diameter [mm]
mmPerRev = np.pi*wheelDia # circumference [mm]
mm2ticks = ticksPerRev/mmPerRev # [ticks/mm]

ANGULAR_FLUX = 1.62;
wheelTrack = 185 # separation of wheels [mm]
mmPerRot = np.pi*wheelTrack # circumference of wheel turning circle [mm]
degPerRot = 360 # [deg]
degPerRev = degPerRot*(mmPerRev/mmPerRot) # degrees of rotation per wheel revolution [deg]
deg2ticks = ticksPerRev/degPerRev * ANGULAR_FLUX # [ticks/deg]


print "Each pixel is",round(1000.0/mapRes,1),"mm, or",round(1000.0/mapRes/25.4,2),"in"

def float2int(x):
  return int(0.5 + x)

class Root(tk.Tk): # Tkinter window, inheriting from Tkinter module
  # init draws and displays window, creates data matrix, and calls updateData and updateMap
  # updateData calls getScanData, and updates the slam object and data matrix with this data, and loops to itself
  # updateMap draws a new map, using whatever data is available, and loops to itself
  # getScanData pulls LIDAR data directly from the serial port and does preliminary processing
  # resetAll recreates slam object to remove previous data it and wipes current data matrix

  def __init__(self):
    tk.Tk.__init__(self) # explicitly initialize base class and create window
    self.geometry('+100+100')

    self.serQueue = Queue.Queue() # FIFO queue by default
    self.numLost = tk.StringVar() # status of serThread (should be a queue...)
    self.serThread = SerialThread(self.serQueue, self.numLost) # initialize thread object

    self.data = Data() # initialize data object, creating matrix and supporting vectors
    self.slam = Slam() # initialize slam object
    self.initUI() # create all the pretty stuff in the Tkinter window

    self.updateData(init = True) # pull data from queue, put into data matrix
    self.updateMap() # draw new data matrix
    self.sendCommand(loop = True) # check for user input and automatically send it

  def initUI(self):
    self.wm_title("Mapping LIDAR") # name window
    self.fig = plt.figure(figsize=(8, 5), dpi=131) # create matplotlib figure
    self.ax = self.fig.add_subplot(111) # add plot to figure
    self.ax.set_title("RPLIDAR Plotting") # name and label plot
    self.ax.set_xlabel("X Position [mm]")
    self.ax.set_ylabel("Y Position [mm]")

    cmap = plt.get_cmap("binary")
    # cmap = plt.get_cmap("jet")
    cmap.set_over("red") # robot is set to higher than maxVal
    self.myImg = self.ax.imshow(self.data.matrix, interpolation="none", cmap=cmap, vmin = 0, vmax = maxVal, # plot data
              extent=[-rad, rad, -rad, rad]) # extent sets labels by matching limits to edges of matrix
    self.ax.set_xlim(-viewSize*1000,viewSize*1000) # pre-zoom image to defined default viewSize
    self.ax.set_ylim(-viewSize*1000,viewSize*1000)
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
    self.bind('r', lambda event: self.resetAll()) # tkinter interrupt function
    self.bind('<Return>', lambda event: self.sendCommand(loop = False)) # tkinter interrupt function

    # create buttons
    tk.Button(self, text="Quit (esc)", command=self.closeWin).pack(side="left", padx = 5, pady = 5) # tkinter interrupt function
    tk.Button(self, text="Restart (r)", command=self.resetAll).pack(side=tk.LEFT, padx = 5) # tkinter interrupt function
    tk.Label(self, text="Command: ").pack(side="left")
    self.entryBox = tk.Entry(master=self)
    self.entryBox.pack(side="left", padx = 5)
    tk.Button(self, text="Send (enter)", command=lambda: self.sendCommand(loop = False)).pack(side=tk.LEFT, padx = 5) # tkinter interrupt function
    tk.Label(self, textvariable = self.numLost).pack(side="left", padx = 5, pady = 5)
    tk.Button(self, text="Save Map", command=self.data.saveImage).pack(side=tk.LEFT, padx = 5) # tkinter interrupt function

  def resetAll(self):
    self.slam = Slam() # restart SLAM (BreezySLAM doesn't provide own reset method)
    self.data.resetData()

  def closeWin(self):
    self.serThread.stop() # tell serial thread to stop running
    self.quit() # kills interpreter (necessary for some reason)
    # self.destroy() # ends this instance of a tk object (not sufficient...?)

  def sendCommand(self, loop=True): # loop indicates how function is called: auto (True) or manual (False)
    # first, figure out which string to send # note that ACK-checking only applies to value-setting commands in 'vwasd'
    if self.serThread.cmdSent and not self.serThread.cmdRcvd:
      resend = True
      strIn = self.serThread.sentCmd # resend previous command
    else:
      strIn = self.entryBox.get() # get new command
      if self.serThread.cmdSent and self.serThread.cmdRcvd: # previous command ACK received
        self.serThread.cmdSent = False # reset state values
        self.serThread.cmdRcvd = False

    # then, send it in the proper manner corresponding to the command
    if strIn: # string is not empty
      if not loop: # manual-send
        self.serThread.writeByte(strIn)
        self.entryBox.delete(0,"end") # clear box after manual-send
      elif strIn in 'WASD' or resend: # auto-send continuous drive commands (capitalized normal commands)
        self.serThread.writeByte(strIn)
      else: pass # wait until manual-send for other commands

    if loop: self.after(commandRate, self.sendCommand) # tkinter interrupt function

  def updateData(self, init = False):
    if self.serQueue.qsize() > numSamp or init:
      self.getScanData() # pull LIDAR data via serial from robot
      self.data.robot_m = self.slam.updateSlam(self.data.dists, self.data.angs) # send data to slam to do stuff
      if init: self.data.robot_m0 = self.data.robot_m # set initial position during initialization
      self.data.drawPoints() # draw points, using updated slam, on the data matrix
    self.after(dataRate, self.updateData) # tkinter interrupt function

  def updateMap(self):
    self.myImg.set_data(self.data.matrix) # 15ms
    self.canvas.draw() #400ms
    self.after(mapRate, lambda: self.updateMap()) # tkinter interrupt function

  def getScanData(self): # 50ms
    i = 0
    while i < numSamp:
      self.data.dists[i], self.data.angs[i] = self.serQueue.get()
      i += 1
    

class Slam(CoreSLAM):
  # updateSlam takes LIDAR data and uses BreezySLAM to calculate the robot's new position

  def __init__(self):
    self.scanLen = 361 # number of points per scan to be passed into BreezySLAM
    CoreSLAM.__init__(self, Laser(self.scanLen, 5.55, 0, +360, 7.0, 0, -0.02), 2*mapSize, mapRes, random_seed=0xabcd)

  def updateSlam(self, dists, angs): # 15ms
    distVec = [0.0 for i in range(self.scanLen)]

    for i in range(numSamp): # create breezySLAM-compatible data from raw scan data
      index = float2int(angs[i])
      if not 0 <= index <= 360: continue
      distVec[index] = dists[i] if distVec[index] == 0 else (dists[i]+distVec[index])/2

    # note that breezySLAM switches the x- and y- axes (their x is forward, 0deg; y is right, +90deg)
    x, y, theta = self.update(distVec, None) # update slam information using particle filtering on LIDAR scan data // 10ms
    return (y, x, theta)


class Data():
  # init creates data matrix and information vectors for processing
  # drawPoints adds scan data to data matrix
  # drawRobot adds robot position to data matrix, generally using slam to find this position
  # saveImage uses PIL to write an image file from the data matrix

  def __init__(self):
    self.matrix = np.zeros((2*mapSize+1, 2*mapSize+1), dtype=int) # initialize data matrix
    self.dists = [0.0 for ind in range(numSamp)] # current scan data
    self.angs = [0.0 for ind in range(numSamp)]
    self.robot_m0 = () # robot location data
    self.robot_m = () # x [m], y [m], th [deg], defined from lower-left corner of map
    self.robot_pix = () # x, y [pix]

  def drawRobot(self, size):
    xLoc = self.robot_pix[0]
    yLoc = self.robot_pix[1]
    th = self.robot_m[2] - self.robot_m0[2]

    robotMat = np.array([[0,0,1,0,0], # shape of robot on map
                         [0,0,1,0,0],
                         [0,0,1,0,0],
                         [0,1,1,1,0],
                         [0,1,1,1,0],
                         [1,1,1,1,1],
                         [1,1,1,1,1]])
    robotMat = rotate(robotMat, -th, output=int)

    rShape = robotMat.shape # rows, columns
    hgt = (rShape[0]-1)/2 # indices of center of robot
    wid = (rShape[1]-1)/2
    for x in range(-wid, wid+1): # relative to center of robot
      for y in range(-hgt, hgt+1):
        if robotMat[y+hgt, x+wid] == 1: # robot should exist here
          self.matrix[yLoc+y, xLoc+x] = robotVal

  def drawPoints(self): # 7ms
    xRobot_mm = 1000*(self.robot_m[0] - self.robot_m0[0]) # displacement from start position (center of map)
    yRobot_mm = 1000*(self.robot_m[1] - self.robot_m0[1])
    Tdeg = self.robot_m[2] - self.robot_m0[2]

    xRobot_pix = mapSize + xRobot_mm*mm2pix # robot wrt map center (mO) + mO wrt matrix origin (xO)  = robot wrt xO [pix]
    yRobot_pix = mapSize - yRobot_mm*mm2pix # negative because np array rows increase downwards (+y_mm = -y_pix = -np rows)
    self.robot_pix = (float2int(xRobot_pix), float2int(yRobot_pix))
    self.drawRobot(1) # new robot position

    for i in range(numSamp):
      x_pix = float2int( xRobot_pix + self.dists[i]*np.sin((self.angs[i]+Tdeg)*deg2rad)*mm2pix ) # pixel location of scan point
      y_pix = float2int( yRobot_pix - self.dists[i]*np.cos((self.angs[i]+Tdeg)*deg2rad)*mm2pix ) # point wrt robot + robot wrt xO
      try:
        self.matrix[y_pix, x_pix] += self.matrix[y_pix, x_pix] < maxVal # increment value at location if below maximum value
      except IndexError:
        print "point out of bounds"

  def resetData(self):
    self.matrix *= 0
    self.robotPos = (0, 0, 0)

  def saveImage(self):
    from PIL import Image # don't have PIL? sorry (try pypng)

    imgData = maxVal - self.matrix # invert colors to make 0 black and maxVal white
    robot = imgData < 0 # save location of robot
    imgData[:] = np.where(robot, 0, imgData) # make robot path black
    GB = (255.0/maxVal*imgData).astype(np.uint8) # scale map data and assign to red, green, and blue layers
    R = GB + (255*robot).astype(np.uint8) # add robot path to red layer

    im = Image.fromarray(np.dstack((R,GB,GB))) # create image from depth stack of three layers
    filename = str(mapRes)+"_pixels_per_meter.png" # filename is map resolution
    im.save(filename) # save image
    print "Image saved to",filename

    import subprocess
    subprocess.call(["eog", filename]) # open with eye of gnome


class SerialThread(threading.Thread):
  def __init__(self, serQueue, numLost):
    super(SerialThread, self).__init__() # nicer way to initialize base class (only works with new-style classes)
    self.serQueue = serQueue
    self.numLost = numLost
    self._stop = threading.Event() # create flag
    self.cmdSent = False
    self.cmdRcvd = False

    portList = sorted([port[0] for port in list_ports.comports() if 'USB' in port[0] or 'COM' in port[0]])
    if not portList: # list empty
      sys.exit("Check your COM ports for connected devices and compatible drivers.")
    elif len(portList) == 1: # if there's only one device connected, use it
      portName = portList[0]
    else: # query user for which port to use
      print "Available COM ports:"
      print portList
      portName = portList[0][0:-1] + raw_input("Please select a port number: ")

    try:
      self.ser = serial.Serial(portName, baudrate=XBEE_BAUD)
    except serial.serialutil.SerialException:
      sys.exit("Invalid port.")
    else:
      time.sleep(1) # give time to connect to serial port
      print "Using %s" % portName

    if raw_input("Would you like to configure the current device's XBee before mapping? [y/N]: ") == 'y':
      while True:
        inputStr = raw_input("Enter XBee command: ")
        sendStr = inputStr if (inputStr == "+++" or inputStr == '') else inputStr + '\x0D'
        self.ser.write(sendStr)
        tstart = time.clock()
        while time.clock() < tstart + 1: # give XBee 1 sec to respond
          if self.ser.inWaiting(): print self.ser.read()
        if inputStr.lower() == "atcn": break
      sys.exit("Please restart XBee and then re-run this code.") # XBee has to power cycle for change to take effect

    while True:
      self.ser.write('l') # tell robot to start lidar
      print "Start command sent to LIDAR... waiting for response..."
      time.sleep(2)
      if self.ser.inWaiting(): break # until it sends something back

    self.start() # put serial data into queue

  def stop(self, try1 = True):
    if try1:
      self._stop.set() # set stop flag to True
      time.sleep(0.2) # give serial reading loop time to finish current point before flushInput()
      # prevents: SerialException: device reports readiness to read but returned no data (device disconnected?)
    self.ser.write('o') # tell robot to turn lidar off
    self.ser.flushInput() # empty input serial buffer
    time.sleep(0.5) # give time to see if data is still coming in
    if self.ser.inWaiting(): self.stop(try1=False)

  def writeByte(self, outByte):
    command = outByte[0]
    if command in 'vwasd': # we're giving the robot a value for a command
      self.ser.write(command) # send the command
      try:
        num = float(outByte[1:])
      except IndexError:
        if command == 'v': print "Invalid command. Enter speed for speed set command."
        else: print "Invalid command. Enter distance/angle for movement commands."
      except ValueError:
        print "Invalid command. Enter a number after command."
      else:
        self.cmdSent = True
        self.sentCmd = outByte
        if command == 'v': self.ser.write(str(num)) # send motor speed as given
        elif command in 'ws': self.ser.write(str(int(mm2ticks*num))) # convert millimeters to ticks
        elif command in 'ad': self.ser.write(str(int(deg2ticks*num))) # convert degrees to ticks

    else:
      self.ser.write(command) # otherwise send only first character

  def run(self):
    tstart = time.clock()
    lagged, missed, total = 0,0,0
    while not self._stop.isSet(): # pulls and processes all incoming serial data
      if self.ser.inWaiting() > 300*PKT_LEN: # data in buffer is too old to be useful (300 packets old)
        lagged += self.ser.inWaiting()/PKT_LEN
        self.ser.flushInput()

      # in order sent (bytes comma-separated):                  dist[0:7], dist[8:12] ang[0:3], ang[4:12]
      # in order received (future data chunks comma-separated): dist[0:12]            ang[0:3], ang[4:12]
      pointLine = self.ser.read(PKT_LEN) # distance and angle (blocking)
      # pointLine = '\x7D\x00\xB4' # 250mm distance and 360deg angle

      if pointLine == NEWLINE*5:
        self.cmdRcvd = True # ACK from Arduino
        continue # move to the next point

      bytes12, byte3 = struct.unpack('<HB',pointLine[0:PKT_LEN-1]) # little-endian 2 bytes and 1 byte
      distCurr = (bytes12 & MASK)/DFAC # 12 least-significant (sent first) bytes12 bits
      angleCurr = ((bytes12 & ~MASK) >> 12 | byte3 << 4)/AFAC # 4 most-significant (sent last) bytes2 bits, 8 byte1 bits

      if pointLine[PKT_LEN-1] == ENDLINE and 100 < distCurr < 6000 and angleCurr <= 360: # valid point
        total += 1
        self.serQueue.put((distCurr, angleCurr))
      else: # data matches what was transmitted
        missed += 1
        while self.ser.read(1) != ENDLINE: pass # delete current packet up to and including endline byte

      if time.clock() > tstart + 1.0: # 1 second later
        tstart += 1.0
        self.numLost.set("Last sec: "+str(lagged)+" lagged, "+str(missed)+" errors out of "+str(total)+" points")
        lagged, missed, total = 0,0,0

if __name__ == '__main__':
  root = Root()
  # print "window actual:",root.winfo_width(),root.winfo_height()
  # print "window desired:",root.winfo_reqwidth(),root.winfo_reqheight()
  root.mainloop()
