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

import sys, os, time
print("Python {}.{}.{}".format(*sys.version_info[0:3]))

sys.path.append('libraries/slambotgui_source') # look in dev package

# used in Root class
if sys.version_info[0] == 2:
  from Tkinter import Tk, StringVar
  from tkMessageBox import askokcancel
elif sys.version_info[0] == 3:
  from tkinter import Tk, StringVar
  from tkinter.messagebox import askokcancel
from slambotgui.maps import DataMatrix
from slambotgui.slams import Slam
from slambotgui.guis import RegionFrame, InsetFrame, StatusFrame
from slambotgui.components import DaguRover5, RPLIDAR
paddedStr = lambda inStr, length: '{0: <{width}s}'.format(inStr, width=length)[0:length] if length != 0 else inStr

# User preferences
INTERNAL_MAP = True
FAST_MAPPING = False
logFileName = 'data_6AUG14_16m.log'
logFilePath = os.path.join('examples',logFileName)
logFile = open(logFilePath, 'r')
if FAST_MAPPING: from slambotgui.cvslamshow import SlamShow # uses OpenCV

# SLAM preferences
USE_ODOMETRY = True
MAP_QUALITY = 3

# GUI constants
DATA_RATE = 5 # minimum time between updating data from lidar [ms]
MAP_RATE = 100 # minimum time between updating map [ms]

# Laser constants (shared with Arduino)
DIST_MIN = 100; # minimum distance
DIST_MAX = 6000; # maximum distance

# Map constants
MAP_SIZE_M = 20.0 # size of region to be mapped [m]
INSET_SIZE_M = 2.0 # size of relative map
MAP_RES_PIX_PER_M = 100 # number of pixels of data per meter [pix/m]
MAP_SIZE_PIXELS = int(MAP_SIZE_M*MAP_RES_PIX_PER_M) # number of pixels across the entire map
CV_IMG_SIZE = 800 # size of OpenCV image, used if FAST_MAPPING
CV_IMG_RES_PIX_PER_MM = CV_IMG_SIZE/MAP_SIZE_M/1000 # number of pixels of data per meter [pix/mm]
MAP_DEPTH = 5 # depth of data points on map (levels of certainty)
print("Each pixel is " + str(round(1000.0/MAP_RES_PIX_PER_M,1)) + "mm, or " + str(round(1000.0/MAP_RES_PIX_PER_M/25.4,2)) + "in.")

KWARGS, gvars = {}, globals()
for var in ['MAP_SIZE_M','INSET_SIZE_M','MAP_RES_PIX_PER_M','MAP_DEPTH','USE_ODOMETRY','MAP_QUALITY']:
  KWARGS[var] = gvars[var] # constants required in modules

def main():
  root = Tk() # create tkinter window
  root.lower() # send tkinter window to back

  # create main app
  app = App(root)

  # Bring GUI to front # app.master = root
  root.lift() # bring tk window to front if initialization finishes
  root.focus_force() # make tk window active one (so you don't need to click on it for hotkeys to work)
  root.mainloop() # start Tkinter GUI loop


class App:
  # init            creates all objects, draws initUI, and starts all loops (including serial thread)
  # closeWin        first prompts the user if they really want to close, then ends serial thread and tkinter
  # resetAll        restarts all objects that store map data, allowing history to be wiped without hard reset
  # saveImage       tells data object to capture the current map and save as a png
  # getScanData     pulls LIDAR data directly from the serial port and does preliminary processing
  # updateData      calls getScanData, and updates the slam object and data matrix with this data, and loops to itself
  # updateMap       draws a new map, using whatever data is available, and loops to itself

  def __init__(self, master):
    self.master = master # root tk window
    self.master.protocol("WM_DELETE_WINDOW", self.closeWin) # control what happens when a window is closed externally (e.g. by the 'x')
    self.master.wm_title("Aerospace Robotics LIDAR Viewer") # name window
    self.master.geometry('+100+100') # position window 100,100 pixels from top-left corner

    # physical objects
    self.robot = DaguRover5()
    self.laser = RPLIDAR(DIST_MIN, DIST_MAX)

    # get data from log file
    self.logFileIndex = 0
    self.logFileContents = [[int(el) for el in rawline.split(' ')] for rawline in logFile.read().strip().split('\n')]

    # initialize root variables
    self.statusStr = StringVar() # status of serThread
    self.restarting = False # are we in the process of soft restarting?
    self.paused = False # should the loops be doing nothing right now?

    # helper objects
    self.data = DataMatrix(**KWARGS) # handle map data
    self.slam = Slam(self.robot, self.laser, **KWARGS) # do slam processing

    if FAST_MAPPING:
      # create the OpenCV window
      self.regionFrame = SlamShow(CV_IMG_SIZE, CV_IMG_RES_PIX_PER_MM, 'SLAM Rover: Hit ESC to quit')
      # create Tkinter control frames
      self.statusFrame = StatusFrame(self.master, self.closeWin, self.restartAll, self.saveImage, self.statusStr, twoLines=True)
      self.insetFrame = InsetFrame(self.master, self.data.getInsetMatrix(), **KWARGS)
      # pack frame
      self.statusFrame.pack(side='bottom', fill='x')
      self.insetFrame.pack(side='left', fill='both', expand=True)
    else:
      # create all the pretty stuff in the Tkinter window
      self.statusFrame = StatusFrame(self.master, self.closeWin, self.restartAll, self.saveImage, self.statusStr)
      self.regionFrame = RegionFrame(self.master, self.data.getMapMatrix(), **KWARGS)
      self.insetFrame = InsetFrame(self.master, self.data.getInsetMatrix(), **KWARGS)
      # pack frames
      self.statusFrame.pack(side='bottom', fill='x')
      self.regionFrame.pack(side='left', fill='both', expand=True)
      self.insetFrame.pack(side='right', fill='both', expand=True)

    # Start loops
    self.updateData() # pull data from queue, put into data matrix
    self.updateMap() # draw new data matrix

  def closeWin(self):
    self.paused = True
    self.statusStr.set(paddedStr("Paused.", len(self.statusStr.get())))
    if askokcancel("Quit?", "Are you sure you want to quit?"):
      self.statusStr.set(paddedStr("Stopping",len(self.statusStr.get()))) # keep length of label constant
      print("Closing program")
      self.master.quit() # kills interpreter (necessary for some reason)
    else: self.paused = False

  def restartAll(self, funcStep=0): # two-part initialization to be re-done at soft reset
    if funcStep == 0:
      self.restarting = True # stop currently running loops
      self.master.after(1000, lambda: self.restartAll(funcStep=1)) # let processor wrap things up elsewhere # should be smarter
      return
    elif funcStep == 1:
      self.logFileIndex = 0 # read from beginning of log file
      self.data = DataMatrix(**KWARGS)
      self.slam = Slam(self.robot, self.laser, **KWARGS)
      self.restarting = False
      self.updateData() # pull data from queue, put into data matrix
      self.updateMap() # draw new data matrix

  def saveImage(self, step=0): # function prototype until data is initialized
    self.paused = True
    self.statusStr.set(paddedStr("Saving image. Close to resume.", len(self.statusStr.get()))) # keep length of label constant
    self.master.update() # force statusStr update
    self.updateMap(loop=False) # make sure we save the newest map
    self.data.saveImage()
    self.paused = False

  def getScanData(self, repeat=False):
    scan = self.logFileContents[self.logFileIndex]

    self.slam.currEncPos = scan[0:3]
    self.points = [point for point in zip(scan[3:], range(self.laser.SCAN_SIZE))] # distance, angle tuples

    self.logFileIndex += 1

  def updateData(self, init=True):
    self.dataInit = init
    if not self.paused and self.logFileIndex < len(self.logFileContents):
      self.statusStr.set('Scan number {0:4d} from {1:s}'.format(self.logFileIndex, logFileName))

      # pull data from log file
      self.getScanData()

      # update robot position
      if init: self.slam.prevEncPos = self.slam.currEncPos # set both values the first time through
      self.data.getRobotPos(self.slam.updateSlam(self.points), init=init) # send data to slam to do stuff # 15ms

      if not INTERNAL_MAP: self.data.drawMap(self.points) # draw map using scan points

      if init: init = False # initial data gathered successfully

    elif self.logFileIndex >= len(self.logFileContents):
      self.statusStr.set(paddedStr("End of data.", len(self.statusStr.get())))

    if not self.restarting: self.master.after(DATA_RATE, lambda: self.updateData(init=init))
    else: self.statusStr.set(paddedStr("Restarting...", len(self.statusStr.get()))) # if loop ends, we're restarting

  def updateMap(self, loop=True):
    if not self.paused and not self.dataInit: # wait until first data update to update map
      if INTERNAL_MAP:
        # draw map using slam data # 16ms
        self.data.drawBreezyMap(self.slam.getBreezyMap())

      if FAST_MAPPING:
        self.data.drawInset() # new relative map # 7ms
        self.insetFrame.updateMap(self.data.get_robot_rel(), self.data.getInsetMatrix()) # 25ms
        self.regionFrame.displayMap(self.data.getMapArray((CV_IMG_SIZE,CV_IMG_SIZE))) # 36ms
        self.regionFrame.displayRobot(self.data.get_robot_abs())
        if self.regionFrame.refresh() == 27: self.closeWin() # ESC key pressed
      else:
        self.data.drawInset() # new relative map # 7ms
        self.insetFrame.updateMap(self.data.get_robot_rel(), self.data.getInsetMatrix())
        self.regionFrame.updateMap(self.data.get_robot_rel(), self.data.getMapMatrix())
    if loop and not self.restarting: self.master.after(MAP_RATE, self.updateMap)


if __name__ == '__main__':
  print(GPL)
  main()
  logFile.close()
