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


from sys import version_info
import os
print("Python {}.{}.{}".format(*version_info[0:3]))

# used in Root class
if version_info[0] == 2:
  from Tkinter import Tk, StringVar
  from tkMessageBox import askokcancel
elif version_info[0] == 3:
  from tkinter import Tk, StringVar
  from tkinter.messagebox import askokcancel
from slambotgui.maps import DataMatrix
from slambotgui.slams import Slam
from slambotgui.guis import MatplotlibMaps, StatusButtons
from slambotgui.components import DaguRover5, RPLIDAR
from slambotgui.cvslamshow import SlamShow
paddedStr = lambda inStr, length: '{0: <{width}s}'.format(inStr, width=length)[0:length] if length != 0 else inStr

# GUI constants
DATA_RATE = 10 # minimum time between updating data from lidar [ms]
MAP_RATE = 100 # minimum time between updating map [ms]

# Protocol constants
NUM_SAMP = 370 # number of serial packets needed for 1 scan (guesstimate)

# User preferences
INTERNAL_MAP = True
FAST_MAPPING = True
dataFile = None # keeps log file from being written to
logName = os.path.join('examples','data_6AUG14_20m.log')

# Laser constants (shared with Arduino)
DIST_MIN = 100; # minimum distance
DIST_MAX = 6000; # maximum distance

# Map constants
MAP_SIZE_M = 16.0 # size of region to be mapped [m]
INSET_SIZE_M = 2.0 # size of relative map
MAP_RES_PIX_PER_M = 50 # number of pixels of data per meter [pix/m]
MAP_SIZE_PIXELS = int(MAP_SIZE_M*MAP_RES_PIX_PER_M) # number of pixels across the entire map
MAP_DEPTH = 5 # depth of data points on map (levels of certainty)
print("Each pixel is " + str(round(1000.0/MAP_RES_PIX_PER_M,1)) + "mm, or " + str(round(1000.0/MAP_RES_PIX_PER_M/25.4,2)) + "in.")

KWARGS, gvars = {}, globals()
for var in ['dataFile','MAP_SIZE_M','INSET_SIZE_M','MAP_RES_PIX_PER_M','MAP_SIZE_PIXELS','MAP_DEPTH']:
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

    # initialize root variables
    self.statusStr = StringVar() # status of serThread
    self.restarting = False # are we in the process of soft restarting?
    self.paused = False # should the loops be doing nothing right now?
    self.fileIndex = 0
    self.fileContents = [[int(el) for el in rawline.split(' ')] for rawline in dataFile.read().strip().split('\n')]

    # helper objects
    self.data = DataMatrix(**KWARGS) # handle map data
    self.slam = Slam(self.robot, self.laser, **KWARGS) # do slam processing

    if FAST_MAPPING:
      # create the OpenCV window
      self.outFrame = SlamShow(MAP_SIZE_PIXELS, MAP_RES_PIX_PER_M/1000.0, 'SLAM Rover: Hit ESC to quit')
      # create Tkinter control bar
      self.inFrame = StatusButtons(self.master, self.closeWin, self.restartAll, self.saveImage, self.statusStr)
      # pack frame
      self.inFrame.pack(side="left", fill='both')
    else:
      # create all the pretty stuff in the Tkinter window
      self.outFrame = MatplotlibMaps(self.master, self.data.getMapMatrix(), self.data.getInsetMatrix(), **KWARGS)
      self.inFrame = StatusButtons(self.master, self.closeWin, self.restartAll, self.saveImage, self.statusStr)
      # pack frames
      self.outFrame.pack(side="top", fill='both')
      self.inFrame.pack(side="left", fill='both', expand=True)

    # Start loops
    self.updateData() # pull data from queue, put into data matrix
    self.updateMap() # draw new data matrix

  def closeWin(self):
    self.paused = True
    self.statusStr.set(paddedStr("Paused",len(self.statusStr.get()))) # keep length of label constant
    if askokcancel("Quit?", "Are you sure you want to quit?"):
      self.statusStr.set(paddedStr("Stopping",len(self.statusStr.get()))) # keep length of label constant
      print("Closing program")
      self.master.quit() # kills interpreter (necessary for some reason)
    else: self.paused = False

  def restartAll(self, funcStep=0): # two-part initialization to be re-done at soft reset
    if funcStep == 0:
      self.restarting = True # stop currently running loops
      self.master.after(2000, lambda: self.restartAll(funcStep=1)) # let processor wrap things up elsewhere # should be smarter
      return
    elif funcStep == 1:
      self.restarting = False
      self.fileIndex = 0
      self.data = DataMatrix(**KWARGS)
      self.slam = Slam(self.robot, self.laser, **KWARGS)
      self.updateData() # pull data from queue, put into data matrix
      self.updateMap() # draw new data matrix

  def saveImage(self): # function prototype until data is initialized
    self.statusStr.set(paddedStr("Saving. Close image to resume.", len(self.statusStr.get()))) # keep length of label constant
    self.updateMap(loop=False) # make sure we save the newest map
    self.data.saveImage()

  def getScanData(self, repeat=False):
    scan = self.fileContents[self.fileIndex]

    self.slam.currEncPos = scan[0:3]
    self.points = [point for point in zip(scan[3:], range(self.laser.SCAN_SIZE))] # distance, angle tuples

    self.fileIndex += 1

  def updateData(self, init=True):
    self.dataInit = init
    if not self.paused and self.fileIndex < len(self.fileContents):
      self.statusStr.set('Scan number: {0:4d} of file "{1:s}"'.format(self.fileIndex, logName))

      self.getScanData() # pull data from log file

      # update robot position
      if init: self.slam.prevEncPos = self.slam.currEncPos # set both values the first time through
      self.data.getRobotPos(self.slam.updateSlam(self.points), init=init) # send data to slam to do stuff # 15ms

      if init: init = False # initial data gathered successfully

    if not self.restarting: self.master.after(DATA_RATE, lambda: self.updateData(init=init))
    else: self.statusStr.set(paddedStr("Restarting", len(self.statusStr.get()))) # if loop ends, we're restarting

  def updateMap(self, loop=True):
    if not self.paused and not self.dataInit: # wait until first data update to update map
      if INTERNAL_MAP:
        self.data.drawBreezyMap(self.slam.getBreezyMap()) # draw map using slam data # 16ms
      else:
        self.data.drawMap(self.points) # draw map using scan points

      if FAST_MAPPING:
        # Display map and robot position, quitting on ESC
        self.outFrame.displayMap(self.slam.getBreezyMap())
        self.outFrame.displayRobot(self.slam.getpos())
        if self.outFrame.refresh() == 27: self.closeWin()
      else:
        # Update display frame with new map
        self.data.drawInset() # new relative map # 6ms
        self.outFrame.updateMaps(self.data.get_robot_rel(), self.data.getMapMatrix(), self.data.getInsetMatrix())
    if loop and not self.restarting: self.master.after(MAP_RATE, self.updateMap)


if __name__ == '__main__':
  print(GPL)
  dataFile = open(logName,'r')
  main()
  dataFile.close()
