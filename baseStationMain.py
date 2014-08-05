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

from slambotgui.maps import DataMatrix
from slambotgui.slams import Slam
from slambotgui.comms import SerialThread

# User preferences
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
MAP_SIZE_PIXELS = int(MAP_SIZE_M*MAP_RES_PIX_PER_M) # number of pixels across the entire map
MAP_DEPTH = 5 # depth of data points on map (levels of certainty)
print("Each pixel is " + str(round(1000.0/MAP_RES_PIX_PER_M,1)) + "mm, or " + str(round(1000.0/MAP_RES_PIX_PER_M/25.4,2)) + "in.")

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

# pack up constants to pass to things that need them
DATA_KWARGS = {'MAP_SIZE_PIXELS':MAP_SIZE_PIXELS,'INSET_SIZE_M':INSET_SIZE_M,'MAP_RES_PIX_PER_M':MAP_RES_PIX_PER_M,'MAP_DEPTH':MAP_DEPTH}
SLAM_KWARGS = {'MAP_SIZE_PIXELS':MAP_SIZE_PIXELS,'MAP_SIZE_M':MAP_SIZE_M,'ANG_MAX':ANG_MAX,'DIST_MAX':DIST_MAX,'TICK_2_MM':TICK_2_MM,'TICK_2_DEG':TICK_2_DEG}
SER_KWARGS = {'DIST_MIN':DIST_MIN,'DIST_MAX':DIST_MAX,'ANG_MIN':ANG_MIN,'ANG_MAX':ANG_MAX}

def main():
  # app window setup
  root = tk.Tk() # create tkinter window
  root.wm_title("Aerospace Robotics LIDAR Viewer") # name window
  root.geometry('+100+100') # position windows 100,100 pixels from top-left corner
  root.lower() # bring terminal to front

  # helper object setup
  data = DataMatrix(**DATA_KWARGS)
  slam = Slam(dataFile if LOG_ALL_DATA else None, **SLAM_KWARGS)
  queues = [Queue(), Queue(), Queue()]
  serThread = SerialThread(*queues, **SER_KWARGS)

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
  from Queue import Empty as QueueEmpty
elif pythonSeries == 3:
  import tkinter as tk
  from tkinter.font import Font
  from tkinter.messagebox import askokcancel
  from queue import Queue
  from queue import Empty as QueueEmpty
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

  def initUI(self):
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
      self.data = DataMatrix(**DATA_KWARGS)
      self.slam = Slam(dataFile if LOG_ALL_DATA else None, **SLAM_KWARGS)
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


if __name__ == '__main__':
  if LOG_ALL_DATA: dataFile = open(LOGFILE_NAME+'.log','w')
  main()
  if LOG_ALL_DATA: dataFile.close()
