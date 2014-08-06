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
pythonSeries = version_info[0]
print("Python {}.{}.{}".format(*version_info[0:3]))

from slambotgui.maps import DataMatrix
from slambotgui.slams import Slam
from slambotgui.comms import SerialThread
from slambotgui.guis import MatplotlibMaps

# User preferences
INTERNAL_MAP = True
LOG_ALL_DATA = True
if LOG_ALL_DATA: dataFile = open('test.log','w')
else: dataFile = None

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

KWARGS, gvars = {}, globals()
for var in ['LOG_ALL_DATA','dataFile','DIST_MIN','DIST_MAX','ANG_MIN','ANG_MAX', \
            'MAP_SIZE_M','INSET_SIZE_M','MAP_RES_PIX_PER_M','MAP_SIZE_PIXELS','MAP_DEPTH','TICK_2_MM','TICK_2_DEG']:
  KWARGS[var] = gvars[var] # constants required in modules


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
paddedStr = lambda inStr, length: '{0: <{width}s}'.format(inStr, width=length)[0:length] if length != 0 else inStr
# GUI constants
CMD_RATE = 100 # minimum time between auto-send commands [ms]
DATA_RATE = 50 # minimum time between updating data from lidar [ms]
MAP_RATE = 500 # minimum time between updating map [ms]
MAX_TX_TRIES = 3 # max number of times to try to resend command before giving up
VIEW_SIZE_M = 8.0 # default size of region to be shown in display [m]
# Protocol constants
NUM_SAMP = 370 # number of serial packets needed for 1 scan (guesstimate)
CMDS = {'v':{'prop':"speed 0..255", 'func':lambda x: str(x)                 }, # send motor speed as given
        'w':{'prop':"forward mm",   'func':lambda x: str(int(MM_2_TICK*x))  }, # convert mm to ticks
        'a':{'prop':"left deg",     'func':lambda x: str(int(DEG_2_TICK*x)) }, # convert degrees to ticks
        's':{'prop':"reverse mm",   'func':lambda x: str(int(MM_2_TICK*x))  }, # convert mm to ticks
        'd':{'prop':"right deg",    'func':lambda x: str(int(DEG_2_TICK*x)) }} # convert degrees to ticks

def main():
  root = tk.Tk() # create tkinter window
  root.lower() # send tkinter window to back

  # create main app
  app = App(root)

  # Start loops
  app.serThread.start() # begin fetching data from serial port and processing it
  app.updateData() # pull data from queue, put into data matrix
  app.updateMap() # draw new data matrix
  app.autosendCommand() # check for user input and automatically send it

  # Bring GUI to front # app.master = root
  root.lift() # bring tk window to front if initialization finishes
  root.focus_force() # make tk window active one (so you don't need to click on it for hotkeys to work)
  root.mainloop() # start Tkinter GUI loop

class App:
  # init            creates all objects, draws initUI, and starts all loops (including serial thread)
  # resetAll        restarts all objects that store map data, allowing history to be wiped without hard reset
  # closeWin        first prompts the user if they really want to close, then ends serial thread and tkinter
  # saveImage       tells data object to capture the current map and save as a png
  # getScanData     pulls LIDAR data directly from the serial port and does preliminary processing
  # updateData      calls getScanData, and updates the slam object and data matrix with this data, and loops to itself
  # updateMap       draws a new map, using whatever data is available, and loops to itself
  # sendCommand     triggered by request to manually send the command in the text box
  # autoSendCommand loop to control sending of commands, including automatic retries and continuous drive commands

  def __init__(self, master):
    self.master = master # root tk window
    self.master.protocol("WM_DELETE_WINDOW", self.closeWin) # control what happens when a window is closed externally (e.g. by the 'x')
    self.master.wm_title("Aerospace Robotics LIDAR Viewer") # name window
    self.master.geometry('+100+100') # position windows 100,100 pixels from top-left corner

    # initialize serial object, prompting user for input if required
    self.statusQueue = Queue() # status of serial thread # FIFO queue by default
    self.RXQueue = Queue() # data from serial to root # FIFO queue by default
    self.TXQueue = Queue() # data from root to serial # FIFO queue by default
    self.serThread = SerialThread(self.statusQueue, self.RXQueue, self.TXQueue, **KWARGS) # initialize thread object

    # initialize root variables
    self.statusStr = tk.StringVar() # status of serThread
    self.resetting = False # are we in the process of soft restarting?
    self.paused = False # should the loops be doing nothing right now?
    self.sendingCommand = False # are we trying to manually send a command?

    # helper objects
    self.data = DataMatrix(**KWARGS) # handle map data
    self.slam = Slam(**KWARGS) # do slam processing

    # create all the pretty stuff in the Tkinter window
    self.outFrame = MatplotlibMaps(self.master, self.data.getMapMatrix(), self.data.getInsetMatrix(), **KWARGS)
    self.ioFrame = tk.Frame(master, bd=5, relief=tk.SUNKEN)

    # create buttons
    tk.Button(self.ioFrame, text="Quit (esc)", command=self.closeWin).pack(side="left", padx=5, pady=5) # tkinter interrupt function
    tk.Button(self.ioFrame, text="Restart (R)", command=self.restartAll).pack(side=tk.LEFT, padx = 5) # tkinter interrupt function
    tk.Label(self.ioFrame, text="Command: ").pack(side="left")
    self.entryBox = tk.Entry(master=self.ioFrame, width=15)
    self.entryBox.pack(side="left", padx = 5)
    tk.Button(self.ioFrame, text="Send (enter)", command=self.sendCommand).pack(side=tk.LEFT, padx=5) # tkinter interrupt function
    monospaceFont = Font(family="Courier", weight='bold', size=12)
    tk.Label(self.ioFrame, textvariable=self.statusStr, font=monospaceFont).pack(side="left", padx=5, pady=5)
    tk.Button(self.ioFrame, text="Save Map", command=self.saveImage).pack(side=tk.LEFT, padx=5) # tkinter interrupt function

    # bind keyboard inputs to functions
    self.master.bind('<Escape>', lambda event: self.closeWin()) # escape exits program after prompt
    self.master.bind('<Shift-R>', lambda event: self.restartAll()) # shift and capital R does a soft reset
    self.entryBox.bind('<Return>', lambda event: self.sendCommand()) # enter sends the command in the command box

    # pack frames
    self.outFrame.pack(side="top", fill='both')
    self.inFrame.pack(side="left", fill='both', expand=True)

  def restartAll(self, funcStep=0): # two-part initialization to be re-done at soft reset
    if funcStep == 0:
      self.resetting = True # stop currently running loops
      self.master.after(2000, lambda: self.restartAll(funcStep=1)) # let processor wrap things up elsewhere # should be smarter
      return
    elif funcStep == 1:
      self.resetting = False
      self.data = DataMatrix(**KWARGS)
      self.slam = Slam(**KWARGS)
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
    else: self.paused = False

  def saveImage(self): # function prototype until data is initialized
    self.statusStr.set(paddedStr("Saving. Close image to resume.", len(self.statusStr.get()))) # keep length of label constant
    self.updateMap(loop=False) # make sure we save the newest map
    self.data.saveImage()

  def getScanData(self, repeat=False):
    self.points = [] # wipe old data before writing new data
    while True:
      # Make sure there's actually data to get
      try: queueItem = self.RXQueue.get_nowait()
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
      try: status = self.statusQueue.get_nowait() # do this before anything else
      except QueueEmpty: pass
      else: self.statusStr.set(paddedStr(status, len(self.statusStr.get())))

      if self.RXQueue.qsize() > (2 if init else 1)*NUM_SAMP: # ready to pull new scan data
        # pull data from serial thread via RXQueue, ignoring the first scan, which is incomplete
        self.getScanData(repeat=init) # 2ms

        # update robot position
        if init: self.slam.prevEncPos = self.slam.currEncPos # set both values the first time through
        self.data.getRobotPos(self.slam.updateSlam(self.points), init=init) # send data to slam to do stuff # 15ms

        if init: init = False # initial data gathered successfully

    if not self.resetting: self.master.after(DATA_RATE, lambda: self.updateData(init=init))
    else: self.statusStr.set(paddedStr("Restarting", len(self.statusStr.get()))) # if loop ends, we're restarting

  def updateMap(self, loop=True):
    if not self.paused and not self.dataInit: # wait until first data update to update map
      if INTERNAL_MAP: self.data.drawBreezyMap(self.slam.getBreezyMap()) # draw map using slam data # 16ms
      else: self.data.drawMap(self.points) # draw map using scan points
      self.data.drawInset() # new relative map # 6ms
      self.outFrame.updateMaps(self.data.get_robot_rel(), self.data.getMapMatrix(), self.data.getInsetMatrix())
    if loop and not self.resetting: self.master.after(MAP_RATE, self.updateMap)

  def sendCommand(self):
    self.sendingCommand = True

  def autosendCommand(self, numTries=0, wantACK=False, strIn='', strOut=''):
    # expecting ACK from Arduino (only for value-setting commands in 'vwasd')
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
        if command in list(CMDS): # we're giving the robot a value for a command
          try:
            num = int(strIn[1:])
          except ValueError:
            self.entryBox.delete(1,"end")
            self.entryBox.insert(1,"[{}]".format(CMDS[command]['prop'])) # prompt user with proper command format
            self.entryBox.selection_range(1,'end')
          else:
            wantACK = True # start resending the command if it isn't received
            strOut = command + CMDS[command]['func'](num) # re-create command with converted values
            self.TXQueue.put(strOut)
        else: # otherwise send only first character
          self.TXQueue.put(command)
          self.entryBox.delete(0,"end") # clear box

      else:
        pass # wait until manual-send for other commands
    
    self.sendingCommand = False # reset manual sending
    self.master.after(CMD_RATE, lambda: self.autosendCommand(numTries=numTries, wantACK=wantACK, strIn=strIn, strOut=strOut))


if __name__ == '__main__':
  main()
  if LOG_ALL_DATA: dataFile.close()
