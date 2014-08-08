#!/usr/bin/env python

# guis.py - GUI frames for Tkinter
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


from sys import version_info

if version_info[0] == 2: import Tkinter as tk
elif version_info[0] == 3: import tkinter as tk


import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2TkAgg
from matplotlib.gridspec import GridSpec

VIEW_SIZE_M = 6.0 # default size of region to be shown in display [m]

class MatplotlibMaps(tk.Frame): # tkinter frame, inheriting from the tkinter Frame class
  # init            draws all fields in the output frame of the main App
  # updateMaps      draws all map data onto the matplotlib figures
  # removeMarkers   deletes all temporary markers on the main map (old robot position)
  # drawMarker      draws robot on main map using matplotlib marker

  def __init__(self, master, mapMatrix, insetMatrix, MAP_SIZE_M=8, INSET_SIZE_M=2, **unused):
    tk.Frame.__init__(self, master, bd=5, relief='sunken') # explicitly initialize base class and create window
    self.markers = [] # current matplotlib markers

    # current (and only) figure
    self.fig = plt.figure(figsize=(9, 5), dpi=131.2, facecolor=self.master.cget('bg')) # create matplotlib figure (dpi calculated from $ xrandr)
    gs = GridSpec(1,3) # layout of plots in figure

    # plot color settings
    cmap = plt.get_cmap('gray') # opposite of "binary"

    # subplot 1 (stationary map)
    self.ax1 = plt.subplot(gs[0,:2]) # add plot 1 to figure
    self.ax1.set_title("Region Map") # name and label plot
    self.ax1.set_xlabel("X Position [mm]")
    self.ax1.set_ylabel("Y Position [mm]")
    self.myImg1 = self.ax1.imshow(mapMatrix, interpolation="none", cmap=cmap, vmin=0, vmax=255, # plot data
              extent=[-MAP_SIZE_M/2, MAP_SIZE_M/2, -MAP_SIZE_M/2, MAP_SIZE_M/2]) # extent sets labels by matching limits to edges of matrix
    self.ax1.set_xlim(-VIEW_SIZE_M/2, VIEW_SIZE_M/2) # pre-zoom image to defined default MAP_SIZE_M
    self.ax1.set_ylim(-VIEW_SIZE_M/2, VIEW_SIZE_M/2)

    # colorbar
    self.cbar = self.fig.colorbar(self.myImg1, orientation='vertical') # create colorbar

    # subplot 2 (relative map)
    self.ax2 = plt.subplot(gs[0,2]) # add plot 2 to figure
    self.ax2.set_title("Robot Environs") # name and label plot
    self.ax2.set_xlabel("", family='monospace')
    self.myImg2 = self.ax2.imshow(insetMatrix, interpolation='none', cmap=cmap, vmin=0, vmax=255, # plot data
              extent=[-INSET_SIZE_M/2, INSET_SIZE_M/2, -INSET_SIZE_M/2, INSET_SIZE_M/2])
    self.drawMarker(self.ax2, (0,0,0), temporary=False) # draw permanent robot at center of inset map

    # do fancy stuff with tkinter and matplotlib
    self.canvas = FigureCanvasTkAgg(self.fig, master=self) # master of the fig canvas is this frame
    self.canvas._tkcanvas.pack(fill='both', expand=True)
    NavigationToolbar2TkAgg(self.canvas, self)

  def updateMaps(self, robotRel, mapMatrix, insetMatrix):
    # send maps to image object
    self.myImg1.set_data(mapMatrix) # 20ms
    self.myImg2.set_data(insetMatrix) # 0.4ms

    # finishing touches
    self.removeMarkers() # delete old robot position from map
    self.drawMarker(self.ax1, robotRel) # add new robot position to map # 0.2ms
    self.ax2.set_xlabel('X = {0:6.1f}; Y = {1:6.1f};\nHeading = {2:6.1f}'.format(*robotRel))

    # refresh the figure
    self.canvas.draw() # 200ms

  def removeMarkers(self):
    for i in range(len(self.markers)):
      self.markers[i].remove()
      del self.markers[i]

  def drawMarker(self, ax, pos, temporary=True):
    marker = ax.plot(pos[0]/1000, pos[1]/1000, markersize=8, color='red', marker=(3,1,-pos[2]), markeredgewidth=0, aa=False)
    if temporary: self.markers.extend(marker) # marker is a list of matplotlib.line.Line2D objects


######################################################################################


if version_info[0] == 2: from tkFont import Font
elif version_info[0] == 3: from tkinter.font import Font

CMD_RATE = 100 # minimum time between auto-send commands [ms]
MAX_TX_TRIES = 3 # max number of times to try to resend command before giving up

class EntryButtons(tk.Frame):
  # sendCommand     triggered by request to manually send the command in the text box
  # autoSendCommand loop to control sending of commands, including automatic retries and continuous drive commands

  def __init__(self, master, robot, closeWin, restartAll, saveImage, getACK, resetACK, TXQueue, statusStr):
    tk.Frame.__init__(self, master, bd=5, relief='sunken') # explicitly initialize base class and create window
    self.master = master

    self.sendingCommand = False # are we trying to manually send a command?
    self.CMDS = {'v':{'prop':"speed 0..255", 'func':lambda x: str(x)                        }, # send motor speed as given
                 'w':{'prop':"forward mm",   'func':lambda x: str(int(robot.MM_2_TICK*x))   }, # convert mm to ticks
                 'a':{'prop':"left deg",     'func':lambda x: str(int(robot.DEG_2_TICK*x))  }, # convert degrees to ticks
                 's':{'prop':"reverse mm",   'func':lambda x: str(int(robot.MM_2_TICK*x))   }, # convert mm to ticks
                 'd':{'prop':"right deg",    'func':lambda x: str(int(robot.DEG_2_TICK*x))  }} # convert degrees to ticks

    self.closeWin, self.restartAll, self.saveImage, self.getACK, self.resetACK, self.TXQueue, self.statusStr \
      =  closeWin,      restartAll,      saveImage,      getACK,      resetACK,      TXQueue, statusStr


    # create buttons
    tk.Button(self, text="Quit (esc)", command=self.closeWin).pack(side="left", padx=5, pady=5) # tkinter interrupt function
    tk.Button(self, text="Restart (R)", command=self.restartAll).pack(side=tk.LEFT, padx = 5) # tkinter interrupt function
    tk.Label(self, text="Command: ").pack(side="left")
    self.entryBox = tk.Entry(master=self, width=15)
    self.entryBox.pack(side="left", padx = 5)
    tk.Button(self, text="Send (enter)", command=self.sendCommand).pack(side=tk.LEFT, padx=5) # tkinter interrupt function
    monospaceFont = Font(family="Courier", weight='bold', size=12)
    tk.Label(self, textvariable=self.statusStr, font=monospaceFont).pack(side="left", padx=5, pady=5)
    tk.Button(self, text="Save Map", command=self.saveImage).pack(side=tk.LEFT, padx=5) # tkinter interrupt function

    # bind keyboard inputs to functions
    master.bind('<Escape>', lambda event: self.closeWin()) # escape exits program after prompt
    master.bind('<Shift-R>', lambda event: self.restartAll()) # shift and capital R does a soft reset
    self.entryBox.bind('<Return>', lambda event: self.sendCommand()) # enter sends the command in the command box

  def sendCommand(self):
    self.sendingCommand = True

  def autosendCommand(self, numTries=0, wantACK=False, strIn='', strOut=''):
    # expecting ACK from Arduino (only for value-setting commands in 'vwasd')
    if wantACK:
      gotACK = self.getACK() # has the serial thread received an ACK?

      # either ACK received or command resent too many times
      if gotACK or numTries >= MAX_TX_TRIES:
        numTries, wantACK = 0, False # reset stuff
        self.entryBox.delete(0,"end") # clear box if we're done sending command
        self.resetACK() # tell serial thread that we got the ACK

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
        if command in list(self.CMDS): # we're giving the robot a value for a command
          try:
            num = int(strIn[1:])
          except ValueError:
            self.entryBox.delete(1,"end")
            self.entryBox.insert(1,"[{}]".format(self.CMDS[command]['prop'])) # prompt user with proper command format
            self.entryBox.selection_range(1,'end')
          else:
            wantACK = True # start resending the command if it isn't received
            strOut = command + self.CMDS[command]['func'](num) # re-create command with converted values
            self.TXQueue.put(strOut)
        else: # otherwise send only first character
          self.TXQueue.put(command)
          self.entryBox.delete(0,"end") # clear box

      else:
        pass # wait until manual-send for other commands
    
    self.sendingCommand = False # reset manual sending
    self.master.after(CMD_RATE, lambda: self.autosendCommand(numTries=numTries, wantACK=wantACK, strIn=strIn, strOut=strOut))


######################################################################################


if version_info[0] == 2: from tkFont import Font
elif version_info[0] == 3: from tkinter.font import Font

class StatusButtons(tk.Frame):
  # sendCommand     triggered by request to manually send the command in the text box
  # autoSendCommand loop to control sending of commands, including automatic retries and continuous drive commands

  def __init__(self, master, closeWin, restartAll, saveImage, statusStr):
    tk.Frame.__init__(self, master, bd=5, relief='sunken') # explicitly initialize base class and create window
    self.master = master

    self.closeWin, self.restartAll, self.saveImage, self.statusStr \
      =  closeWin,      restartAll,      saveImage, statusStr

    # create buttons
    tk.Button(self, text="Quit (esc)", command=self.closeWin).pack(side="left", padx=5, pady=5)
    tk.Button(self, text="Restart (R)", command=self.restartAll).pack(side=tk.LEFT, padx = 5)
    monospaceFont = Font(family="Courier", weight='bold', size=12)
    tk.Label(self, textvariable=self.statusStr, font=monospaceFont).pack(side="left", padx=5, pady=5)
    tk.Button(self, text="Save Map", command=self.saveImage).pack(side=tk.LEFT, padx=5)

    # bind keyboard inputs to functions
    master.bind('<Escape>', lambda event: self.closeWin()) # escape exits program after prompt
    master.bind('<Shift-R>', lambda event: self.restartAll()) # shift and capital R does a soft reset