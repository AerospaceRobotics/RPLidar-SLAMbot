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
from mpl_toolkits.axes_grid1 import make_axes_locatable

VIEW_SIZE_M = 6.0 # default size of region to be shown in display [m]
DPI = 132.2 # calculated from $ xrandr
CMAP = plt.get_cmap('gray') # opposite of "binary"


def drawMarker(ax, pos, temporary=True):
  return ax.plot(pos[0]/1000, pos[1]/1000, markersize=8, color='red', marker=(3,1,-pos[2]), markeredgewidth=0, aa=False)

def removeMarkers(markers):
  for i in range(len(markers)):
    markers[i].remove()
    del markers[i]


class RegionFrame(tk.Frame): # tkinter frame, inheriting from the tkinter Frame class
  # init            draws all fields in the output frame of the main App
  # updateMaps      draws all map data onto the matplotlib figures
  # removeMarkers   deletes all temporary markers on the main map (old robot position)
  # drawMarker      draws robot on main map using matplotlib marker

  def __init__(self, master, mapMatrix, MAP_SIZE_M=8, **unused):
    tk.Frame.__init__(self, master) # explicitly initialize base class and create window
    self.markers = [] # current matplotlib markers

    # current (and only) figure
    self.fig = plt.figure(figsize=(5, 5), dpi=DPI, facecolor=self.master.cget('bg')) # create matplotlib figure

    # subplot 1 (stationary map)
    self.ax = plt.subplot(111) # add plot 1 to figure
    self.ax.set_title("Region Map") # name and label plot
    self.ax.set_xlabel("X Position [m]")
    self.ax.set_ylabel("Y Position [m]")
    self.myImg = self.ax.imshow(mapMatrix, interpolation="none", cmap=CMAP, vmin=0, vmax=255, # plot data
              extent=[-MAP_SIZE_M/2, MAP_SIZE_M/2, -MAP_SIZE_M/2, MAP_SIZE_M/2]) # extent sets labels by matching limits to edges of matrix
    self.ax.set_xlim(-VIEW_SIZE_M/2, VIEW_SIZE_M/2) # pre-zoom image to defined default MAP_SIZE_M
    self.ax.set_ylim(-VIEW_SIZE_M/2, VIEW_SIZE_M/2)

    # colorbar
    cax = make_axes_locatable(self.ax).append_axes("right", size="3%", pad=0.05)
    cbar = self.fig.colorbar(self.myImg, cax=cax, orientation='vertical') # create colorbar
    cbar.ax.tick_params(labelsize=10)

    # do fancy stuff with tkinter and matplotlib
    self.canvas = FigureCanvasTkAgg(self.fig, master=self) # master of the fig canvas is this frame
    self.canvas._tkcanvas.config(highlightthickness=0)
    self.canvas._tkcanvas.pack(fill='both', expand=True)
    NavigationToolbar2TkAgg(self.canvas, self)

  def updateMap(self, robotRel, mapMatrix):
    # send maps to image object
    self.myImg.set_data(mapMatrix) # 20ms

    # finishing touches
    removeMarkers(self.markers) # delete old robot position from map
    marker = drawMarker(self.ax, robotRel) # add new robot position to map # 0.2ms
    self.markers.extend(marker) # marker is a list of matplotlib.line.Line2D objects

    # refresh the figure
    self.canvas.draw() # 200ms


######################################################################################


from math import atan2, degrees

class InsetFrame(tk.Frame): # tkinter frame, inheriting from the tkinter Frame class
  # init            draws all fields in the output frame of the main App
  # updateMaps      draws all map data onto the matplotlib figures
  # removeMarkers   deletes all temporary markers on the main map (old robot position)
  # drawMarker      draws robot on main map using matplotlib marker

  def __init__(self, master, insetMatrix, sendCommand=None, INSET_SIZE_M=2, **unused):
    tk.Frame.__init__(self, master) # explicitly initialize base class and create window
    self.sendCommand = sendCommand

    # current (and only) figure
    self.fig = plt.figure(figsize=(3, 4.5), dpi=DPI, facecolor=self.master.cget('bg')) # create matplotlib figure

    # subplot 2 (relative map)
    self.ax = plt.subplot(111) # add plot 2 to figure
    self.ax.set_title("Robot Environs") # name and label plot
    self.ax.set_xlabel("", family='monospace')
    self.myImg = self.ax.imshow(insetMatrix, interpolation='none', cmap=CMAP, vmin=0, vmax=255, # plot data
              extent=[-INSET_SIZE_M/2, INSET_SIZE_M/2, -INSET_SIZE_M/2, INSET_SIZE_M/2])
    self.ax.tick_params(axis='both', which='major', labelsize=12)
    drawMarker(self.ax, (0,0,0), temporary=False) # draw permanent robot at center of inset map

    # do fancy stuff with tkinter and matplotlib
    self.canvas = FigureCanvasTkAgg(self.fig, master=self) # master of the fig canvas is this frame
    self.canvas._tkcanvas.config(highlightthickness=0)
    self.canvas._tkcanvas.pack(fill='both', expand=True)

    # make notification string to display mouse position
    self.notifyStr = tk.StringVar()
    self.notifyStr.set("\n") # pre-fill to final size
    tk.Label(self, textvariable=self.notifyStr).pack(side='bottom', fill='both')

    if sendCommand:
      self.fig.canvas.mpl_connect('button_press_event', self.onClick)
      self.fig.canvas.mpl_connect('motion_notify_event', self.onMovement)

  def onClick(self, event):
    if event.inaxes == self.ax: # click is on inset map
      x, y = 1000*event.xdata, 1000*event.ydata
      ang = degrees(atan2(x,y))
      dist = (x**2 + y**2)**0.5
      self.sendCommand('c{0:0.1f}c{1:0.0f}'.format(ang,dist))

  def onMovement(self, event):
    if event.inaxes == self.ax: # mouse is over inset map
      self.notifyStr.set('Mouse at: {0:0.0f}, {1:0.0f}\nClick to send robot here.'.format(1000*event.xdata,1000*event.ydata))

  def updateMap(self, robotRel, insetMatrix):
    # send maps to image object
    self.myImg.set_data(insetMatrix) # 0.4ms

    # finishing touches
    self.ax.set_xlabel('X = {0:6.1f}; Y = {1:6.1f};\nHeading = {2:6.1f}'.format(*robotRel))

    # refresh the figure
    self.canvas.draw() # 200ms


######################################################################################


if version_info[0] == 2: from tkFont import Font
elif version_info[0] == 3: from tkinter.font import Font

CMD_RATE = 100 # minimum time between auto-send commands [ms]
MAX_TX_TRIES = 3 # max number of times to try to resend command before giving up

class EntryFrame(tk.Frame):
  # sendCommand     send string to command entry box, identical to typing command and hitting Send/<Return>
  # manualSend      triggered by request to manually send the command in the text box
  # autoSendCommand loop to control sending of commands, including automatic retries and continuous drive commands

  def __init__(self, master, robot, closeWin, restartAll, saveImage, getACK, resetACK, TXQueue, statusStr, twoLines=False):
    tk.Frame.__init__(self, master) # explicitly initialize base class and create window
    self.master = master

    self.manualSending = False # are we trying to manually send a command?
    self.CMDS = {'v':{'prop':"speed 0..255",'func':lambda x: str(x)                       }, # send motor speed as given
                 'w':{'prop':"forward mm",  'func':lambda x: str(int(robot.MM_2_TICK*x))  }, # convert mm to ticks
                 'a':{'prop':"left deg",    'func':lambda x: str(int(robot.DEG_2_TICK*x)) }, # convert degrees to ticks
                 's':{'prop':"reverse mm",  'func':lambda x: str(int(robot.MM_2_TICK*x))  }, # convert mm to ticks
                 'd':{'prop':"right deg",   'func':lambda x: str(int(robot.DEG_2_TICK*x)) }} # convert degrees to ticks

    self.closeWin, self.restartAll, self.saveImage, self.getACK, self.resetACK, self.TXQueue, self.statusStr \
      =  closeWin,      restartAll,      saveImage,      getACK,      resetACK,      TXQueue, statusStr


    # create buttons
    monospaceFont = Font(family="Courier", weight='bold', size=12)
    tk.Label(self, textvariable=self.statusStr, font=monospaceFont).pack(side="bottom" if twoLines else "right", pady=5)

    tk.Button(self, text="Quit (esc)", command=self.closeWin).pack(side="left", pady=5) # tkinter interrupt function
    tk.Button(self, text="Restart (R)", command=self.restartAll).pack(side=tk.LEFT) # tkinter interrupt function
    self.entryBox = tk.Entry(master=self, width=15, font=monospaceFont)
    self.entryBox.insert(0, "Command")
    self.entryBox.pack(side="left")
    tk.Button(self, text="Send (enter)", command=self.manualSend).pack(side=tk.LEFT) # tkinter interrupt function
    tk.Button(self, text="Save Map", command=self.saveImage).pack(side=tk.LEFT) # tkinter interrupt function

    # bind keyboard inputs to functions
    master.bind('<Escape>', lambda event: self.closeWin()) # escape exits program after prompt
    master.bind('<Shift-R>', lambda event: self.restartAll()) # shift and capital R does a soft reset
    self.entryBox.bind('<Return>', lambda event: self.manualSend()) # enter sends the command in the command box

  def sendCommand(self, command):
    self.entryBox.delete(0,"end")
    self.entryBox.insert(0,command)
    self.manualSend()

  def manualSend(self):
    self.manualSending = True

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
      elif self.manualSending:
        command = strIn[0]
        if command in list(self.CMDS): # we're giving the robot a value for a command
          try:
            num = float(strIn[1:])
          except ValueError:
            self.entryBox.delete(1,"end")
            self.entryBox.insert(1,"[{}]".format(self.CMDS[command]['prop'])) # prompt user with proper command format
            self.entryBox.selection_range(1,'end')
          else:
            wantACK = True # start resending the command if it isn't received
            strOut = command + self.CMDS[command]['func'](num) + command # re-create command with converted values (and terminate)
            self.TXQueue.put(strOut)
        elif command == 'c': # we're giving the robot a compound command
          try:
            ang, dist = [float(val) for val in strIn[1:].split('c')]
          except ValueError:
            self.entryBox.delete(1,"end")
            self.entryBox.insert(1,"[theta]c[dist]") # prompt user with proper command format
            self.entryBox.selection_range(1,'end')
          else:
            wantACK = True
            strOut = command + self.CMDS['d']['func'](ang) + 'c' + self.CMDS['w']['func'](dist) + command
            self.TXQueue.put(strOut)
        else: # otherwise send only first character
          self.TXQueue.put(command)
          self.entryBox.delete(0,"end") # clear box

      else:
        pass # wait until manual-send for other commands
    
    self.manualSending = False # reset manual sending
    self.master.after(CMD_RATE, lambda: self.autosendCommand(numTries=numTries, wantACK=wantACK, strIn=strIn, strOut=strOut))


######################################################################################


if version_info[0] == 2: from tkFont import Font
elif version_info[0] == 3: from tkinter.font import Font

class StatusFrame(tk.Frame):
  def __init__(self, master, closeWin, restartAll, saveImage, statusStr, twoLines=False):
    tk.Frame.__init__(self, master, bd=5, relief='sunken') # explicitly initialize base class and create window
    self.master = master

    self.closeWin, self.restartAll, self.saveImage, self.statusStr \
      =  closeWin,      restartAll,      saveImage, statusStr

    # create buttons
    monospaceFont = Font(family="Courier", weight='bold', size=12)
    tk.Label(self, textvariable=self.statusStr, font=monospaceFont).pack(side="bottom" if twoLines else "right", pady=5)

    tk.Button(self, text="Quit (esc)", command=self.closeWin).pack(side="left", pady=5)
    tk.Button(self, text="Restart (R)", command=self.restartAll).pack(side=tk.LEFT)
    tk.Button(self, text="Save Map", command=self.saveImage).pack(side=tk.LEFT)

    # bind keyboard inputs to functions
    master.bind('<Escape>', lambda event: self.closeWin()) # escape exits program after prompt
    master.bind('<Shift-R>', lambda event: self.restartAll()) # shift and capital R does a soft reset