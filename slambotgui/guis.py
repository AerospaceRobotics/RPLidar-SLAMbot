
from sys import version_info

if version_info[0] == 2: from Tkinter import Frame as tkFrame
elif version_info[0] == 3: from tkinter import Frame as tkFrame

import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2TkAgg
from matplotlib.gridspec import GridSpec

class MatplotlibMaps(tkFrame): # tkinter frame, inheriting from the tkinter Frame class
  # init            draws all fields in the output frame of the main App
  # updateMaps      draws all map data onto the matplotlib figures
  # removeMarkers   deletes all temporary markers on the main map (old robot position)
  # drawMarker      draws robot on main map using matplotlib marker

  def __init__(self, master, mapMatrix, insetMatrix, MAP_SIZE_M=8, VIEW_SIZE_M=4, INSET_SIZE_M=2, MAP_DEPTH=5, **unused):
    tkFrame.__init__(self, master, bd=5, relief='sunken') # explicitly initialize base class and create window
    self.markers = [] # current matplotlib markers

    # current (and only) figure
    self.fig = plt.figure(figsize=(9, 5), dpi=131.2, facecolor=self.master.cget('bg')) # create matplotlib figure (dpi calculated from $ xrandr)
    gs = GridSpec(1,3) # layout of plots in figure

    # plot color settings
    cmap = plt.get_cmap('binary') # opposite of "gray"
    cmap.set_over('red') # robot map value is set to higher than maximum map value

    # subplot 1 (stationary map)
    self.ax1 = plt.subplot(gs[0,:2]) # add plot 1 to figure
    self.ax1.set_title("Region Map") # name and label plot
    self.ax1.set_xlabel("X Position [mm]")
    self.ax1.set_ylabel("Y Position [mm]")
    self.myImg1 = self.ax1.imshow(mapMatrix, interpolation="none", cmap=cmap, vmin=0, vmax=MAP_DEPTH, # plot data
              extent=[-MAP_SIZE_M/2, MAP_SIZE_M/2, -MAP_SIZE_M/2, MAP_SIZE_M/2]) # extent sets labels by matching limits to edges of matrix
    self.ax1.set_xlim(-VIEW_SIZE_M/2, VIEW_SIZE_M/2) # pre-zoom image to defined default MAP_SIZE_M
    self.ax1.set_ylim(-VIEW_SIZE_M/2, VIEW_SIZE_M/2)

    # colorbar
    self.cbar = self.fig.colorbar(self.myImg1, orientation='vertical') # create colorbar

    # subplot 2 (relative map)
    self.ax2 = plt.subplot(gs[0,2]) # add plot 2 to figure
    self.ax2.set_title("Robot Environs") # name and label plot
    self.ax2.set_xlabel("", family='monospace')
    self.myImg2 = self.ax2.imshow(insetMatrix, interpolation='none', cmap=cmap, vmin=0, vmax=MAP_DEPTH, # plot data
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