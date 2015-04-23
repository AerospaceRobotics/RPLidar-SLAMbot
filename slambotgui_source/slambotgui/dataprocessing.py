#!/usr/bin/env python

# maps.py - data objects to represent SLAM maps
# 
# Copyright (C) 2015 Michael Searing
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


# note that DataMatrix.saveImage() imports PIL and subprocess for map image saving and viewing
from tools import vecDiff, wrt, shrinkTo, radians, float2int, Feature
import time
import numpy as np # for array processing and matplotlib display
from scipy.ndimage.interpolation import rotate
from scipy.ndimage.measurements import label, find_objects
from scipy.misc import imresize


class DataMatrix(object):
  # init            creates data matrix and information vectors for processing
  # getMapMatrix    returns the map matrix
  # getInsetMatrix  returns the inset map matrix
  # get_robot_rel   returns the robot's position in mm relative to where is started
  # getRobotPos     populates robot position information needed by other methods of Data
  # drawBreezyMap   adds the BreezySLAM internal map to the map matrix
  # drawPointMap    adds scan data to map matrix
  # drawInset       adds scan data to inset matrix
  # drawRobot       adds robot position to data matrix, in the form of an arrow of red pixels
  # drawPath        draws portion of the robot's trajectory in the form of red dots on the desired object
  # saveImage       uses PIL to write an image file from the data matrix

  def __init__(self, MAP_SIZE_M=8.0, INSET_SIZE_M=2, MAP_RES_PIX_PER_M=100, MAP_DEPTH=5, INTERNAL_MAP=False, SMARTNESS_ON=False, **unused):
    self.INTERNAL_MAP = INTERNAL_MAP # should we use the map which BreezySLAM uses?
    self.SMARTNESS_ON = SMARTNESS_ON # should we use both maps and do smart things?
    self.USE_BREEZY_MAP = SMARTNESS_ON or INTERNAL_MAP
    self.USE_POINT_MAP = SMARTNESS_ON or not INTERNAL_MAP
    self.mapIncr = 255/MAP_DEPTH # height of each certainty layer
    self.mapMin = 255-self.mapIncr*MAP_DEPTH
    self.mapSize_m = MAP_SIZE_M
    self.mapSize_pix = int(MAP_SIZE_M*MAP_RES_PIX_PER_M) # number of pixels across the entire map
    self.insetSize_pix = int(INSET_SIZE_M*MAP_RES_PIX_PER_M) # number of pixels across the inset map
    self.mapCenter_pix = int(self.mapSize_pix/2) # indices of map center []
    self.mm2pix = MAP_RES_PIX_PER_M/1000.0 # [pix/mm]

    if self.USE_BREEZY_MAP: self.breezyMap = 255*np.ones((self.mapSize_pix, self.mapSize_pix), dtype=np.uint8)
    if self.USE_POINT_MAP: self.pointMap = 255*np.ones((self.mapSize_pix, self.mapSize_pix), dtype=np.uint8)
    self.insetMatrix = 255*np.ones((self.insetSize_pix, self.insetSize_pix), dtype=np.uint8)
    self.trajectory = [] # robot location history, in pixels
    self.robot_init = () # x [mm], y [mm], th [deg], defined from lower-left corner of map
    self.robot_abs = () # current robot location
    self.robot_rel = () # robot location relative to start position
    self.robot_pix = () # robot pixel location, relative to upper-left (0,0) of image matrix
    self.destination = None # absolute coordinates of current target [mm]
    self.displayMode = 0 # which map to display, set in RegionFrame
    self.features = [] # list of all unexplored frontiers
    self.minTargSize = 6 # minimum size of unexplored frontier to be considered

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

  def setDisplayMode(self, mode):
    self.displayMode = mode if self.SMARTNESS_ON else 0 # only allow default unless other maps are turned on before startup

  def getMapMatrix(self):
    if self.displayMode == 0: return self.breezyMap if self.INTERNAL_MAP else self.pointMap
    if self.displayMode == 1: return self.breezyMap
    if self.displayMode == 2: return self.pointMap
    fow_darkLimit = 80 # breezyMap value cutoff for points too low to be fog of war
    fow_brightLimit = 200 # breezyMap value cutoff for points too high to be fog of war
    wall_brightLimit = 240 # pointMap value cutoff for points too high to be walls
    road_darkLimit = 240 # breezyMap value cutoff for points too low to be road
    shrink_size = 200 # size of reduced-size map (used to speed up processing)
    # stime = time.clock()
    pointMapShrunk = shrinkTo(self.pointMap, shrink_size, shrink_size)
    breezyMapShrunk = shrinkTo(self.breezyMap, shrink_size, shrink_size)
    unexplored = np.logical_and(fow_darkLimit < breezyMapShrunk, breezyMapShrunk < fow_brightLimit)
    wall = pointMapShrunk < wall_brightLimit
    display = np.where(wall, 0, np.where(unexplored, 127, 255))
    if self.displayMode == 3: return display # filtered

    if self.displayMode == 6: return np.where(breezyMapShrunk > road_darkLimit, 255, 0) # roads

    gradientx, gradienty = np.zeros((shrink_size,shrink_size), dtype=bool), np.zeros((shrink_size,shrink_size), dtype=bool)
    n = 1 # number of times to take the diff (width of diff)
    gradientx[:,:-n], gradienty[:-n,:] = np.absolute(np.diff(display, n=n, axis=1))==128, np.absolute(np.diff(display, n=n, axis=0))==128
    targets = np.any([gradientx, np.roll(gradientx, n, axis=1), gradienty, np.roll(gradienty, n, axis=0)], axis=0)
    # targets = np.logical_or(gradientx, gradienty)
    if self.displayMode == 4: return np.where(targets, 127, np.where(wall, 0, 255)) # edges

    lbl, num_lbls = label(targets, structure=[[1,1,1],[1,1,1],[1,1,1]])
    self.addFeatures(lbl, num_lbls, shrink_size)
    # etime = time.clock()
    # print etime-stime
    if self.displayMode == 5: return lbl*255/(lbl.max() if lbl.max() != 0 else 1) # targets

  def addFeatures(self, lbl, num_lbls, shrink_size):
    slices = find_objects(lbl) # index regions of each object in targets matrix
    for i, coords in ((i, np.where(lbl == i+1)) for i in range(num_lbls)): # coordinates list for each feature
      mass = len(coords[0]) # number of points in each object
      if mass >= self.minTargSize: # only continue if object is of reasonable size
        bounds = [slices[i][j].indices(shrink_size)[:2] for j in (0,1)] # slice.indices takes maximum index as argument
        com = np.mean(coords, axis=1, dtype=int) # scipy's center_of_mass is too heavy (for literally no reason)
        # self.features.append(Feature(mass, com, bounds, coords))

  def getMapArray(self, size):
    # return bytearray(imresize(self.breezyMap if self.INTERNAL_MAP else self.pointMap, size, interp='nearest'))
    return bytearray(imresize(self.getMapMatrix(), size, interp='nearest'))

  def getInsetMatrix(self):
    return self.insetMatrix

  def getDestination(self):
    return self.destination if self.destination != None else self.robot_rel

  def getRelDestination(self):
    relDestination = wrt(self.destination, (0.0,0.0,0.0), self.robot_rel) if self.destination != None else (0.0,0.0,0.0)
    return relDestination

  def setRelDestination(self, relDestination):
    self.destination = wrt(relDestination, self.robot_rel, (0.0,0.0,0.0))

  def get_robot_rel(self):
    return self.robot_rel

  def get_robot_abs(self):
    return self.robot_abs

  def getRobotPos(self, curr_pos, init=False):
    self.robot_abs = curr_pos
    if init: self.robot_init = self.robot_abs
    self.robot_rel = vecDiff(self.robot_abs, self.robot_init) # displacement from start position (center of map)
    xpix = float2int(curr_pos[0]*self.mm2pix) # robot wrt map center (mO) + mO wrt matrix origin (xO)  = robot wrt xO [pix]
    ypix = self.mapSize_pix-float2int(curr_pos[1]*self.mm2pix) # y is negative because pixels increase downwards (+y_mm = -y_pix = -np rows)
    self.robot_pix = (xpix, ypix, self.robot_rel[2])
    self.trajectory.append(self.robot_pix)

  def drawBreezyMap(self, breezyMap):
    if self.USE_BREEZY_MAP:
      self.breezyMap = np.flipud(np.resize(np.array(breezyMap, dtype=np.uint8), (self.mapSize_pix,self.mapSize_pix)).T) # 7ms

  def drawPointMap(self, points):
    if self.USE_POINT_MAP:
      for (dist, ang) in (point for point in points if point[0] > 0): # only draw valid points
        # pixel location of scan point # point wrt robot + robot wrt x0 = point wrt x0
        x_pix = float2int( self.mapCenter_pix + ( self.robot_rel[0] + dist*np.sin(radians(ang+self.robot_rel[2])) )*self.mm2pix )
        y_pix = float2int( self.mapCenter_pix - ( self.robot_rel[1] + dist*np.cos(radians(ang+self.robot_rel[2])) )*self.mm2pix )
        try:
          self.pointMap[y_pix, x_pix] -= self.mapIncr*(self.pointMap[y_pix, x_pix] > self.mapMin) # decrement value at location if above minimum value
        except IndexError:
          pass # scan out of bounds

  def drawInset(self):
    source = self.breezyMap if self.INTERNAL_MAP else self.pointMap
    x, y = self.robot_pix[0:2] # indices of center of robot in main map
    raw = int(self.insetSize_pix*0.75) # half the size of the main map segment to capture
    rad = self.insetSize_pix/2 # half the size of the final segment

    mapChunk = source[y-raw:y+raw, x-raw:x+raw]
    s = slice(raw-rad, raw+rad, 1) # region of rotated chunk that we want
    self.insetMatrix = rotate(mapChunk, self.robot_pix[2], output=np.uint8, order=1, reshape=False)[s,s]

  def drawRobot(self, mapObject, pos, val):
    robotMat = rotate(self.robotSprite, -pos[2])
    hgt = (robotMat.shape[0]-1)/2 # indices of center of robot
    wid = (robotMat.shape[1]-1)/2
    x = slice(pos[0]-wid, pos[0]+wid+1, 1) # columns
    y = slice(pos[1]-hgt, pos[1]+hgt+1, 1) # rows
    mapObject[y,x][robotMat.astype(bool)] = val

  def drawPath(self, mapObject, inSlice, val):
    for x, y, theta in self.trajectory[inSlice]:
      mapObject[y,x] = val

  def saveImage(self):
    from PIL import Image # don't have PIL? sorry (try pypng)
    import os
    filename = time.strftime('%Y-%m-%dT%Hh%Mm%Ss', time.localtime()) + "_" + str(int(self.mapSize_m)) + "meters.png"

    robot = np.zeros((self.mapSize_pix, self.mapSize_pix), dtype=bool) # initialize robot matrix
    self.drawRobot(robot, self.robot_pix, 1)
    self.drawPath(robot, slice(None,-1,None), 1) # draw all values except last one # same as ":-1"

    GB = np.where(robot, 0, self.breezyMap if self.INTERNAL_MAP else self.pointMap).astype(np.uint8) # copy map, assign to color layers
    R = np.where(robot, 255, GB).astype(np.uint8) # add robot path to red layer

    im = Image.fromarray(np.dstack((R,GB,GB))) # create image from depth stack of three layers
    # filepath = os.path.join('examples',filename)
    filepath = os.path.join(filename)
    im.save(filepath) # save image
    print("Image saved to " + filepath)

    import subprocess # used to display the image (not necessary for save)
    subprocess.call(["eog", filepath]) # open with eye of gnome