#!/usr/bin/env python

# maps.py - data objects to represent SLAM maps
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


# note that DataMatrix.saveImage() imports PIL and subprocess for map image saving and viewing
import time
import numpy as np # for array processing and matplotlib display
from scipy.ndimage.interpolation import rotate
from scipy.misc import imresize
float2int = lambda x: int(0.5 + x)

DEG_2_RAD = np.pi/180.0

class DataMatrix:
  # init            creates data matrix and information vectors for processing
  # getMapMatrix    returns the map matrix
  # getInsetMatrix  returns the inset map matrix
  # get_robot_rel   returns the robot's position in mm relative to where is started
  # getRobotPos     populates robot position information needed by other methods of Data
  # drawBreezyMap   adds the BreezySLAM internal map to the map matrix
  # drawMap         adds scan data to map matrix
  # drawInset       adds scan data to inset matrix
  # drawRobot       adds robot position to data matrix, in the form of an arrow of red pixels
  # drawPath        draws portion of the robot's trajectory in the form of red dots on the desired object
  # saveImage       uses PIL to write an image file from the data matrix

  def __init__(self, MAP_SIZE_M=8.0, INSET_SIZE_M=2, MAP_RES_PIX_PER_M=100, MAP_DEPTH=5, **unused):
    self.mapIncr = 255/MAP_DEPTH # height of each certainty layer
    self.mapMin = 255-self.mapIncr*MAP_DEPTH
    self.mapSize_m = MAP_SIZE_M
    self.mapSize_pix = int(MAP_SIZE_M*MAP_RES_PIX_PER_M) # number of pixels across the entire map
    self.insetSize_pix = int(INSET_SIZE_M*MAP_RES_PIX_PER_M) # number of pixels across the inset map
    self.mapCenter_pix = int(self.mapSize_pix/2) # indices of map center []
    self.mm2pix = MAP_RES_PIX_PER_M/1000.0 # [pix/mm]

    self.mapMatrix = np.empty((self.mapSize_pix, self.mapSize_pix), dtype=np.uint8) # initialize data matrix
    self.mapMatrix.fill(255)
    self.insetMatrix = np.empty((self.insetSize_pix, self.insetSize_pix), dtype=np.uint8) # initialize inset matrix
    self.trajectory = [] # robot location history, in pixels
    self.robot_init = () # x [mm], y [mm], th [deg], defined from lower-left corner of map
    self.robot_abs = () # current robot location
    self.robot_rel = () # robot location relative to start position
    self.robot_pix = () # robot pixel location, relative to upper-left (0,0) of image matrix

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

  def getMapMatrix(self):
    return self.mapMatrix

  def getMapArray(self, size):
    return bytearray(imresize(self.mapMatrix, size, interp='nearest'))

  def getInsetMatrix(self):
    return self.insetMatrix

  def get_robot_rel(self):
    return self.robot_rel

  def get_robot_abs(self):
    return self.robot_abs

  def getRobotPos(self, curr_pos, init=False):
    self.robot_abs = curr_pos
    if init: self.robot_init = self.robot_abs
    self.robot_rel = tuple([curr-init for (curr,init) in zip(self.robot_abs, self.robot_init)]) # displacement from start position (center of map)
    xpix = float2int(curr_pos[0]*self.mm2pix) # robot wrt map center (mO) + mO wrt matrix origin (xO)  = robot wrt xO [pix]
    ypix = self.mapSize_pix-float2int(curr_pos[1]*self.mm2pix) # y is negative because pixels increase downwards (+y_mm = -y_pix = -np rows)
    self.robot_pix = (xpix, ypix, self.robot_rel[2])
    self.trajectory.append(self.robot_pix)

  def drawBreezyMap(self, breezyMap):
    self.mapMatrix = np.flipud(np.resize(np.array(breezyMap, dtype=np.uint8), (self.mapSize_pix,self.mapSize_pix)).T) # 7ms

  def drawMap(self, points):
    for (dist, ang) in [point for point in points if point[0] > 0]: # only draw valid points
      # pixel location of scan point # point wrt robot + robot wrt x0 = point wrt x0
      x_pix = float2int( self.mapCenter_pix + ( self.robot_rel[0] + dist*np.sin((ang+self.robot_rel[2])*DEG_2_RAD) )*self.mm2pix )
      y_pix = float2int( self.mapCenter_pix - ( self.robot_rel[1] + dist*np.cos((ang+self.robot_rel[2])*DEG_2_RAD) )*self.mm2pix )
      try:
        self.mapMatrix[y_pix, x_pix] -= self.mapIncr*(self.mapMatrix[y_pix, x_pix] > self.mapMin) # decrement value at location if above minimum value
      except IndexError:
        pass # scan out of bounds

  def drawInset(self):
    x, y = self.robot_pix[0:2] # indices of center of robot in main map
    raw = int(self.insetSize_pix*0.75) # half the size of the main map segment to capture
    rad = self.insetSize_pix/2 # half the size of the final segment

    mapChunk = self.mapMatrix[y-raw:y+raw, x-raw:x+raw]
    s = slice(raw-rad, raw+rad, 1) # region of rotated chunk that we want
    self.insetMatrix = rotate(mapChunk, self.robot_rel[2], output=np.uint8, order=1, reshape=False)[s,s]

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

    robot = np.zeros((self.mapSize_pix, self.mapSize_pix), dtype=bool) # initialize data matrix
    self.drawRobot(robot, self.robot_pix, 1)
    self.drawPath(robot, slice(None,-1,None), 1) # draw all values except last one # same as ":-1"

    GB = np.where(robot, 0, self.mapMatrix).astype(np.uint8) # copy map, invert values, assign to color layers
    R = np.where(robot, 255, GB).astype(np.uint8) # add robot path to red layer

    im = Image.fromarray(np.dstack((R,GB,GB))) # create image from depth stack of three layers
    filepath = os.path.join('examples',filename)
    im.save(filepath) # save image
    print("Image saved to " + filepath)

    import subprocess # used to display the image (not necessary for save)
    subprocess.call(["eog", filepath]) # open with eye of gnome