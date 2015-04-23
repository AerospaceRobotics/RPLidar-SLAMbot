#!/usr/bin/env python

# tools.py - helper tools for package
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


from math import sin, cos, degrees, radians
from matplotlib.lines import Line2D

import sys
PYTHON_SERIES = sys.version_info[0]

from os.path import isfile


# Python tools
if PYTHON_SERIES == 3: raw_input = lambda inStr: input(inStr)
else: raw_input = raw_input
def askForFile(filePath, mode):
  if not (mode == 'r' or mode == 'w'):
    sys.exit("Use open() directly to open files for reading and writing.")
  if isfile(filePath): # file already exists
    if 'r' in mode:
      pass # open it
    if 'w' in mode:
      if raw_input("File {0:s} already exists. Overwrite? [Y/n]: ".format(filePath)).lower() == 'n':
        sys.exit("Please select new log file location.")
      else:
        pass # rewrite it
  else: # file doesn't exist
    if 'r' in mode:
      print("IOError: Could not find file at {0:s}".format(filePath))
      sys.exit("Please check file name.")
    if 'w' in mode:
      print("Creating log file at {0:s}.".format(filePath))
      pass # create it
  try:
    return open(filePath, mode)
  except IOError:
    print("IOError: Could not open or create file at {0:s}".format(filePath))
    sys.exit("Please check permissions.")


# Drawing tools
ROBOT_WIDTH = 0.227 # [m]
ROBOT_HEIGHT = 0.237
left, right, bottom, top = -ROBOT_WIDTH/2, ROBOT_WIDTH/2, -ROBOT_HEIGHT/2, ROBOT_HEIGHT/2
ROBOT = [(0,top), (left,bottom), (right,bottom), (0,top)]
def drawMarker(ax, pos, destination=None): # input is mm
  x_abs, y_abs, th = pos[0]/1000.0, pos[1]/1000.0, radians(pos[2])
  s, c = sin(th), cos(th)
  robot = ROBOT # robot sprite
  robot = [(c*x+s*y+x_abs, -s*x+c*y+y_abs) for x, y in robot] # rotate and shift robot sprite to current position
  robot.append((destination[0]/1000.0, destination[1]/1000.0) if destination else (x_abs, y_abs)) # add absolute destination coordinates
  return ax.add_line(Line2D(*zip(*robot), color='red', alpha= 0.7, linewidth=1.0)) # those *args sure do look cool
def removeMarkers(markers):
  for i in range(len(markers)):
    markers[i].remove()
    del markers[i]


# Matrix tools
def shrinkTo(data, rows, cols): # from http://stackoverflow.com/a/10685869
  shrunk = data.reshape(rows, data.shape[0]/rows, cols, data.shape[1]/cols).sum(axis=1).sum(axis=2)
  return shrunk*255/shrunk.max()
class Feature(object):
  def __init__(self, mass, com, bounds, coords):
    self.com = com # (row, col) location of center of feature [pix]
    self.mass = mass # size of feature [square pixels]
    self.coords = coords # ([rows], [cols]) locations of all points of feature [pix]
    self.hgt = sum(bounds[0]) # [pix]
    self.wid = sum(bounds[1]) # [pix]
    self.size = self.hgt + self.wid # [pix]
    self.rLims = bounds[0] # min and max row indices in this feature
    self.cLims = bounds[1] # min and max column indices in this feature
  # def getOverlap(self, that): # makes doesOverlap a LOT faster # not done yet...

  #   return slice(self.rLims, , None), slice(, , None)
  def isAdjacentTo(self, that): # dumb check of two features for overlap or adjacency of domains
    return not (self.rLims[1]<that.rLims[0] or self.rLims[0]>that.rLims[1] or self.cLims[1]<that.cLims[0] or self.cLims[0]>that.cLims[1])
  def isSeparateFrom(self, that):
    return self.rLims[1]<=that.rLims[0] or self.rLims[0]>=that.rLims[1] or self.cLims[1]<=that.cLims[0] or self.cLims[0]>=that.cLims[1]
  def doesOverlap(self, that):
    if self.isSeparateFrom(that): return False
    overlap = getOverlap(self, that)
    for point1 in zip(*self.coords):
      for point2 in zip(*that.coords):
        if point1 == point2:
          return True
    return False


# Geometry tools
def rotatePt(point, angle): # performs clockwise rotation by given angle about origin
  x, y, theta = point
  s, c = sin(radians(angle)), cos(radians(angle))
  return (c*x+s*y, -s*x+c*y, theta+angle)
def translatePt(point, vector): # performs translation by given vector
  return (point[0]+vector[0], point[1]+vector[1], point[2])
def vecDiff(vec1, vec2):
  return tuple([el1 - el2 for el1, el2 in zip(vec1, vec2)])
def wrt(point_old, frame_old, frame_new): # return point with respect to new reference frame (all defined within same frame)
  diff = vecDiff(frame_new, frame_old)
  return rotatePt(translatePt(rotatePt(point_old, frame_old[2]), [-diff[0], -diff[1]]), -frame_new[2])


# Math tools
def float2int(x):
  return int(0.5 + x)
def coerceToRange(inNum, (lower, upper), wrapAround=False):
  if not wrapAround: # simple coerce
    return (upper if inNum > upper else (lower if inNum < lower else inNum))
  else:
    domain = upper - lower
    return inNum - domain*((inNum-lower-1)//domain)*(inNum>upper) + domain*((upper-inNum)//domain)*(inNum<lower)


# Binary tools
def bits2mask(bitList):
  return sum([2**el for el in bitList])


# String tools
def paddedStr(inStr, length):
  return '{0: <{width}s}'.format(inStr, width=length)[0:length] if length != 0 else inStr


# Pathfinding tools
class AStarMap(object):
  def __init__(self, mapMatrix):
    # mapMatrix is bool array, True where roads, False where obstacles
    self.walls = np.logical_not(mapMatrix)
    hgt, wid = mapMatrix.shape
    self.nodes = np.array([[AStarNode((row, col)) for row in range(hgt)] for col in range(wid)])
    self.neighbors = {} # maps each node to references to its neighbors (called a graph if you like math) # should be stored in node data object...
    for row, col in ((i,j) for i in range(hgt) for j in range(wid)):
      node = nodes[row, col]
      self.neighbors[node] = []
      counter = 8
      for r, c in ((row+i,col+j) for i in [-1,0,1] for j in [-1,0,1] if not (i==0 and j==0)): # all neighbors (excluding current)
        if not (0<=r<hgt or 0<=c<wid): continue # don't add out-of-bounds neighbors
        if self.walls[r, c]: continue # don't add neighbors in walls (fixed boundaries are defined here)
        counter -= 1
        self.neighbors[self.nodes[row, col]].append(self.nodes[r, c]) # add neighbor
      node.numWalls = counter

  def heuristic(self, current, start, end):
    # defined as path length to end (ignoring obstacles)
    dist_row = abs(current.row-end.row)
    dist_col = abs(current.col-end.col)
    return 101*abs(dist_row - dist_col) + 142*min(dist_row, dist_col) # favor proximity to end over proximity to start
      
  def search(self, startPoint, endPoint):
    start, end = self.nodes[startPoint], self.nodes[endPoint]
    openSet, closedSet = set(), set()
    current = start # start at beginning
    openSet.add(current) # begin with first node as only open one
    while openSet: # as long as there are open nodes
      current = min(openSet, key=lambda node: node.g+node.h) # proceed with best node
      if current == end: # done
        path = []
        while current.parent != self.nodes[current.parent].parent: # create path from end to start via parents of current
          path.append((current.row, current.col))
          current = self.nodes[current.parent]
        path.append(current) # add start (whose parent is itself)
        return path[::-1] # reverse order to give proper direction
      openSet.remove(current) # close current node
      closedSet.add(current)
      for neighbor in self.neighbors[current]: # proceed by acting on all neighbors of current node
        if neighbor in closedSet: # ignore node if closed
          continue
        if node in openSet: # determine if path from current is better than path from parent
          g = current.g + current.moveCost(neighbor)
          if neighbor.g > g: # moving to neighbor node from current node is better than from neighbor's parent
            neighbor.g = g # update with new path
            neighbor.parent = current.index
        else: # activate child node
          neighbor.g = current.g + current.moveCost(neighbor)
          neighbor.h = self.heuristic(neighbor, start, end)
          neighbor.parent = current.index
          openSet.add(node)
    return None # ran out of open nodes with no path available

class AStarNode(object):
  def __init__(self, (row, col)):
    self.g = 0 # cost to get here (includes distance and other stuff)
    self.h = 0 # heuristic cost (affects which open node to explore next)
    self.numWalls = 8 # number of walls that border this node
    self.index = (row, col) # location of self
    self.parent = (row, col) # location of parent

  def moveCost(self, that): # calculate g of moving from self to that
    corner = ((self.row - that.row) + (self.col - that.col)) % 2 == 0
    return (141 if corner else 100) + 70*that.numWalls # costs more to travel near walls