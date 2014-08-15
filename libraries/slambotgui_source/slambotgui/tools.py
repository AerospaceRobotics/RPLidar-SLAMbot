#!/usr/bin/env python

# tools.py - helper tools for package
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


from math import sin, cos, degrees, radians
from matplotlib.lines import Line2D

from sys import version_info
PYTHON_SERIES = version_info[0]

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
  def __init__(self, com, mass, coords, bounds):
    self.com = com # (row, col) location of center of feature [pix]
    self.mass = mass # size of feature [square pixels]
    self.coords = coords # ([rows], [cols]) locations of all points of feature [pix]
    self.hgt = sum(bounds[0]) # [pix]
    self.wid = sum(bounds[1]) # [pix]
    self.size = self.hgt + self.wid # [pix]
    self.rLims = bounds[0] # min and max row indices in this feature
    self.cLims = bounds[1] # min and max column indices in this feature
  def overlapsFast(self, feature): # dumb, but fast
    return abs(self.com[0]-feature.com[0]) + abs(self.com[1]-feature.com[1]) < (self.size + feature.size)/3
  def overlaps(self, feature): # dumb, but finds adjacent
    return not (self.rLims[1]<feature.rLims[0] or self.rLims[0]>feature.rLims[1] or self.cLims[1]<feature.cLims[0] or self.cLims[0]>feature.cLims[1])
  def overlapsSlow(self, feature): # dumb, but thorough
    for point1 in zip(*self.coords):
      for point2 in zip(*feature.coords):
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
def wrt(point_old, frame_old, frame_new): # return point with respect to new reference frame
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
    self.roads = np.where(mapMatrix, 100, 0) # prefer to stay away from walls # need to implement in better way
    self.walls = np.logical_not(mapMatrix)
    hgt, wid = mapMatrix.shape
    self.nodes = [[AStarNode(row, col) for row in range(hgt)] for col in range(wid)]
    self.graph = {} # maps each node to references to its neighbors
    for row, col in ((i,j) for i in range(hgt) for j in range(wid)):
      node = nodes[row][col]
      self.graph[node] = []
      for i, j in ((i,j) for i in [-1,0,1] for j in [-1,0,1] if not (i==0 and j==0)): # all neighbors
        if not (0<=row+i<hgt or 0<=col+j<wid): continue # don't add out-of-bounds neighbors
        if self.walls[i][j]: continue # don't add neighbors in walls
        self.graph[self.nodes[row][col]].append(self.nodes[row+i][col+j]) # add neighbor
      
  def heuristic(self, current, start, end):
    return abs(current.x-end.x) + abs(current.y-end.y) + self.roads[node]
      
  def search(self, startPoint, endPoint):
    start = self.nodes[startPoint]
    end = self.nodes[endPoint]
    openSet = set()
    closedSet = set()
    current = start # start at beginning
    openSet.add(current) # begin with first node as only open one
    while openSet: # as long as there are open nodes
      current = min(openSet, key=lambda node: node.g+node.h) # proceed with best node
      if current == end: # done
        path = []
        while current.parent: # create path from chain of parents
          path.append(current)
          current = current.parent
        path.append(current)
        return path[::-1]
      openSet.remove(current) # close current node
      closedSet.add(current)
      for node in self.graph[current]: # proceed by acting on all neighbors of current
        if node in closedSet: # ignore node if closed
          continue
        if node in openSet: # compare weights of pathing through this neighbor
          new_g = current.g + current.move_cost(node)
          if node.g > new_g:
            node.g = new_g
            node.parent = current
        else: # activate child node
          node.g = current.g + current.move_cost(node)
          node.h = self.heuristic(node, start, end)
          node.parent = current
          openSet.add(node)
    return None

class AStarNode(object):
  def __init__(self, (row, col)):
    self.g = 0
    self.h = 0
    self.row, self.col = row, col
    self.parent = None

  def moveCost(self, that):
    corner = abs(self.row - that.row) == 1 and abs(self.col - that.col) == 1
    return 14 if corner else 10