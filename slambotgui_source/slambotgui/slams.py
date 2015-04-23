#!/usr/bin/env python

# slams.py - SLAM algorithms
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


from tools import coerceToRange
from breezyslam.algorithms import RMHC_SLAM

HOLE_WIDTH_MM = 200
RANDOM_SEED = 0xabcd

class Slam(RMHC_SLAM):
  # init          creates the BreezySLAM objects needed for mapping
  # getBreezyMap  returns BreezySLAM's current internal map
  # updateSlam    takes LIDAR data and uses BreezySLAM to calculate the robot's new position
  # getVelocities uses encoder data to return robot position deltas, is only run if USE_ODOMETRY

  def __init__(self, robot, laser, logFile=None, MAP_SIZE_M=8.0, MAP_RES_PIX_PER_M=100, USE_ODOMETRY=True, MAP_QUALITY=5, **unused):
    self.USE_ODOMETRY = USE_ODOMETRY
    MAP_SIZE_PIXELS = int(MAP_SIZE_M*MAP_RES_PIX_PER_M) # number of pixels across the entire map
    RMHC_SLAM.__init__(self, \
                       laser, \
                       MAP_SIZE_PIXELS, \
                       MAP_SIZE_M, \
                       MAP_QUALITY, \
                       HOLE_WIDTH_MM, \
                       RANDOM_SEED)

    self.robot = robot
    self.scanSize = laser.SCAN_SIZE
    self.logFile = logFile

    self.prevEncPos = () # robot encoder data
    self.currEncPos = () # left wheel [ticks], right wheel [ticks], timestamp [ms]

    self.breezyMap = bytearray(MAP_SIZE_PIXELS**2) # initialize map array for BreezySLAM's internal mapping

  def getBreezyMap(self):
    return self.breezyMap

  def updateSlam(self, points): # 15ms
    distVec = [0 for i in range(self.scanSize)]

    for point in points: # create breezySLAM-compatible data from raw scan data
      dist = point[0]
      index = int(point[1])
      if not 0 <= index < self.scanSize: continue
      distVec[index] = int(dist)

    # note that breezySLAM switches the x- and y- axes (their x is forward, 0deg; y is right, +90deg)
    if self.logFile: self.logFile.write(' '.join((str(el) for el in list(self.currEncPos)+distVec)) + '\n')
    distVec = [distVec[i-180] for i in range(self.scanSize)] # rotate scan data so middle of vector is straight ahead, 0deg
    self.update(distVec, self.getVelocities() if self.USE_ODOMETRY else None) # 10ms
    x, y, theta = self.getpos()

    self.getmap(self.breezyMap) # write internal map to breezyMap

    return (y, x, coerceToRange(theta, (-180.0,180.0), wrapAround=True))

  def getVelocities(self):
    velocities = self.robot.getVelocities(self.currEncPos, self.prevEncPos)
    self.prevEncPos = self.currEncPos
    return velocities
