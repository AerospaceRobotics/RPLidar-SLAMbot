#!/usr/bin/env python

# components.py - physical robot components
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


from numpy import pi as PI
from math import copysign

class TrackedRobot(object):
  def __init__(self, REV_2_TICK, WHEEL_DIAMETER, WHEEL_BASE, WHEEL_TRACK, TREAD_ERROR):

    # geometry # note that revolution (REV) is when the wheel turns 360 degrees
    REV_2_MM = PI*WHEEL_DIAMETER # circumference [mm]

    # dimensional analysis
    self.MM_2_TICK = REV_2_TICK/REV_2_MM # [ticks/mm]
    self.TICK_2_MM = REV_2_MM/REV_2_TICK # [mm/tick]

    # math
    ANGULAR_FLUX = TREAD_ERROR*((WHEEL_BASE/WHEEL_TRACK)**2+1); # tread slippage [] # our math assumed wheels, hence TREAD_ERROR

    # geometry # note that rotation (ROT) is when the robot turns 360 degrees
    ROT_2_MM = PI*WHEEL_TRACK # circumference of wheel turning circle [mm]
    ROT_2_DEG = 360.0 # [deg]

    # dimensional analysis
    REV_2_DEG = ROT_2_DEG*(REV_2_MM/ROT_2_MM) # degrees of rotation per wheel revolution [deg]
    self.DEG_2_TICK = REV_2_TICK/REV_2_DEG * ANGULAR_FLUX # [ticks/deg]
    self.TICK_2_DEG = REV_2_DEG/REV_2_TICK * 1.0/ANGULAR_FLUX # [deg/tick]

  def getVelocities(self, currEncPos, prevEncPos):
    dLeft, dRight, dt = [curr - prev for (curr,prev) in zip(currEncPos, prevEncPos)]

    # overflow correction:
    if dLeft > 2**15: dLeft -= 2**16-1 # signed short
    elif dLeft < -2**15: dLeft += 2**16-1

    if dRight > 2**15: dRight -= 2**16-1 # signed short
    elif dRight < -2**15: dRight += 2**16-1

    if dt < -2**15: dt += 2**16-1 # unsigned short # time always increases, so only check positive overflow

    dxy = self.TICK_2_MM * (dLeft + dRight)/2 # forward change in position
    dtheta = self.TICK_2_DEG * (dLeft - dRight)/2 # positive theta is clockwise

    if abs(dxy) > 50 or abs(dtheta) > 20: # encoder data is messed up
      return copysign(2, dxy), copysign(1, dtheta), 0.2
    return dxy, dtheta, dt/1000.0 # [mm], [deg], [s]



#####################################################################


class DaguRover5(TrackedRobot):
  def __init__(self):
    REV_2_TICK = 1000.0/3 # encoder ticks per wheel revolution
    WHEEL_DIAMETER = 58.2 # size of wheels in mm
    WHEEL_BASE = 177.0 # length of treads [mm]
    WHEEL_TRACK = 190.0 # separation of treads [mm]
    TREAD_ERROR = 0.90 # continuous tread slips this much relative to slip of wheels (experimental)
    TrackedRobot.__init__(self, \
                          REV_2_TICK, \
                          WHEEL_DIAMETER, \
                          WHEEL_BASE, \
                          WHEEL_TRACK, \
                          TREAD_ERROR)


#####################################################################


from breezyslam.components import Laser

class RPLIDAR(Laser):
  def __init__(self, DIST_MIN, DIST_MAX):
    self.DIST_MIN = DIST_MIN
    self.DIST_MAX = DIST_MAX

    self.SCAN_SIZE = 360 # number of points per scan
    POINTS_PER_SEC = 1980 # 1980points/sec * scan/360points [scans/sec]
    self.SCAN_RATE_HZ = float(POINTS_PER_SEC)/self.SCAN_SIZE # 1980points/sec * scan/360points [scans/sec]
    SCAN_DETECTION_ANGLE = 360
    SCAN_DISTANCE_NO_DETECTION_MM = self.DIST_MAX
    SCAN_DETECTION_MARGIN = 0
    LASER_OFFSET_MM = -35
    Laser.__init__(self, \
                   self.SCAN_SIZE, \
                   self.SCAN_RATE_HZ, \
                   SCAN_DETECTION_ANGLE, \
                   SCAN_DISTANCE_NO_DETECTION_MM, \
                   SCAN_DETECTION_MARGIN, \
                   LASER_OFFSET_MM)
