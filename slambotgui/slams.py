
from breezyslam.algorithms import RMHC_SLAM
from breezyslam.components import Laser

USE_ODOMETRY = True

# Laser constants
SCAN_SIZE = 360 # number of points per scan
SCAN_RATE_HZ = 1980.0/360 # 1980points/sec * scan/360points [scans/sec]
SCAN_DETECTION_MARGIN = 0
LASER_OFFSET_MM = 35 # this value is negative what it should be # update() returns LIDAR unit position
MAP_QUALITY = 50
HOLE_WIDTH_MM = 200
RANDOM_SEED = 0xabcd


class Slam(RMHC_SLAM):
  # init          creates the BreezySLAM objects needed for mapping
  # getBreezyMap  returns BreezySLAM's current internal map
  # updateSlam    takes LIDAR data and uses BreezySLAM to calculate the robot's new position
  # getVelocities uses encoder data to return robot position deltas (forward, angular, and time)

  def __init__(self, dataFile=None, MAP_SIZE_PIXELS=800, MAP_SIZE_M=8, ANG_MAX=360, DIST_MAX=6000, TICK_2_MM=1, TICK_2_DEG=1, **unused):
    SCAN_DETECTION_ANGLE = ANG_MAX
    SCAN_DISTANCE_NO_DETECTION_MM = DIST_MAX
    laser = Laser(SCAN_SIZE, \
                  SCAN_RATE_HZ, \
                  SCAN_DETECTION_ANGLE, \
                  SCAN_DISTANCE_NO_DETECTION_MM, \
                  SCAN_DETECTION_MARGIN, \
                  LASER_OFFSET_MM)
    RMHC_SLAM.__init__(self, \
                       laser, \
                       MAP_SIZE_PIXELS, \
                       MAP_SIZE_M, \
                       MAP_QUALITY, \
                       HOLE_WIDTH_MM, \
                       RANDOM_SEED)

    self.tick2mm = TICK_2_MM
    self.tick2deg = TICK_2_DEG
    self.dataFile = dataFile

    self.prevEncPos = () # robot encoder data
    self.currEncPos = () # left wheel [ticks], right wheel [ticks], timestamp [ms]

    self.breezyMap = bytearray(MAP_SIZE_PIXELS**2) # initialize map array for BreezySLAM's internal mapping

  def getBreezyMap(self):
    return self.breezyMap

  def updateSlam(self, points): # 15ms
    distVec = [0 for i in range(SCAN_SIZE)]

    for point in points: # create breezySLAM-compatible data from raw scan data
      dist = point[0]
      index = int(point[1])
      if not 0 <= index < SCAN_SIZE: continue
      distVec[index] = int(dist)

    # note that breezySLAM switches the x- and y- axes (their x is forward, 0deg; y is right, +90deg)
    if self.dataFile: self.dataFile.write(' '.join((str(el) for el in list(self.currEncPos)+distVec)) + '\n')
    distVec = [distVec[i-180] for i in range(SCAN_SIZE)]
    self.update(distVec, self.getVelocities() if USE_ODOMETRY else None) # update slam information using particle filtering on LIDAR scan data // 10ms
    x, y, theta = self.getpos()

    self.getmap(self.breezyMap) # write internal map to breezyMap

    return (y, x, theta)

  def getVelocities(self):
    dLeft, dRight, dt = [curr - prev for (curr,prev) in zip(self.currEncPos, self.prevEncPos)]
    self.prevEncPos = self.currEncPos

    # overflow correction:
    if dLeft > 2**15: dLeft -= 2**16-1 # signed short
    elif dLeft < -2**15: dLeft += 2**16-1

    if dRight > 2**15: dRight -= 2**16-1 # signed short
    elif dRight < -2**15: dRight += 2**16-1

    if dt < -2**15: dt += 2**16-1 # unsigned short # time always increases, so only check positive overflow

    dxy = self.tick2mm * (dLeft + dRight)/2 # forward change in position
    dtheta = self.tick2deg * (dLeft - dRight)/2 # positive theta is clockwise

    if abs(dxy) > 50 or abs(dtheta) > 20: # encoder data is messed up
      return copysign(2, dxy), copysign(1, dtheta), 1/SCAN_RATE_HZ
    return dxy, dtheta, dt/1000.0 # [mm], [deg], [s]
