#!/usr/bin/env python

'''
log2pgm.py : BreezySLAM Python demo.  Reads logfile with odometry and scan data
             from Paris Mines Tech and produces a .PGM image file showing robot 
             trajectory and final map.
             
For details see

    @inproceedings{coreslam-2010,
      author    = {Bruno Steux and Oussama El Hamzaoui},
      title     = {CoreSLAM: a SLAM Algorithm in less than 200 lines of C code},
      booktitle = {11th International Conference on Control, Automation, 
                   Robotics and Vision, ICARCV 2010, Singapore, 7-10 
                   December 2010, Proceedings},
      pages     = {1975-1979},
      publisher = {IEEE},
      year      = {2010}
    }
                 
Copyright (C) 2014 Simon D. Levy

This code is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as 
published by the Free Software Foundation, either version 3 of the 
License, or (at your option) any later version.

This code is distributed in the hope that it will be useful,     
but WITHOUT ANY WARRANTY without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public License 
along with this code.  If not, see <http://www.gnu.org/licenses/>.
'''

# Log / PGM file root name
DATASET                         = "exp2"

# Flags
USE_ODOMETRY                    = False
USE_PARTICLE_FILTER             = True

# Maps size, scale
MAP_SIZE_PIXELS                 = 800
MAP_SCALE_PIXELS_PER_METER      = 30

# Arbitrary
RANDOM_SEED                     = 0xabcd
        
from breezyslam.algorithms import CoreSLAM
from breezyslam.robots import WheeledRobot, Laser

from math import pi
from sys import argv, exit, stdout
from math import pi, degrees

def main():
	    
	# Load the data from the file    
    lidars, odometries = load_data(DATASET)
    
    # Build a robot model if we want odometry
    robot = Rover() if USE_ODOMETRY else None
    
    # Create a CoreSLAM object with laser params and optional robot object
    slam = CoreSLAM(URG04(), MAP_SIZE_PIXELS, MAP_SCALE_PIXELS_PER_METER, \
                    random_seed=RANDOM_SEED)
    
    # If using odoemtry, turn off off particle filter by setting search variances to zero
    if not USE_PARTICLE_FILTER:
        slam.SIGMA_XY_METERS = 0
        slam.SIGMA_THETA_DEGREES = 0
        
    # Report what we're doing
    nscans = len(lidars)
    print('Processing %d scans with%s odometry / with%s particle filter...' % \
        (nscans, \
         '' if USE_ODOMETRY else 'out', '' if USE_PARTICLE_FILTER else 'out'))
    progbar = make_progress_bar(nscans)
    
    # Start with an empty trajectory of positions
    trajectory = []
    
    for scanno in range(nscans):
           
        # Convert odometry to velocities if indicated
        velocities = robot.computeVelocities(odometries[scanno]) \
                     if USE_ODOMETRY else None
                                                     
        # Update SLAM with lidar, velocities, obtaining new position      
        x, y, theta = slam.update(lidars[scanno], velocities)
                                
        # Add new position to trajectory
        trajectory.append((x, y))
        
        # Tame impatience
        show_progress_bar(progbar, scanno)
                
    # Create a byte array to receive the computed maps
    mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)
    
    # Get final map    
    slam.getmap(mapbytes)
    
    # Put trajectory into map as black pixels
    for coords in trajectory:
        
        x, y = coords
                        
        x = meters2pixels(x)
        y = meters2pixels(y)
                        
        mapbytes[coords2index(x, MAP_SIZE_PIXELS-y)] = 0
                    
    # Save map and trajectory as PGM file    
        
    filename = '%s.pgm' % DATASET
    print('\nSaving map to file %s' % filename)
        
    output = open(filename, 'wt')
    
    output.write('P2\n%d %d 255\n' % (MAP_SIZE_PIXELS, MAP_SIZE_PIXELS))
    
    progbar = make_progress_bar(MAP_SIZE_PIXELS)

    for y in range(MAP_SIZE_PIXELS):
        for x in range(MAP_SIZE_PIXELS):
            output.write('%d ' % mapbytes[coords2index(x, y)])
        output.write('\n')
        show_progress_bar(progbar,y)
            
    output.close()
    stdout.write('\n')
    
    
# Map coordinate helpers -------------------------------------------------------

def coords2index(x, y):
    
    return (MAP_SIZE_PIXELS - 1 - y) * MAP_SIZE_PIXELS + x
   
  
def meters2pixels(c):
    
    return int(c * MAP_SCALE_PIXELS_PER_METER)

    
# Method to load all from file ------------------------------------------------
# Each line in the file has the format:
#
#  TIMESTAMP  ... Q1  Q1 ... Distances
#  (usec)                    (mm)
#  0          ... 2   3  ... 24 ... 
#  
#where Q1, Q2 are odometry values

def load_data(dataset):
    
    filename = '%s.dat' % dataset
    print('Loading data from %s...' % filename)
    
    fd = open(filename, 'rt')
    
    scans = []
    odometries = []
    
    while True:  
        
        s = fd.readline()
        
        if len(s) == 0:
            break       
            
        toks = s.split()[0:-1] # ignore ''
                
        odometry = (long(toks[0]), int(toks[2]), int(toks[3]))
                
        lidar = [int(tok) for tok in toks[24:]]
                
        scans.append(lidar)
        odometries.append(odometry)
        
    fd.close()
    
    return scans, odometries

# Class for Hokuyo URG04 LIDAR -------------------------------------------------

class URG04(Laser):
    
    def __init__(self):
        
        Laser.__init__(self, 682, 10, -120, +120, 4.0, 70, .145)
        
# Class for MinesRover custom robot ------------------------------------------

class Rover(WheeledRobot):
    
    def __init__(self):
        
        WheeledRobot.__init__(self, 0.077, 0.165)
        
        self.ticks_per_cycle = 2000
                        
    def __str__(self):
        
        return '<%s ticks_per_cycle=%d>' % (WheeledRobot.__str__(self), self.ticks_per_cycle)
        
    def computeVelocities(self, odometry):
        
        return WheeledRobot.computeVelocities(self, odometry[0], odometry[1], odometry[2])

    def extractOdometry(self, timestamp, leftWheel, rightWheel):
                
        # Convert microseconds to seconds, ticks to angles        
        return timestamp / 1e6, \
               self._ticks_to_degrees(leftWheel), \
               self._ticks_to_degrees(rightWheel)
               
    def odometryStr(self, odometry):
        
        return '<timestamp=%d usec leftWheelTicks=%d rightWheelTicks=%d>' % \
               (odometry[0], odometry[1], odometry[2])
               
    def _ticks_to_degrees(self, ticks):
        
        return ticks * (180. / self.ticks_per_cycle)
        
               
# Progress-bar class
# Downloaded from http://code.activestate.com/recipes/168639-progress-bar-class/
# 12 January 2014

class progressBar:
	
	def __init__(self, minValue = 0, maxValue = 10, totalWidth=12):
		self.progBar = "[]"   # This holds the progress bar string
		self.min = minValue
		self.max = maxValue
		self.span = maxValue - minValue
		self.width = totalWidth
		self.amount = 0       # When amount == max, we are 100% done 
		self.updateAmount(0)  # Build progress bar string

	def updateAmount(self, newAmount = 0):
		
		if newAmount < self.min: newAmount = self.min
		if newAmount > self.max: newAmount = self.max
		self.amount = newAmount

		# Figure out the new percent done, round to an integer
		diffFromMin = float(self.amount - self.min)
		percentDone = (diffFromMin / float(self.span)) * 100.0
		percentDone = round(percentDone)
		percentDone = int(percentDone)

		# Figure out how many hash bars the percentage should be
		allFull = self.width - 2
		numHashes = (percentDone / 100.0) * allFull
		numHashes = int(round(numHashes))

		# build a progress bar with hashes and spaces
		self.progBar = \
                    "[" + '#'*numHashes + ' '*(allFull-numHashes) + "]"

		# figure out where to put the percentage, roughly centered
		percentPlace = \
                    int((len(self.progBar) / 2) - len(str(percentDone))) 
		percentString = str(percentDone) + "%"
		
		# slice the percentage into the bar
		self.progBar = self.progBar[0:percentPlace] + percentString + \
                       self.progBar[percentPlace+len(percentString):]

	def __str__(self):
		
		return str(self.progBar)
		
# Progress-bar helpers ---------------------------------------------------------        

def make_progress_bar(maxval):
    return progressBar(0, maxval, 80)

def show_progress_bar(progbar, amt):
    progbar.updateAmount(amt)
    stdout.write('\r%s' % str(progbar))
    stdout.flush()
            
    
                    
main()
