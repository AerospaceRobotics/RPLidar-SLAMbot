'''
BreezySLAM: Simple, efficient SLAM in Python

robots.py: odometry models for different kinds of robots

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
along with this code.  If not, see <http:#www.gnu.org/licenses/>.
You should also have received a copy of the Parrot Parrot AR.Drone 
Development License and Parrot AR.Drone copyright notice and disclaimer 
and If not, see 
<https:#projects.ardrone.org/attafchments/277/ParrotLicense.txt> 
and
<https:#projects.ardrone.org/attachments/278/ParrotCopyrightAndDisclaimer.txt>.

Change Log:

01-MAR-2014: Simon D. Levy - Initial release
19-MAR-2014: SDL           - Added Laser class
31-MAR-2014: SDL           - Robot.computeVelocities returns dtheta in degrees
'''

import math

class WheeledRobot(object):
    '''
    An abstract class supporting ododmetry for wheeled robots.  Your implementing
    class should provide the method:
    
      extractOdometry(self, timestamp, leftWheel, rightWheel) --> 
        (timestampSeconds, leftWheelDegrees, rightWheelDegrees)      
    '''
    
    def __init__(self, wheelRadiusMeters, halfAxleLengthMeters):
        '''
        wheelRadiusMeters    radius of each odometry wheel, in meters        
        halfAxleLengthMeters half the length of the axle between the odometry wheels, in meters  
        '''
        self.wheelRadiusMeters = wheelRadiusMeters     
        self.halfAxleLengthMeters = halfAxleLengthMeters
        
        self.timestampSecondsPrev = None        
        self.leftWheelDegreesPrev = None
        self.rightWheelDegreesPrev = None
                
    def __str__(self):
        
        return '<Wheel radius=%f m Half axle Length=%f m>' % \
        (self.wheelRadiusMeters, self.halfAxleLengthMeters)
        
    def __repr__(self):
        
        return self.__str__()
        
    def computeVelocities(self, timestamp, leftWheelOdometry, rightWheelOdometry):
        '''
        Computes forward and angular velocities based on odometry.
        
        Parameters:
        
          timestamp          time stamp, in whatever units your robot uses       
          leftWheelOdometry  odometry for left wheel, in whatever units your robot uses       
          rightWheelOdometry odometry for right wheel, in whatever units your robot uses
        
        Returns a tuple (dxyMeters, dthetaDegrees, dtSeconds)
        
          dxyMeters     forward distance traveled, in meters
          dthetaDegrees change in angular position, in degrees
          dtSeconds     elapsed time since previous odometry, in seconds
        '''                      
        dxyMeters = 0
        dthetaDegrees = 0
        dtSeconds = 0
                       
        timestampSecondsCurr, leftWheelDegreesCurr, rightWheelDegreesCurr = \
            self.extractOdometry(timestamp, leftWheelOdometry, rightWheelOdometry)
            
        if self.timestampSecondsPrev != None:  
            
            leftDiffDegrees = leftWheelDegreesCurr - self.leftWheelDegreesPrev
            rightDiffDegrees = rightWheelDegreesCurr - self.rightWheelDegreesPrev
            
            dxyMeters =  self.wheelRadiusMeters * \
                    (math.radians(leftDiffDegrees) + math.radians(rightDiffDegrees))
               
            dthetaDegrees = self.wheelRadiusMeters / self.halfAxleLengthMeters * \
                    (rightDiffDegrees - leftDiffDegrees)
                
            dtSeconds = timestampSecondsCurr - self.timestampSecondsPrev
                
        # Store current odometry for next time
        self.timestampSecondsPrev = timestampSecondsCurr        
        self.leftWheelDegreesPrev = leftWheelDegreesCurr
        self.rightWheelDegreesPrev = rightWheelDegreesCurr

        # Return linear velocity, angular velocity, time difference
        return dxyMeters, dthetaDegrees, dtSeconds 
        
        
class Laser(object):
    
    def __init__(self, \
        scanSize,\
        scanRateHz,\
        angleMinDegrees,\
        angleMaxDegrees,\
        distanceNoDetectionMeters,\
        detectionMargin,\
        offsetMeters):
        '''
        scanSize                   number of rays per scan
        scanRateHz                 laser scan rate in Hertz
        angleMinDegrees            minimum laser angle in degrees
        angleMaxDegrees            maximum laser angle in degrees
        distanceNoDetectionMeters  scan distances above this are treated as infinfity
        detectionMargin            number of rays at edges of scan to ignore
        offsetMeters               forward/backward offset of laser motor from robot center
        '''
        
        self.scan_size = scanSize
        self.scan_rate_hz = scanRateHz
        self.angle_min_degrees = angleMinDegrees
        self.angle_max_degrees = angleMaxDegrees
        self.distance_no_detection_meters = distanceNoDetectionMeters
        self.detection_margin = detectionMargin
        self.offset_meters = offsetMeters
        
    def __str__(self):
        
        return  '<offset=%3.0f m scan_size=%d scan_rate = %d hz' % \
        (self.offset_meters, self.scan_size, self.scan_rate_hz) + \
        'angle_min=%d deg angle_max=%d deg' % \
        (self.angle_min_deg, self.angle_max_deg) + \
        'detection_margin=%d distance_no_detection_meters=%4.4f m>' % \
        (self.detection_margin, self.distance_no_detection_meters)
        
    def __repr__(self):
        
        return self.__str__()        
            

