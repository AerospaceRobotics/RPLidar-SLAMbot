'''
BreezySLAM: Simple, efficient SLAM in Python

components.py: SLAM components (Laser, Map, Position, Scan, Map),
implemented as C extensions for efficiency. 

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
'''

# These classes are implemented as C extensions
from pybreezyslam import Scan, Map, Position

class Laser(object):
    '''
    A class representing the specifications of a scanning laser rangefinder (Lidar).
    '''
    def __init__(self, scan_size, scan_rate_hz, detection_angle_degrees, distance_no_detection_mm, detection_margin=0, offset_mm=0):
        
        self.scan_size = scan_size
        self.scan_rate_hz = scan_rate_hz
        self.detection_angle_degrees = detection_angle_degrees
        self.distance_no_detection_mm = distance_no_detection_mm
        self.distance_no_detection_mm = distance_no_detection_mm
        self.detection_margin = detection_margin
        self.offset_mm = offset_mm
        
    def __str__(self):
        
        return  'scan_size=%d | scan_rate=%3.3f hz | detection_angle=%3.3f deg | distance_no_detection=%7.4f mm | detection_margin=%d | offset=%4.4f m' % \
        (self.scan_size,  self.scan_rate_hz,  self.detection_angle_degrees, self.distance_no_detection_mm,  self.detection_margin, self.offset_mm)
        
    def __repr__(self):
        
        return str(self)


class URG04LX(Laser):
    '''
    A class for the Hokuyo URG-04LX
    '''
    def __init__(self, detectionMargin = 0, offsetMillimeters = 0):
        
        Laser.__init__(self, 682, 10, 240, 4000, detectionMargin, offsetMillimeters)


