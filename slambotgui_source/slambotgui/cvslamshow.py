'''
cvslamshow.py - OpenCV classes for displaying maps and robots in SLAM projects

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
# Modified by Michael Searing under GPL


# Robot display params
ROBOT_COLOR_BGR                 = (0, 0, 255)
ROBOT_HEIGHT                    = 237
ROBOT_WIDTH                     = 227

# Scan point display params
SCANPOINT_RADIUS                = 1
SCANPOINT_COLOR_BGR             = (0, 255, 0)

# Display params for odometry-based velocity
SENSOR_V_MAX_MM                 = 1000
SENSOR_THETA_MAX_DEG            = 20
SENSOR_BAR_X                    = 150
SENSOR_BAR_Y_OFFSET             = 3
SENSOR_BAR_WIDTH                = 20
SENSOR_BAR_MAX_HEIGHT           = 200
SENSOR_TEXT_X                   = 20
SENSOR_V_Y                      = 30
SENSOR_THETA_Y                  = 80
SENSOR_LABEL_COLOR_BGR          = (255,0,0)
SENSOR_POSITIVE_COLOR_BGR       = (0,255,0)
SENSOR_NEGATIVE_COLOR_BGR       = (0,0,255)

# Trajectory display params
TRAJECTORY_COLOR_BGR            = (255, 0, 0)

import cv

# Arbitrary font for OpenCV
FONT_FACE                       = cv.CV_FONT_HERSHEY_COMPLEX

from math import sin, cos, radians

class SlamShow(object):

    def __init__(self, map_size_pixels, map_scale_pixels_per_mm, window_name):
    
        # Store constants for update
        self.map_size_pixels = map_size_pixels
        self.map_scale_pixels_per_mm = map_scale_pixels_per_mm
        self.window_name = window_name

        # Create a byte array to display the map with a color overlay
        self.bgrbytes = bytearray(map_size_pixels * map_size_pixels * 3)
        
        # Create an empty OpenCV image to be filled with map bytes
        self.image = cv.CreateImageHeader((map_size_pixels,map_size_pixels), cv.IPL_DEPTH_8U, 3)
    
        # Create an OpenCV window for displaying the map
        cv.NamedWindow(window_name, cv.CV_WINDOW_AUTOSIZE)
        
        # Set up font for displaying velocities
        self.font = cv.InitFont(FONT_FACE, 1, 1)
    
        # Display initial empty image
        cv.SetData(self.image, self.bgrbytes, self.map_size_pixels*3)
        cv.ShowImage(self.window_name, self.image)


    def displayMap(self, mapbytes):
        
        # Interleave the grayscale map bytes into the color bytes
        self.bgrbytes[0::3] = mapbytes
        self.bgrbytes[1::3] = mapbytes
        self.bgrbytes[2::3] = mapbytes
        
        # Put color bytes into image
        cv.SetData(self.image, self.bgrbytes, self.map_size_pixels*3)
 
 
    def displayRobot(self, (x_mm, y_mm, theta_deg), scale=1, color=ROBOT_COLOR_BGR, line_thickness=1):
                        
        # Get a polyline (e.g. triangle) to represent the robot icon
        robot_points = self.robot_polyline(scale)
        
        # Rotate the polyline by the current angle
        robot_points = map(lambda pt: rotate(pt, -theta_deg), robot_points)
                        
        # Move the polyline to the current robot position
        robot_points = map(lambda pt: (x_mm+pt[0], y_mm+pt[1]), robot_points)
        
        # Convert the robot position from meters (up is positive) to pixels (down is positive)
        robot_points = map(lambda pt: (self.mm2pix(pt[0]), self.map_size_pixels-self.mm2pix(pt[1])), robot_points)
        
        # Add an icon for the robot
        cv.PolyLine(self.image, [robot_points], True, color, line_thickness) 


    def displayScan(self, scan, offset_mm = (0,0), color=SCANPOINT_COLOR_BGR):
   
       for point in scan:
           cv.Circle(self.image, \
                     (self.mm2pix(point[0]+offset_mm[0]), self.mm2pix(point[1]+offset_mm[1])), \
                     SCANPOINT_RADIUS, color)
      
               
    def displayVelocities(self, dxy_mm, dtheta_deg):
        
        # Add velocity bars
        self.show_velocity(dxy_mm,      SENSOR_V_MAX_MM,      '   dXY', SENSOR_V_Y)
        self.show_velocity(dtheta_deg,  SENSOR_THETA_MAX_DEG, 'dTheta', SENSOR_THETA_Y)
                       
    def displayTrajectory(self, trajectory):
        
        for k in range(1, len(trajectory)):
            
            x1_mm, y1_mm = trajectory[k-1]
            x2_mm, y2_mm = trajectory[k]
            
            cv.Line(self.image, 
                (self.mm2pix(x1_mm), self.mm2pix(y1_mm)), \
                (self.mm2pix(x2_mm), self.mm2pix(y2_mm)), \
                TRAJECTORY_COLOR_BGR)
                
    def refresh(self):                   
                       
        # Display image
        cv.ShowImage(self.window_name, self.image)
                                         
        # Force image display, returning any key hit
        key = cvdisplay()
        return key if key > -1 else None
     

    def waitkey(self, action):
        
        print('Hit any key to %s ...' % action)
        
        key = -1
        
        while True:
            
            key = cvdisplay()
            if key > -1:
                break
        
        return key    
        
        
    # Puts text in the image to label the velocity display
    def show_velocity(self, value, valspan, label, y):
        cv.PutText(self.image, label+':', (SENSOR_TEXT_X, y), self.font, SENSOR_LABEL_COLOR_BGR) 
        bar_x1 = SENSOR_BAR_X + SENSOR_BAR_MAX_HEIGHT
        bar_y1 = y + SENSOR_BAR_Y_OFFSET
        bar_x2 = bar_x1 + int(value / valspan * SENSOR_BAR_MAX_HEIGHT)
        bar_y2 = y - SENSOR_BAR_WIDTH + SENSOR_BAR_Y_OFFSET
        bar_color = SENSOR_NEGATIVE_COLOR_BGR if value < 0 else SENSOR_POSITIVE_COLOR_BGR
        cv.Rectangle(self.image, (bar_x1, bar_y1), (bar_x2, bar_y2), bar_color, cv.CV_FILLED) 


    # Builds an array of points for a polyline representing the robot, pointing 
    # upward and centered at (0,0).
    # Currently builds an isoceles triangle pointing upward
    def robot_polyline(self, scale):
        xlft = -ROBOT_WIDTH / 2 * scale
        xrgt =  ROBOT_WIDTH / 2 * scale
        ybot = -ROBOT_HEIGHT / 2 * scale
        ytop =  ROBOT_HEIGHT / 2 * scale
        return [(xrgt,ybot), (0,ytop), (xlft,ybot)]
                        
    # Converts millimeters to pixels
    def mm2pix(self, mm):
        return int(mm * self.map_scale_pixels_per_mm)
                
# Helpers -------------------------------------------------------------        
        
# Forces OpenCV image display, returning id of key it or -1 if none            
def cvdisplay():
    return cv.WaitKey(1)
    
# Rotates a point by a specified number of degrees
def rotate(pt, deg):
    rad = radians(deg)
    c = cos(rad)
    s = sin(rad)
    x,y = pt  
    return int(x*c - y*s), int(x*s + y*c)
    




