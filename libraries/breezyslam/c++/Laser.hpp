/**
*
* Laser.hpp - C++ header for Laser model class
*
* Copyright (C) 2014 Simon D. Levy

* This code is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as 
* published by the Free Software Foundation, either version 3 of the 
* License, or (at your option) any later version.
* 
* This code is distributed in the hope that it will be useful,     
* but WITHOUT ANY WARRANTY without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
* 
* You should have received a copy of the GNU Lesser General Public License 
* along with this code.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdio.h>
#include <math.h>

#include <iostream>
#include <vector>
using namespace std; 


/**
* A class for scanning laser rangefinder (Lidar) parameters.
*/
class Laser  
{
    friend class CoreSLAM;
    friend class SinglePositionSLAM;
    friend class RMHC_SLAM;
    friend class Scan;
    
public:
    
    /**
    * Builds a Laser object from parameters based on the specifications for your 
    * Lidar unit.
    * @param scan_size                  number of rays per scan
    * @param scan_rate_hz               laser scan rate in Hertz
    * @param angle_min_degrees          minimum laser angle in degrees
    * @param angle_max_degrees          maximum laser angle in degrees
    * @param distance_no_detection_mm   scan distances above this are treated as infinfity
    * @param detection_margin           number of rays at edges of scan to ignore
    * @param offset_mm                  forward/backward offset of laser motor from robot center
    * @return a new Laser object
    * 
    */
    Laser(
        int scan_size,
        int scan_rate_hz,
        int angle_min_degrees,
        int angle_max_degrees,
        float distance_no_detection_mm,
        int detection_margin,
        float offset_mm
        );
    
    /**
    * Builds an empty Laser object (all parameters zero).
    */
    Laser(void);
    
    /**
    * Dealloates memory for this Laser object.
    */
    ~Laser(void);
    
    
    friend ostream& operator<< (ostream & out, Laser & laser);

private:
    
    struct laser_t * laser;
    
};


