/*
cppcoreslam.cpp - C++ wrapper for CoreSLAM class

Copyright (C) 2013 Simon D. Levy

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
*/

#include "breezyslam.hpp"
#include "coreslam.h"

CoreSLAM::CoreSLAM(Laser & laser, int map_size_pixels, double map_scale_pixels_per_meter, 
                   int random_seed)
{
    // Set default params
    this->map_quality = DEFAULT_MAP_QUALITY;
    this->hole_width_meters = DEFAULT_HOLE_WIDTH_METERS;   
    
    this->sigma_xy_meters = DEFAULT_SIGMA_XY_METERS;
    this->sigma_theta_degrees = DEFAULT_SIGMA_THETA_DEGREES;

    this->state = new state_t;
    
    // Extract laser parameters for this CoreSLAM object 
    // (meters will be convereted to millimeters)
    this->state->laser.offset_meters = laser.offset_meters;
    this->state->laser.scan_rate = laser.scan_rate_hz;
    this->state->laser.angle_min = laser.angle_min_degrees;
    this->state->laser.angle_max = laser.angle_max_degrees;
    this->state->laser.detection_margin = laser.detection_margin;
    this->state->laser.distance_no_detection_meters = laser.distance_no_detection_meters;
    this->state->laser.scan_size = laser.scan_size;    
    
    state_init(
        this->state,
        map_size_pixels, 
        map_scale_pixels_per_meter, 
        random_seed);
}

CoreSLAM::CoreSLAM(Laser & laser, int map_size_pixels, double map_scale_pixels_per_meter)
{
    CoreSLAM(laser, map_size_pixels, map_scale_pixels_per_meter, -1);
}


CoreSLAM::~CoreSLAM(void)
{
    state_free(this->state);
    delete this->state; 
}


Position CoreSLAM::update(int * scanvals) 
{
    return update(scanvals, 0, 0, 0);
}


Position CoreSLAM::update(int * scanvals, double dxyMeters, double dthetaDegrees, double dtSeconds)
{    
    // Update state
    position_t pos =
    state_update(
        this->state, 
        scanvals,
        dxyMeters, 
        dthetaDegrees,
        dtSeconds,
        this->map_quality, 
        this->hole_width_meters, 
        this->sigma_xy_meters,
        this->sigma_theta_degrees);
    
    // Return current position from state
    return Position(pos.x, pos.y, pos.theta);
}


void CoreSLAM::getmap(unsigned char * mapbytes)
{
    state_get_map(this->state, (char *)mapbytes);
}

void CoreSLAM::getcloud(vector<class Position>  & cloud)
{
    
    for (int k=0; k<this->state->cloud_size; ++k)
    {     
        cloudpoint_t cloudpoint = this->state->point_cloud[k];  
        position_t pos = cloudpoint.position;
        cloud.push_back(Position(pos.x, pos.y, pos.theta, cloudpoint.likelihood));
    }
}

void CoreSLAM::setmap(unsigned char * mapbytes)
{
    state_set_map(this->state, (char *)mapbytes);
}



