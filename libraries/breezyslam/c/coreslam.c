/*
coreslam.c adapted from CoreSLAM.c, CoreSLAM_state.c, and CoreSLAM.h 
downloaded from openslam.org on 01 January 2014.  Contains implementations 
of scan and map methods.

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

Change log

07-FEB-2014 : Simon D. Levy  - Initial release
01-MAR-2014 : SDL - Converted millimeters to meters for API
21-JUN-2014 : SDL - Added support for SSE and NEON
*/


#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <string.h>
#include <math.h>

#include "coreslam.h"
#include "coreslam_internals.h"

#include "random.h"

/* Local helpers--------------------------------------------------- */

static void * safe_malloc(size_t size)
{
    void * v = malloc(size);
    
    if (!v)
    {
        fprintf(stderr, "Unable to allocate %lu bytes\n", (unsigned long)size);
        exit(1);
    }
    
    return v;
}


static double * double_alloc(int size)
{
    return (double *)safe_malloc(size * sizeof(double));
}

static float * float_alloc(int size)
{
    return (float *)safe_malloc(size * sizeof(float));
}


static void 
swap(int * a, int * b)
{
    int tmp = *a;
    *a = *b;
    *b = tmp;
}


static int roundup(double x)
{
    return (int)floor(x + 0.5);
}


static int 
out_of_bounds(int value, int bound)
{
    return value  < 0 || value >= bound;
}


static double 
radians(double degrees)
{
    return degrees * M_PI / 180;
}

static int 
clip(int *xyc, int * yxc, int xy, int yx, int map_size)
{
    
    if (*xyc < 0) 
    {
        if (*xyc == xy) 
        {
            return 1;
        }
        *yxc += (*yxc - yx) * (- *xyc) / (*xyc - xy);
        *xyc = 0;
    }
    
    if (*xyc >= map_size)
    {
        if (*xyc == xy) 
        {
            return 1;
        }
        *yxc += (*yxc - yx) * (map_size - 1 - *xyc) / (*xyc - xy);
        *xyc = map_size - 1;
    }
    
    return 0;
}


static void
map_laser_ray(
    pixel_t * map_pixels,  
    int map_size, 
    int x1, 
    int y1, 
    int x2, 
    int y2, 
    int xp, 
    int yp, 
    int value, 
    int alpha)
{            
    if (out_of_bounds(x1, map_size) || out_of_bounds(y1, map_size))
    {
        return; 
    }
    
    int x2c = x2; 
    int y2c = y2;
    
    if (clip(&x2c, &y2c, x1, y1, map_size) || clip(&y2c, &x2c, y1, x1, map_size)) 
    {
        return;
    }
    
    int dx = abs(x2 - x1); 
    int dy = abs(y2 - y1);
    int dxc = abs(x2c - x1); 
    int dyc = abs(y2c - y1);
    int incptrx = (x2 > x1) ? 1 : -1;
    int incptry = (y2 > y1) ? map_size : -map_size;
    int sincv = (value > NO_OBSTACLE) ? 1 : -1; 
    
    
    int derrorv = 0;
    
    if (dx > dy) 
    {
        derrorv = abs(xp - x2);
    } 
    else 
    {
        swap(&dx, &dy); 
        swap(&dxc, &dyc); 
        swap(&incptrx, &incptry);    
        derrorv = abs(yp - y2);
    }
    
    int error = 2 * dyc - dxc;
    int horiz = 2 * dyc;
    int diago = 2 * (dyc - dxc);
    int errorv = derrorv / 2;
    
    int incv = (value - NO_OBSTACLE) / derrorv;   
    int incerrorv = value - NO_OBSTACLE - derrorv * incv;     
    
    pixel_t * ptr = map_pixels + y1 * map_size + x1;
    int pixval = NO_OBSTACLE;
    
    int x = 0;
    for (x = 0; x <= dxc; x++, ptr += incptrx) 
    {
        if (x > dx - 2 * derrorv) 
        {
            if (x <= dx - derrorv) 
            {
                pixval += incv;
                errorv += incerrorv;
                if (errorv > derrorv) 
                {
                    pixval += sincv;
                    errorv -= derrorv; 
                }
            } 
            else 
            {
                pixval -= incv;
                errorv -= incerrorv;
                if (errorv < 0) 
                {
                    pixval -= sincv;
                    errorv += derrorv; 
                }
            }
        } 
        
        /* Integration into the map */
        *ptr = ((256 - alpha) * (*ptr) + alpha * pixval) >> 8;      
        
        if (error > 0) 
        {
            ptr += incptry;
            error += diago;
        } else 
        {
            error += horiz;
        }
    }    
}

static double 
random_normal(
    void * randomizer, 
    double mu, double 
    sigma)
{
	return mu + sigma * random_rnor(randomizer);
}


static void 
scan_update_xy(
    scan_t * scan,
    int offset,
    int distance,
    int scanval,
    laser_t laser,
    double horz_mm,
    double rotation)
{
    int j;
    for (j=0; j<scan->span; ++j)
    {    
        double k = (double)(offset*scan->span+j) * (laser.angle_max_degrees-laser.angle_min_degrees) / 
                   (laser.scan_size * scan->span - 1);
        double angle = radians(laser.angle_min_degrees + k * rotation); 
        
        double x = distance * cos(angle) - k * horz_mm;
        double y = distance * sin(angle);
        
        scan->value[scan->npoints] = scanval;
        
        scan->x_mm[scan->npoints] = x;
        scan->y_mm[scan->npoints] = y;
        scan->npoints++;
    }
}


/* Exported functions --------------------------------------------------------*/

int * 
int_alloc(
    int size)
{
    return (int *)safe_malloc(size * sizeof(int));
}

void 
map_init(
    map_t * map, 
    int size_pixels, 
    int scale_mm_per_pixel)
{    
    int npix = size_pixels * size_pixels;
    
    map->pixels = (pixel_t *)safe_malloc(npix * sizeof(pixel_t));
    
    int k;
    for (k=0; k<npix; ++k)
    {
        map->pixels[k] = (OBSTACLE + NO_OBSTACLE) / 2;
    }    
    
    map->size_pixels = size_pixels;
    
    map->scale_mm_per_pixel = scale_mm_per_pixel;     
}

void
map_free(
    map_t * map)
{
    free(map->pixels);
}

void map_string(
    map_t map,
    char * str)
{
    sprintf(str, "size = %d x %d pixels | scale = %d mm/pixel", 
        map.size_pixels, map.size_pixels, map.scale_mm_per_pixel);
}

void 
map_update(
    map_t * map,
    scan_t * scan, 
    position_t position,
    int map_quality,
    double hole_width_mm)
{
    double position_theta_radians = radians(position.theta_degrees);
    double costheta = cos(position_theta_radians);
    double sintheta = sin(position_theta_radians);
    
    int x1 = roundup(position.x_mm / map->scale_mm_per_pixel);
    int y1 = roundup(position.y_mm / map->scale_mm_per_pixel);

    int i = 0;
    for (i = 0; i != scan->npoints; i++) 
    {
        double x2p = costheta * scan->x_mm[i] - sintheta * scan->y_mm[i];
        double y2p = sintheta * scan->x_mm[i] + costheta * scan->y_mm[i];
        
        int xp = roundup((position.x_mm + x2p) / map->scale_mm_per_pixel);
        int yp = roundup((position.y_mm + y2p) / map->scale_mm_per_pixel);
        
        
        double dist = sqrt(x2p * x2p + y2p * y2p);
        double add = hole_width_mm / 2 / dist;
        x2p *= 1./map->scale_mm_per_pixel * (1 + add);
        y2p *= 1./map->scale_mm_per_pixel * (1 + add); 
        
        int x2 = roundup(position.x_mm / map->scale_mm_per_pixel + x2p);
        int y2 = roundup(position.y_mm / map->scale_mm_per_pixel + y2p);
        
        int value = OBSTACLE;
        int q = map_quality;
        
        if (scan->value[i] == NO_OBSTACLE) 
        { 
            q = map_quality / 4;
            value = NO_OBSTACLE;
        }
        
        map_laser_ray(map->pixels, map->size_pixels, x1, y1, x2, y2, xp, yp, value, q);        
    }            

}    

void
map_get(
    map_t * map, 
    char * bytes)
{    
    int k;
    for (k=0; k<map->size_pixels*map->size_pixels; ++k)
    {
        bytes[k] = map->pixels[k] >> 8;
    }

}


void
map_set(
    map_t * map, 
    char * bytes)
{    
    int k;
    for (k=0; k<map->size_pixels*map->size_pixels; ++k)
    {
        map->pixels[k] = bytes[k];
        map->pixels[k] <<= 8;
    }
}

void 
scan_init(
    scan_t * scan, 
    int size,
    int span)
{
    scan->x_mm = double_alloc(size*span);
    scan->y_mm = double_alloc(size*span); 
    scan->value = int_alloc(size*span);
    scan->span = span;
    
    /* assure size multiple of 4 for SSE */
    scan->obst_x_mm = float_alloc(size*span+4);
    scan->obst_y_mm = float_alloc(size*span+4);
}


void 
scan_free(
    scan_t * scan)
{
    free(scan->x_mm);
    free(scan->y_mm);
    free(scan->value);
    
    free(scan->obst_x_mm);
    free(scan->obst_y_mm);
}

void scan_string(
    scan_t scan,
    char * str)
{
    sprintf(str, "%d obstacle points | %d free points", scan.obst_npoints, scan.npoints-scan.obst_npoints);    
}

void 
scan_update(
    scan_t * scan, 
    int * lidar_mm, 
    laser_t laser,
    double hole_width_mm,
    double velocities_dxy_mm,
    double velocities_dtheta_degrees)
{        
    scan->npoints = 0;
    scan->obst_npoints = 0;
    
    /* Take velocity into account */
    int degrees_per_second = laser.scan_rate_hz * 360;
    double horz_mm = velocities_dxy_mm / degrees_per_second;
    double rotation = 1 + velocities_dtheta_degrees / degrees_per_second;
       
    /* Span the laser scans to better cover the space */
    int i;
    for (i=laser.detection_margin+1; i<laser.scan_size-laser.detection_margin; ++i)
    {
        int lidar_value_mm = lidar_mm[i];
        
        /* No obstacle */
        if (lidar_value_mm == 0)
        {
            scan_update_xy(scan, i, laser.distance_no_detection_mm, NO_OBSTACLE, laser, horz_mm, rotation);
        }
        
        /* Obstacle */
        else if (lidar_value_mm > hole_width_mm / 2)
        {
            int oldstart = scan->npoints;
        
            scan_update_xy(scan, i, lidar_value_mm, OBSTACLE, laser, horz_mm, rotation);
            
            /* Store obstacles separately for SSE */
            int j;
            for (j=oldstart; j<scan->npoints; ++j)
            {
                if (scan->value[j] == OBSTACLE)
                {                
                    scan->obst_x_mm[scan->obst_npoints] = scan->x_mm[j];
                    scan->obst_y_mm[scan->obst_npoints] = scan->y_mm[j];
                    scan->obst_npoints++;
                }
            }
        }
    }      
}

void 
laser_string(
    laser_t laser, 
    char * str)
{
    sprintf(str, 
        "scan_size=%d | scan_rate=%d hz | angle_min=%d deg | angle_max=%d deg | " 
        "distance_no_detection=%7.4f mm | detection_margin=%d | offset=%4.4f m",
        laser.scan_size,
        laser.scan_rate_hz, 
        laser.angle_min_degrees, 
        laser.angle_max_degrees,
        laser.distance_no_detection_mm,
        laser.detection_margin, 
        laser.offset_mm
        );
}    



position_t 
rmhc_position_search(
    position_t start_pos,
	map_t * map,
    scan_t * scan,
    laser_t laser,
	double sigma_xy_mm,
	double sigma_theta_degrees,
	int max_search_iter,
	void * randomizer)
{   	    
	position_t currentpos = start_pos;
	position_t bestpos = start_pos;
	position_t lastbestpos = start_pos;
	
	int current_distance = distance_scan_to_map(map, scan, currentpos);
	
	int lowest_distance =  current_distance;   
	int last_lowest_distance = current_distance;   
	
	int counter = 0;
	do 
	{    
		currentpos = lastbestpos;
		
		currentpos.x_mm = random_normal(randomizer, currentpos.x_mm, sigma_xy_mm);
		currentpos.y_mm = random_normal(randomizer, currentpos.y_mm, sigma_xy_mm); 	
		currentpos.theta_degrees = random_normal(randomizer, currentpos.theta_degrees, sigma_theta_degrees);
		
		current_distance = distance_scan_to_map(map, scan, currentpos);
				
		/* -1 indicates infinity */		
		if ((current_distance > -1) && (current_distance < lowest_distance))
		{
			lowest_distance = current_distance;
			bestpos = currentpos;
		} 
		else 
		{
			counter++;
		}
		
		if (counter > max_search_iter / 3) 
		{
			if (lowest_distance < last_lowest_distance) 
			{
				lastbestpos = bestpos;
				last_lowest_distance = lowest_distance;
				counter = 0;
				sigma_xy_mm *= 0.5;
				sigma_theta_degrees *= 0.5;
			}
		}
		
	} while (counter < max_search_iter);
    
	return bestpos;
}



int 
distance_scan_to_map(
    map_t *  map,
    scan_t * scan,
    position_t position)
{    
    int npoints = 0; /* number of points where scan matches map */
    int64_t sum = 0; /* sum of map values at those points */
    
    /* Pre-compute sine and cosine of angle for rotation */
    double position_theta_radians = radians(position.theta_degrees);
    double costheta = cos(position_theta_radians) / map->scale_mm_per_pixel;
    double sintheta = sin(position_theta_radians) / map->scale_mm_per_pixel;
    
    /* Pre-compute pixel offset for translation */
    double pos_x_pix = position.x_mm / map->scale_mm_per_pixel;
    double pos_y_pix = position.y_mm / map->scale_mm_per_pixel;

    /* Compute number of matching points and their sum */
    compute_distance(map, scan, costheta, sintheta, pos_x_pix, pos_y_pix, &npoints, &sum);
        
    /* Return sum scaled by number of points, or -1 if none */
    return npoints ? (int)(sum * 1024 / npoints) : -1;  
}

void
add_if_in_bounds(
    map_t * map, 
    int x, 
    int y, 
    int * npoints, 
    int64_t * sum)
{
    /* Check boundaries */
    if (!out_of_bounds(x, map->size_pixels) && !out_of_bounds(y,  map->size_pixels)) 
    {
        sum += map->pixels[y * map->size_pixels + x];
        npoints++;
    }
}

