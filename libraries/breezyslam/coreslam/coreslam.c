/*
coreslam.c adapted by Simon D. Levy from CoreSLAM.c, CoreSLAM_state.c, and
CoreSLAM.h downloaded from openslam.org on 01 January 2013

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
17-FEB-2014 : SDL - Random seed defaults to time if < 1 (was < 0)
01-MAR-2014 : SDL - Converted millimeters to meters for API
*/


static const int NO_OBSTACLE            = 65500;
static const int OBSTACLE               = 0;

static const double MAXDIFF_MM          = 10000;


#include "coreslam.h"
#include "random.h"

#include <limits.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <string.h>

#ifdef _MSC_VER
typedef __int64 int64_t;       /* Define it from MSVC's internal type */
#else
#include <stdint.h>            /* Use the C99 official header */
#endif

/* Utility functions--------------------------------------------------- */

static void 
swap(int * a, int * b)
{
    int tmp = *a;
    *a = *b;
    *b = tmp;
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

static int roundup(double x)
{
    return (int)floor(x + 0.5);
}

static double square(double x)
{
    return x * x;
}


/* Wrapper functions ------------------------------------------------ */

static double 
costheta(position_t position)
{
    return cos(radians(position.theta));
}

static double 
sintheta(position_t position)
{
    return sin(radians(position.theta));
}

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

static int * int_alloc(int size)
{
    return (int *)safe_malloc(size * sizeof(int));
}

static double * double_alloc(int size)
{
    return (double *)safe_malloc(size * sizeof(double));
}


/* Workhorse functions ------------------------------------------------ */

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
map_laser_ray(pixel_t * map_pixels,  int map_size, int x1, int y1, int x2, int y2, 
    int xp, int yp, int value, int alpha)
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


static void 
build_scan(state_t * state, 
           int * lidar, 
           scan_t * scan, 
           int hole_width_mm, 
           int span)
{    
    scan->npoints = 0;            
    
    laser_t laser = state->laser;

    /* Span the laser scans to better cover the space */
    int i;
    for (i =laser.detection_margin+1; i<laser.scan_size-laser.detection_margin; ++i)
    {
        int lidar_value = lidar[i];
        
        int distance = -1; /* OOB values */
        int scanval = -1;
        
        if (lidar_value == 0)
        {
            distance = laser.distance_no_detection_meters * 1000; // m to mm
            scanval = NO_OBSTACLE;
        }
        
        else if (lidar_value > hole_width_mm / 2)
        {
            distance = lidar_value;
            scanval = OBSTACLE;
        }
        
        else
        {
            continue;
        }
        
        int degrees_per_second = laser.scan_rate * 360;
        
        int j;
        for (j=0; j<span; ++j)
        {    
            double scale = (double)(i * span + j)  * (laser.angle_max - laser.angle_min) / 
                           (laser.scan_size * span - 1);
            double angle = radians(laser.angle_min + scale * 
                           (1 + state->dtheta_degrees / degrees_per_second));
            
            /* Convert m/s to mm/s */
            double dxy_mm = state->dxy_meters * 1000;
            
            double x = distance * cos(angle) - dxy_mm * scale / degrees_per_second;
            double y = distance * sin(angle);

            scan->x_mm[scan->npoints] = x;
            scan->y_mm[scan->npoints] = y;
            scan->value[scan->npoints] = scanval;
            scan->npoints++;
        }     
    }      
}


static double
init_coord_mm(state_t * state)
{
    return 0.5 * state->map_size / state->map_scale;
}

position_t get_default_start_position(state_t * state)
{
    position_t startpos;
    
    startpos.x = init_coord_mm(state);
    startpos.y = init_coord_mm(state);
    startpos.theta = 0;
    
    return startpos;
}


static void
update_map(state_t * state, position_t likeliest_position, double c, double s,
    int map_quality, int hole_width_mm)
{
    int x1 = roundup(likeliest_position.x * state->map_scale);
    int y1 = roundup(likeliest_position.y * state->map_scale);
    
    scan_t scan = state->scan_for_update;
    
    /* Translate and rotate scan to robot position */
    int i = 0;
    for (i = 0; i != scan.npoints; i++) 
    {
        double x2p = c * scan.x_mm[i] - s * scan.y_mm[i];
        double y2p = s * scan.x_mm[i] + c * scan.y_mm[i];
        
        int xp = roundup((likeliest_position.x + x2p) * state->map_scale);
        int yp = roundup((likeliest_position.y + y2p) * state->map_scale);
        
        double dist = sqrt(x2p * x2p + y2p * y2p);
        double add = hole_width_mm / 2 / dist;
        x2p *= state->map_scale * (1 + add);
        y2p *= state->map_scale * (1 + add); 
                
        int x2 = roundup(likeliest_position.x * state->map_scale + x2p);
        int y2 = roundup(likeliest_position.y * state->map_scale + y2p);
                
        int value = OBSTACLE;
        int q = map_quality;
        
        if (scan.value[i] == NO_OBSTACLE) 
        { 
            q = map_quality / 4;
            value = NO_OBSTACLE;
        }
                        
        /* Update current map */
        map_laser_ray(state->map_history[0], state->map_size, 
            x1, y1, x2, y2, xp, yp, value, q);        
    }            
}

static void scan_init(scan_t * scan, int size)
{
    scan->x_mm = double_alloc(size);
    scan->y_mm = double_alloc(size);
    scan->value = int_alloc(size);
    
    scan->npoints = 0;
}


static void scan_free(scan_t * scan)
{
    free(scan->x_mm);
    free(scan->y_mm);
    free(scan->value);
}

/* Turn point-cloud distances into probabilities */
static void normalize_pointcloud(state_t * state)
{    
    double sum = 0;
    int k;
    
    for (k=0; k<state->cloud_size; ++k)
    {
        sum += state->point_cloud[k].likelihood;   
    }    
    
    for (k=0; k<state->cloud_size; ++k)
    {        
        state->point_cloud[k].likelihood /= sum; 
    }    
}

/* Exported functions ------------------------------------------------ */

void
state_init(
    state_t * state, 
    int map_size_pixels, 
    double map_scale_pixels_per_meter, 
    int random_seed)
{    

    /* Start with zero velocities */
    state->dxy_meters = 0;
    state->dtheta_degrees = 0;
    
    /* Use current time as random seed if none provided */
    if (random_seed < 1)
    {
        random_seed = time(NULL) % 0xFFFFFFFF;
    }
    
    /* Create a randomizer for the particle filter */
    state->randomizer = random_init(random_seed);
    
    int scan_size = state->laser.scan_size;
    
    /* Initialize a scan for computing distance to map, and one for updating map */
    scan_init(&state->scan_for_distance, scan_size);
    scan_init(&state->scan_for_update, scan_size*3);
    
    /* Initialize the map */
    int npix = map_size_pixels * map_size_pixels;
    state->map_history[0] = (pixel_t *)safe_malloc(npix * sizeof(pixel_t));
    int k;
    for (k=0; k<npix; ++k)
    {
        state->map_history[0][k] = (OBSTACLE + NO_OBSTACLE) / 2;
    }
    
    state->map_size = map_size_pixels;
    
    /* Convert scale from pixels per meter to pixels per millimeter */
    state->map_scale = map_scale_pixels_per_meter / 1000;
    
    /* Initialize the robot's position */
    state->position_history[0] = get_default_start_position(state);
    
    /* TinySLAM 1.1 */
    state->history_count = 0;
    
}

void
state_free(
    state_t * state)
{
    scan_free(&state->scan_for_distance);
    scan_free(&state->scan_for_update);
    
    free(state->map_history[0]);
    
    random_free(state->randomizer);
}

position_t 
state_update(
    state_t * state, 
    int * lidar,
    double dxy_meters, 
    double dtheta_degrees,
    double dt_seconds,
    int map_quality, 
    double hole_width_meters, 
    double sigma_xy_meters,
    double sigma_theta_degrees)
{            
    /* Convert hole width to millimeters */
    int hole_width_mm = hole_width_meters * 1000;
        
    /* Build a scan for computing distance to map, and one for updating map */
    build_scan(state, lidar, &state->scan_for_update,   hole_width_mm, 3);
    build_scan(state, lidar, &state->scan_for_distance, hole_width_mm, 1);
  
    /* Update velocities */
    double velocity_factor = (dt_seconds > 0) ?  (1 / dt_seconds) : 0;
    state->dxy_meters = dxy_meters * velocity_factor;
    state->dtheta_degrees = dtheta_degrees * velocity_factor;
        
    double c = costheta(state->position_history[0]);
    double s = sintheta(state->position_history[0]);
    
    position_t search_start_position = state->position_history[0];
    
    /* Convert m to mm */
    double dxy_mm = dxy_meters * 1000;
    double offset_mm = state->laser.offset_meters * 1000;
    
    search_start_position.x += dxy_mm * c;
    search_start_position.y += dxy_mm * s;
    
    search_start_position.theta += dtheta_degrees;
    
    search_start_position.x += offset_mm * c;
    search_start_position.y += offset_mm * s;
         
    /* Start with a new point-cloud */
    state->cloud_size = 0;
    
    position_t likeliest_position = 
    
    /* Optimization -- only run particle filter search when search params are non-zero */
    sigma_xy_meters > 0 && sigma_theta_degrees > 0 ?
        
    particle_filter_search(state, sigma_xy_meters, sigma_theta_degrees, search_start_position) : 
    
    search_start_position;
        
    /* Turn point-cloud likelihoods into probabilities */
    normalize_pointcloud(state);
    
    /* Update the current state with the  a modified version of the new position */
    
    state->position_history[0] = likeliest_position;    
    
    c = costheta(state->position_history[0]);
    s = sintheta(state->position_history[0]);
    
    state->position_history[0].x -= offset_mm * c;
    state->position_history[0].y -= offset_mm * s;
    
    double dist = 0;
    
    if (state->history_count > 4)
    {
        double wgts[5] = {1, 2, 4, 2, 1};
        double x = 0;
        double y = 0;
        double wgtsum = 0;
        int j = 0;
        for (j=0; j<5; ++j)
        {
            position_t position = state->position_history[j];
            x += wgts[j] * position.x;
            y += wgts[j] * position.y;
            wgtsum += wgts[j];
        }
        x /= wgtsum;
        y /= wgtsum;
        position_t position = state->position_history[0];
        dist = sqrt(square(position.x-x) + square(position.y-y));
        
    }
            
    if (dist < MAXDIFF_MM)
    {
        update_map(state, likeliest_position, c, s, map_quality, hole_width_mm);
    }
    
    /* TinySLAM 1.1 */
    state->history_count++;
    int j = (state->history_count < 4) ? state->history_count : 4;
    while (j > 0)
    {
        state->position_history[j] = state->position_history[j-1];
        j--;
    }
    
    // Return current position in meters, degrees
    position_t pos = state->position_history[0];
    pos.x /= 1000;
    pos.y /= 1000;
    return pos;
}

/* adapted from distance_scan_to_map() */
double 
state_get_position_likelihood(
    state_t * state, 
    position_t position)
{
    int npoints = 0;
    int64_t sum = 0;
    
    double c = costheta(position) * state->map_scale;
    double s = sintheta(position) * state->map_scale;
    
    double pos_x = position.x * state->map_scale;
    double pos_y = position.y * state->map_scale;
    
    scan_t scan = state->scan_for_distance;
    
    /* Translate and rotate scan to robot position
    and compute the distance */
    int i = 0;
    for (i = 0; i != state->scan_for_distance.npoints; i++) {
        
        if (scan.value[i] != NO_OBSTACLE) {
            
            int x = roundup(pos_x + c * scan.x_mm[i] - s * scan.y_mm[i]);
            int y = roundup(pos_y + s * scan.x_mm[i] + c * scan.y_mm[i]);
            
            /* Check boundaries */
            if (!out_of_bounds(x,state->map_size) && !out_of_bounds(y, state->map_size)) 
            {
                sum += state->map_history[0][y * state->map_size + x];
                npoints++;
            }
            
        }
    }
    
    return npoints ? (INT_MAX - (int)(sum / npoints)) / (double)INT_MAX  : 0;        
}


void
state_get_map(
    state_t * state, 
    char * bytes)
{    
    int k;
    for (k=0; k<state->map_size*state->map_size; ++k)
    {
        bytes[k] = state->map_history[0][k] >> 8;
    }
}

void
state_set_map(
    state_t * state, 
    char * bytes)
{    
    int k;
    for (k=0; k<state->map_size*state->map_size; ++k)
    {
        state->map_history[0][k] = bytes[k];
        state->map_history[0][k] <<= 8;
    }
}

void 
state_add_cloudpoint(
    state_t * state,
    position_t position,
    double likelihood)
{
    /* Convert millimeters to meters */
    position.x /= 1000;
    position.y /= 1000;
	
    state->point_cloud[state->cloud_size].position = position;    
    
    /* We will turn distance into likelihood later */
    state->point_cloud[state->cloud_size].likelihood = likelihood;
        
    state->cloud_size++;
    
    /* This should never happen */
    if (state->cloud_size > MAX_POINTCLOUD_SIZE)
    {
        fprintf(stderr, "Cloud size = %d; cannot add a new point\n",
            state->cloud_size);
        exit(1);
    }
}
