/*

coreslam.h adapted by Simon D. Levy from CoreSLAM.h downloaded from
openslam.org on 01 January 2013

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
*/

static const int DEFAULT_MAP_QUALITY            = 50; /* out of 255 */
static const double DEFAULT_HOLE_WIDTH_METERS   = 0.6;

static const double DEFAULT_SIGMA_XY_METERS     = 0.1;
static const double DEFAULT_SIGMA_THETA_DEGREES = 20;

#define MAX_POINTCLOUD_SIZE                     1000

typedef unsigned short pixel_t;

typedef struct 
{
    double x, y; // Units are millimeters (internally) or meters (externally)
    double theta;
    
} position_t;

typedef struct 
{
    position_t position;
    double likelihood;
    
} cloudpoint_t;


typedef struct
{
    double * x_mm;
    double * y_mm;
    int    * value;
    
    int npoints;
    
} scan_t;

typedef struct
{
    double offset_meters;                   /* position of the laser wrt center of rotation */
    int scan_size;                          /* number of points per scan */
    int scan_rate;                          /* scans per second */
    int angle_min;                          /* start angle for scan */
    int angle_max;                          /* end angle for scan */
    int detection_margin;                   /* first scan element to consider */
    double distance_no_detection_meters;    /* default value when the laser returns 0 */
    
} laser_t;


typedef struct state_t
{
    void * randomizer;
    cloudpoint_t point_cloud[MAX_POINTCLOUD_SIZE];
    int cloud_size;
    laser_t laser;
    double dxy_meters;
    double dtheta_degrees;
    int map_size;
    double map_scale;
    int map_quality;
    scan_t scan_for_distance;
    scan_t scan_for_update;
    
    /* TinySLAM 1.1 */
    pixel_t * map_history[3];
    position_t position_history[5];
    int history_count;
        
} state_t;	

/* These routines are in coreslam.c ----------------------------------------- */

#ifdef __cplusplus 
extern "C" {
#endif

void
state_init(
    state_t * state, 
    int map_size_pixels, 
    double map_scale_pixels_per_meter, 
    int random_seed);

void
state_free(
    state_t * state);

// returns position in meters, degrees
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
    double sigma_theta_degrees);

double 
state_get_position_likelihood(
    state_t * state, 
    position_t position);

void
state_get_map(
    state_t * state, 
    char * bytes);

void
state_set_map(
    state_t * state, 
    char * bytes);

void 
state_add_cloudpoint(
    state_t * state,
    position_t position,
    double likelihood);


/* This routine is in rmhc_filter.c or whatever particle filter you are using ------ */

position_t 
particle_filter_search(
    state_t * state,            /* current map and scan */
    double sigma_xy_meters,     /* standard deviation for position search */
    double sigma_theta_degrees, /* standard deviation for angle search */
    position_t start_pos);      /* starting position */

#ifdef __cplusplus 
}
#endif


