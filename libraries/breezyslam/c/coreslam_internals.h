/*
coreslam_internals.h internal support for CoreSLAM

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
*/

#ifdef _MSC_VER
typedef __int64 int64_t;       /* Define it from MSVC's internal type */
#else
#include <stdint.h>            /* Use the C99 official header */
#endif


static const int NO_OBSTACLE            = 65500;
static const int OBSTACLE               = 0;


/* Implemented hardware-specifically */
void 
compute_distance(
    map_t * map, 
    scan_t * scan,
    double costheta, 
    double sintheta, 
    int pos_x_pix, 
    int pos_y_pix, 
    int * npoints, 
    int64_t * sum);

/* Helper for above */
void
add_if_in_bounds(
    map_t * map, 
    int x, 
    int y, 
    int * npoints, 
    int64_t * sum);
    
/* SIMD support */
void 
simd_init(
    double costheta,
    double sintheta,
    int pos_x_pix,
    int pos_y_pix);
