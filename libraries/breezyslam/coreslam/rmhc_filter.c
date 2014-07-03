/*

rmhc_filter.c Random-Mutation Hill-Climber particle filter for CoreSLAM

adapted by Simon D. Levy from CoreSLAM_random.c downloaded from openslam.org
on 01 January 2013

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

Change log:

15-MAR-2014 Simon D. Levy - Initial release

03-APR-2014 SDL           - Use minimum-improvement check to solve 
                            floating-point comparison problem on
                            32-bit architectures (caused point 
                            cloud to grow without bounds)
*/

#include <stdlib.h>
#include <math.h>
#include <strings.h>
#include <stdio.h>

#include <limits.h>

#include "coreslam.h"
#include "random.h"

/* helps avoid rounding error */
static const double MINIMUM_IMPROVEMENT = 1e-6;

static double random_normal(void * randomizer, double mu, double sigma)
{
	return mu + sigma * random_rnor(randomizer);
}

position_t 
particle_filter_search(
	state_t * state, 
	double sigma_xy_meters,
	double sigma_theta_degrees,
	position_t start_pos)
{   	
	position_t currentpos = start_pos;
	position_t bestpos = start_pos;
	position_t lastbestpos = start_pos;
	
	double current_likelihood = state_get_position_likelihood(state, currentpos);
        
	double highest_likelihood =  current_likelihood;   
	double last_highest_likelihood = current_likelihood;   
        
	state_add_cloudpoint(state, bestpos, highest_likelihood);
	
	double sigma_xy_mm = sigma_xy_meters * 1000;
	
	int counter = 0;
	do 
	{    
		currentpos = lastbestpos;
		
		currentpos.x = random_normal(state->randomizer, currentpos.x, sigma_xy_mm);
		currentpos.y = random_normal(state->randomizer, currentpos.y, sigma_xy_mm); 	
		currentpos.theta = random_normal(state->randomizer, currentpos.theta, sigma_theta_degrees);
				
		current_likelihood = state_get_position_likelihood(state, currentpos);
				
                /* avoid rounding error */
		if ((current_likelihood-highest_likelihood) > MINIMUM_IMPROVEMENT)
                {
			highest_likelihood = current_likelihood;
			bestpos = currentpos;
			state_add_cloudpoint(state, bestpos, highest_likelihood);
		} 
		else 
		{
			counter++;
		}
		
		if (counter > MAX_POINTCLOUD_SIZE / 3) 
		{
			if (highest_likelihood < last_highest_likelihood) 
			{
				lastbestpos = bestpos;
				last_highest_likelihood = highest_likelihood;
				counter = 0;
				sigma_xy_meters *= 0.5;
				sigma_theta_degrees *= 0.5;
			}
		}
		
		
	} while (counter < MAX_POINTCLOUD_SIZE);
	
	
	return bestpos;
}


