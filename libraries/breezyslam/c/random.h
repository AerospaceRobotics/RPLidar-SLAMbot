/*

random.h - Function prototypes fora random-number generator supporting normal 
distributions

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

#ifdef __cplusplus 
extern "C" {
#endif

/* Creates and initializes a new random-number generator */
void * random_init(int seed);

/* Make a copy of the specified random-number generator */
void * random_copy(void * r);

/* Deallocates memory for a random-number generator */
void random_free(void * v);

/* Returns a  standard normal variate with mean mu, variance sigma */
double random_normal(void * v, double mu, double sigma);

#ifdef __cplusplus 
}
#endif

