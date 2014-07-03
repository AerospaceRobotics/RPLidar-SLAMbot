/*

random.h - Function prototypes fora random-number generator supporting normal and
exponential distributions

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

/* Deallocates memory for a random-number generator */
void random_free(void * v);

/* Returns a  an exponential variate with density exp(-x),x>0 */
float random_rexp(void * v);

/* Returns a  standard normal variate with mean zero, variance 1 */
float random_rnor(void * v);

#ifdef __cplusplus 
}
#endif

