/**
* 
* Random.hpp - C++ header for Random class
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
* Pseudorandom number generator class.
*/
class Random 
{    
    friend class CoreSLAM;
    
public:
    
    /**
    * Creates a pseudorandom number generator with a specified seed.
    * @param seed the seed
    */
    Random(unsigned seed);
    
    /**
    * Creates a copy of another pseudorandom number generator.
    * @param src the other pseudorandom number generator
    */
    Random(Random & src);

    /**
    * Deallocates this pseudorandom number generator.
    */
    ~Random(void);
    

    /**
    * Returns a  standard normal variate with mean zero, variance 1.
    */
    float rnor(void);
    
    /**
    * Returns a  an exponential variate with density exp(-x),x>0.
    */
    float rexp(void);
    
    friend ostream& operator<< (ostream & out, Random & random)
    {
        char str[100];
        
        sprintf(str, "<seed = %d>", random.seed);
        
        out << str;
        
        return out;
    }
    
private:
    
    void * randomizer;
    
    unsigned seed; // for reporting
        
};

