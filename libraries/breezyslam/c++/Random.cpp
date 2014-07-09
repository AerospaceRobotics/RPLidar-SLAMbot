/**
* 
* \mainpage BreezySLAM: Simple, efficient SLAM in C++
*
* Random.cpp - C++ code for Random class
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

#include <time.h>
#include <stdlib.h>

#include <iostream>
#include <vector>
using namespace std; 

#include "Random.hpp"
#include "random.h"

Random::Random(unsigned seed)
{    
    this->seed = seed;
    
    this->randomizer = random_init(seed);
}

Random::Random(Random & src)
{
    this->seed = src.seed;
    
    this->randomizer = random_copy(src.randomizer);
}

Random::~Random(void)
{    
    random_free(this->randomizer);
}

float Random::rnor(void)
{
    return random_rnor(this->randomizer);
}

float Random::rexp(void)
{
    return random_rexp(this->randomizer);
}

