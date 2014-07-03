/*

ziggrand.c Ziggurat-method random-number generator supporting multiple instances

Adapted by Simon D. Levy from http://www.ka9q.net/suitsat/dsp/rnorrexp.c
downloaded 01 January 2013.

Based on

    @article{Marsaglia:Tsang:2000:JSSOBK:v05i08,
      author =	"George Marsaglia and Wai Wan Tsang",
      title =	"The Ziggurat Method for Generating Random Variables",
      journal =	"Journal of Statistical Software",
      volume =	"5",
      number =	"8",
      pages =	"1--7",
      day =  	"2",
      month =	"10",
      year = 	"2000",
      CODEN =	"JSSOBK",
      ISSN = 	"1548-7660",
      bibdate =	"2000-10-02",
      URL =  	"http://www.jstatsoft.org/v05/i08",
      accepted =	"2000-10-02",
      submitted =	"2000-04-15
    }

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

07-MAR-2014 : Simon D. Levy  - Initialize jsr to 123456789
*/

#include <math.h>

typedef unsigned int uint32;

typedef signed int int32;

typedef struct
{
    
    uint32 jz;
    uint32 jsr;
    int32 hz;
    uint32 iz;
    uint32 kn[128];
    uint32 ke[256];
    float wn[128];
    float fn[128];
    float we[256];
    float fe[256];
    
} randomizer_t;


static uint32 shr3(randomizer_t * rand) 
{    
    rand->jz = rand->jsr;
    rand->jsr ^= (rand->jsr<<13);
    rand->jsr ^=(rand->jsr>>17);
    rand->jsr ^= (rand->jsr<<5);
    return rand->jz + rand->jsr;
}

static float uni(randomizer_t * rand)
{
    return .5 + (int32)shr3(rand) *.2328306e-9;
}


/* nfix() generates variates from the residue when rejection in RNOR occurs. */

static float nfix(randomizer_t * rand)
{
    const float r = 3.442620f;     /* The start of the right tail */
    static float x, y;
    
    for(;;)
    {  
        x = rand->hz*rand->wn[rand->iz];      /* iz==0, handles the base strip */
        
        if (rand->iz==0)
        { 
            do 
            { 
                x=-log(uni(rand))*0.2904764; 
                y=-log(uni(rand));
            }	/* .2904764 is 1/r */
            while(y+y<x*x);
            
            return (rand->hz>0)? r+x : -r-x;
        }
        
        /* iz>0, handle the wedges of other strips */
        if (rand->fn[rand->iz]+uni(rand)*(rand->fn[rand->iz-1]-rand->fn[rand->iz]) < exp(-.5*x*x) )
        {
            return x;
        }
        
        /* initiate, try to exit for(;;) for loop*/
        rand->hz = shr3(rand);
        rand->iz = rand->hz&127;
        if(fabs(rand->hz)<rand->kn[rand->iz]) return (rand->hz*rand->wn[rand->iz]);
    }
    
}

/* efix() generates variates from the residue when rejection in REXP occurs. */
static float efix(randomizer_t * rand)
{ 
    float x;
    
    for(;;)
    {  
        if (rand->iz==0) 
        {
            return (7.69711-log(uni(rand)));          /* iz==0 */
        }
        
        x = rand->jz*rand->we[rand->iz]; 
        
        if (rand->fe[rand->iz]+uni(rand)*(rand->fe[rand->iz-1]-rand->fe[rand->iz]) < exp(-x) )
        {
            return (x);
        }
        
        /* initiate, try to exit for(;;) loop */
        rand->jz = shr3(rand);
        rand->iz = (rand->jz & 255);
        
        if(rand->jz<rand->ke[rand->iz]) 
        {
            return (rand->jz*rand->we[rand->iz]);
        }
    }
}

/* Set up tables for RNOR */
static void zigset_nor(randomizer_t * rand, uint32 jsrseed)
{  
    const double m1 = 2147483648.0;
    double dn = 3.442619855899;
    double tn = dn;
    double vn=9.91256303526217e-3;
    
    rand->jsr ^= jsrseed;
    
    double q = vn / exp(-.5*dn*dn);
    rand->kn[0] = (dn/q)*m1;
    rand->kn[1] = 0;
    
    rand->wn[0] = q/m1;
    rand->wn[127] = dn/m1;
    
    rand->fn[0] = 1.;
    rand->fn[127] = exp(-.5*dn*dn);
    
    int i;
    for(i=126; i>=1; i--)
    {
        dn = sqrt(-2.*log(vn/dn+exp(-.5*dn*dn)));
        rand->kn[i+1] = (dn/tn)*m1;
        tn = dn;
        rand->fn[i] = exp(-.5*dn*dn);
        rand->wn[i] = dn/m1;
    }
}

/* Set up tables for REXP */
static void zigset_exp(randomizer_t * rand, uint32 jsrseed)
{  
    const double m2 = 4294967296.;
    double de = 7.697117470131487;
    double te = de;
    double ve = 3.949659822581572e-3;
    
    /* doing it twice, once from zigset_exp and from zigset_nor, cancels the effect!
       jsr^=jsrseed; */ 
    
    double q = ve/exp(-de);
    rand->ke[0] = (de/q)*m2;
    rand->ke[1] = 0;
    
    rand->we[0] = q/m2;
    rand->we[255] = de/m2;
    
    rand->fe[0] = 1.;
    rand->fe[255] = exp(-de);
    
    int i;
    for(i=254;i>=1;i--)
    {
        de = -log(ve/de+exp(-de));
        rand->ke[i+1] =  (de/te)*m2;
        te = de;
        rand->fe[i] = exp(-de);
        rand->we[i] = de/m2;
    }
}


static void zigset(randomizer_t * rand, uint32 jsrseed)
{
    zigset_nor(rand, jsrseed);
    zigset_exp(rand, jsrseed);
}

static float rnor(randomizer_t * rand)
{
    rand->hz = shr3(rand);
    rand->iz = rand->hz & 127;
        
    return (fabs(rand->hz)<rand->kn[rand->iz]) ?
    rand->hz*rand->wn[rand->iz] :
    nfix(rand);
}

static float rexp(randomizer_t * rand)
{
    rand->jz = shr3(rand);
    rand->iz = rand->jz & 255;
    
    return (rand->jz<rand->ke[rand->iz]) ?
    rand->jz*rand->we[rand->iz] :
    efix(rand);
}

/* Exported functions -------------------------------------------------- */
#include <stdlib.h>
#include "random.h"

void * random_init(int jsrseed)
{
    randomizer_t * rand = (randomizer_t *)malloc(sizeof(randomizer_t));
    
    if (!rand)
    {
        return NULL;
    }
    
    rand->jsr = 123456789;
        
    zigset(rand, jsrseed);
    
    return (void *)rand;
}

void random_free(void * v)
{
    free(v);
}

float random_rexp(void * v)
{
    return rexp((randomizer_t *)v);
}

float random_rnor(void * v)
{
    return rnor((randomizer_t *)v);
}
