#!/usr/bin/env python

'''
setup.py - Python distutils setup file for BreezySLAM package.

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
along with this code.  If not, see <http://www.gnu.org/licenses/>.
'''

from distutils.core import setup, Extension

module = Extension('coreslam', 
    sources = [ 
        'pycoreslam.c', 
        '../coreslam/coreslam.c',
        '../coreslam/rmhc_filter.c',
        '../coreslam/ziggrand.c'  
    ])

setup (name = 'BreezySLAM',
    version = '0.1',
    description = 'Simple, efficient SLAM in Python',
    packages = ['breezyslam'],
    ext_modules = [module])

