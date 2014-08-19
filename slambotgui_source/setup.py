#!/usr/bin/env python

# setup.py - install file for code in Aerospace Robotics SLAMbot project
# 
# Copyright (C) 2014 Michael Searing
# 
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.


from distutils.core import setup, Extension

setup(name = 'SLAMBotGUI',
      version = '0.2',
      description = 'GUI supporting SLAM-enabled robot platform',
      packages = ['slambotgui'],
      scripts=['readLogData.py', 'baseStationMain.py'],
      author='Michael Searing and William Warner',
      author_email='michael.searing@students.olin.edu',
      url='http://aerospacerobotics.com',
      download_url='https://github.com/AerospaceRobotics/RPLidar-SLAMbot',
      license='LGPL',
      platforms='Linux; Windows',
      long_description = 'Classes for custom serial protocol and graphical representation of data'
      )
