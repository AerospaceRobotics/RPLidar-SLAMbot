RPLidar-SLAMbot
===============

Evolving codebase for a SLAM-capable robot using the RoboPeak LIDAR sensor

## Contents:

### libraries
The files required by the robot to run its sensors and support its functionality.
#### breezyslam
Python and C++ files to enable SLAM, released as open-source BreezySLAM (http://home.wlu.edu/~levys/software/breezyslam/).
#### Encoder
Allows precise, high-frequency, low-overhead encoder monitoring (http://www.pjrc.com/teensy/td_libs_Encoder.html).
#### RPLidarDriver
Provides simple methods for retrieving data from the RPLidar sensor (http://rplidar.robopeak.com/subsites/rplidar/download.html).

### breezySLAM_XBee.py
Contains all base station code to run the slamBot.  BreezySLAM for Python must already be installed on your machine to use this code.  Written in Python 2.7, but will add support for Python 3.x in the future.  Run in Ubuntu 14.04, however should run on all Linux and Windows machines.

### breezySLAM_XBee/breezySLAM_XBee.ino
Arduino code for the Seeeduino Mega on our slamBot.  Encoder and RPLidarDriver should be placed in the sketchbook folder to be properly added by the Arduino compiler at compile-time.  Cannot be run on an Arduino with fewer than 4 serial ports if full functionality is to be maintained.