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
#### Standard Python libraries
These are all the standard Python libraries that we use in the Base Station code
##### Version-independent libraries
sys, time, threading, serial, struct, numpy, scipy, matplotlib
##### Python2.x-specific libraries
Tkinter, tkMessageBox, Queue
##### Python3.x-specific libraries
tkinter (contains messagebox), queue

### breezySLAM_XBee.py
Contains all base station code to run the slamBot.  BreezySLAM for Python must already be installed on your machine to use this code.  Written in Python 2.7.6, tested and functioning in Python 3.4.0.  Written and tested in Ubuntu 14.04, however should run on all Linux machines.  Have not tested in Windows or OS X.

### breezySLAM_XBee/breezySLAM_XBee.ino
Arduino code for the Seeeduino Mega on our slamBot.  Encoder and RPLidarDriver should be placed in the sketchbook folder to be properly added by the Arduino compiler at compile-time.  Cannot be run on an Arduino with fewer than 4 serial ports if full functionality is to be maintained.

### Linux Custom Baud Hack
Allows you to use non-standard baud rates on a Linux machine (more info: https://groups.google.com/forum/#!msg/ultimaker/BNjPpoJpfrE/Xmbp0XxTWXEJ).
If you receive the error "Inappropriate ioctl for device", traced back to within pySerial, apply this patch.
#### Python2
Contains all the patch information for Python 2.x.
##### pyserial.patch
Created with "diff -u list_ports_posix_old.py list_ports_posix_new.py > listports.patch".
##### serialposix_old.py
This is our backup of the serialposix.py file.
##### serialposix_new.py
This is what your serialposix.py file should look like after applying the patch.
#### Python3
If you receive the error "TypeError: 'str' does not support the buffer interface", traced back to within pySerial, apply this patch (more info: http://stackoverflow.com/questions/5471158/typeerror-str-does-not-support-the-buffer-interface).  The second patch in the patch file addresses this issue (the first is the baud rate hack).
##### pyserial.patch
Created with "diff -u list_ports_posix_old.py list_ports_posix_new.py > listports.patch".
##### serialposix_old.py
This is our backup of the serialposix.py file.
##### serialposix_new.py
This is what your serialposix.py file should look like after applying the patch.


### Linux list_ports Hack
Fixes list_ports in Python 3.  Do not apply this patch to pySerial if you are running Python 2.x.
#### Python3
If you receive the error "TypeError: can't use a string pattern on a bytes-like object", traced back to within pySerial, apply this patch (more info: http://stackoverflow.com/questions/5184483/python-typeerror-on-regex).
##### listports.patch
Created with "diff -u list_ports_posix_old.py list_ports_posix_new.py > listports.patch".
##### list_ports_posix_old.py
This is our backup of the list_ports_posix.py file.
##### list_ports_posix_new.py
This is what your list_ports_posix.py file should look like after applying the patch.