RPLidar-SLAMbot
===============

Evolving codebase for a SLAM-capable robot using the RoboPeak LIDAR sensor

Contents:
---------

All directories and files used by the SLAMbot.


# `SLAMBotGUI-0.2.tar.gz`
Current stable release of our slambotgui package.  To install on your machine, download and unzip the file, then run the `setup.py` script with command-line option `install`.
To download and unzip:

    wget https://github.com/AerospaceRobotics/RPLidar-SLAMbot/raw/master/SLAMBotGUI-0.2.tar.gz
    tar -zxvf SLAMBotGUI-0.2.tar.gz
    cd SLAMBotGUI-0.2/
    
If you do not want to install the package yet, you can test the distribution now (there are several user preference flags near the top of the `readLogData` python file, which you should play around with to ensure full functionality):

    python readLogData.py
    
To install the package:

    sudo python setup.py install
    
If you received no errors, test your install by:

    sudo rm -rf slambotgui/
    python readLogData.py
    
If that works, congratulations, it's installed!  You can now do global imports of any of our provided classes into your code.  If you have a LIDAR unit with an Arduino interface, check out the `comms` module and how we use it in [`baseStationMain.py`](https://github.com/AerospaceRobotics/RPLidar-SLAMbot/blob/master/baseStationMain.py), also included in the distribution.


# Dependencies
The files required by the robot to run its sensors and support its functionality.
### Standard Python Libraries
The libraries we use in the Base Station code, but are not required, are PIL and OpenCV.  We also use numpy, scipy, and matplotlib, which need to be installed explicitly:
    
    sudo apt-get install python-numpy python-matplotlib python-scipy
    
### OpenCV (Python)
Although not required, OpenCV dramatically improves the performance of this library.  Matplotlib can only achieve a refresh rate of 2hz, while also bogging down the rest of the program.  Installing OpenCV is notoriously tricky, but we've had the pleasure of figuring out the best way to do it so that you don't have to ([link](https://help.ubuntu.com/community/OpenCV/)). * To start off, you'll need to download the highest version number `opencv.sh` from [this repository](https://github.com/jayrambhia/Install-OpenCV/tree/master/Ubuntu).  We've included the current one (2.4.9; as of this writing) in our repo for your convenience:

    wget https://raw.githubusercontent.com/AerospaceRobotics/RPLidar-SLAMbot/master/libraries/opencv_ubuntu.sh
    chmod +x opencv_ubuntu.sh
    ./opencv_ubuntu.sh
    
This will take a long time, so go ahead and give the rest of this README a looksy while you wait!  If it fails, check out the link above.

*working on an ODROID?  Do this instead:  
Try the steps immediately following this paragraph.  We got this from [the ODROID forum](http://forum.odroid.com/viewtopic.php?f=112&t=8036), and it works for us on our ODROID XU3-Lite which is running [the Lubuntu image from ODROID](http://odroid.in/ubuntu_14.04lts/).

    wget https://raw.githubusercontent.com/AerospaceRobotics/RPLidar-SLAMbot/master/libraries/opencv_odroid.sh
    chmod +x opencv_odroid.sh
    ./opencv_odroid.sh

### breezyslam (Python)
Python and C++ files to enable SLAM, released as open-source BreezySLAM ([link](http://home.wlu.edu/~levys/software/breezyslam/)).
### Encoder (Arduino)
Allows precise, high-frequency, low-overhead encoder monitoring ([link](http://www.pjrc.com/teensy/td_libs_Encoder.html)).
### RPLidarDriver (Arduino)
Provides simple methods for retrieving data from the RPLidar sensor ([link](http://rplidar.robopeak.com/subsites/rplidar/download.html)).


# `slamBotMain/slamBotMain.ino` (Arduino)
Arduino code for the Seeeduino Mega on our slamBot.  Encoder and RPLidarDriver should be placed in the sketchbook folder to be properly added by the Arduino compiler at compile-time.  Cannot be run on an Arduino with fewer than 4 serial ports if full functionality is to be maintained.  Hence, we recommend the Arduino Mega, or the [Seeeduino Mega](http://aerospacerobotics.com/products/seeeduino-mega-arduino-compatible-board), [which has more features](http://aerospacerobotics.com/blogs/learn/14882125-seeeduino-mega-pin-control).


# slambotgui_source (Python)
Working directory for our slambotgui package, which combines serial protocol, XBee configuration, robot control, SLAM processing, and several GUI options for any robotics system (currently designed for use with an Arduino-based SLAM-enabled robot).
### `baseStationMain.py`
Contains all base station code to run the slamBot.  BreezySLAM for Python must already be installed on your machine to use this code.  Written in Python 2.7.6, tested and functioning in Python 3.4.0.  Written and tested in Ubuntu 14.04, however should run on all Linux machines.  Have not tested in Windows or OS X.  BreezySLAM does not support Python 3.x as of this writing, however we are working on that.  OpenCV does not currently support Python 3.x, however the dev build is available and reportedly functioning ([link](http://stackoverflow.com/questions/20953273/install-opencv-for-python-3-3)).  We highly recommend running Python 2, but are trying to support Python 3 through a possible future transition to PyQt from Tkinter.
### `readLogData.py`
Stripped-down version of `baseStationMain.py` designed to be used to read log files without communication with the robot.
### `setup.py`
Script used to create tarball file and install package.
### slambotgui
Main package directory.
#### `comms.py`
Serial communication thread.  Custom serial protocol very easy to implement.
#### `components.py`
Physical robot parts, including the specific hardware we're using in our project.  Contains generalized Robot and Laser classes.
#### `guis.py`
Tkinter frames used in our GUI.  If FAST_MAPPING is true, only EntryButtons is used, which serves as the robot control panel alongside the OpenCV display window.
#### `dataprocessing.py`
Data objects to store map information.
#### `slams.py`
BreezySLAM-base SLAM classes.
#### `cvslamshow.py`
OpenCV helper class from the BreezySLAM creator, Simon D. Levy.  We made minor modifications and use it for rapid-mapping (matplotlib is very slow).
### examples
Log files included with current stable distribution.
#### `data_[date]_[room_size].log`
File we've created using baseStationMain.py to allow for project demonstation without the accompanying hardware.  The file is ascii-encoded space-delimited decimal, where scans are newline-delimited, and each scan takes the following format:

    <int16_t left_wheel_ticks> <int16_t right_wheel_ticks> <uint16_t counter_ms> <scan value at 0deg> <scan value at 1deg> ...
### `MANIFEST`(`.in`)
List of all files in current stable distribution.


# examples
Images generated by baseStationMain.py.
### `data_[date]_[room_size].png`
Image file created by reading the log data in the corresponding log file.  Just as in the naming of the log file, `room_size` in the file name denotes the approximate size of the room mapped in the log file.


# Linux Custom Baud Hack
Note: If you receive the "Invalid serial port," it is likely due to serial port permissions.  Fix with: `sudo gpasswd --add ${USER} dialout` then log out and back in.
Allows you to use non-standard baud rates on a Linux machine (more info: [link](https://groups.google.com/forum/#!msg/ultimaker/BNjPpoJpfrE/Xmbp0XxTWXEJ)).
If you receive the error "Inappropriate ioctl for device", traced back to within pySerial, apply this patch.  Please note that this appears to be fixed in the current (accessed 29DEC2014) release of pySerial.  If you try to apply the patch and the patch fails, the problem has probably already been fixed in your version of pySerial.
### Python2
Contains all the patch information for Python 2.x.
#### pyserial.patch
Created with `diff -u serialposix_old.py serialposix_new.py > pyserial.patch`.

To use:

    cd /usr/lib/python2.7/dist-packages/serial
or

    cd /usr/local/lib/python2.7/dist-packages/serial
    
depending on where pySerial is installed on your machine.  Then do:

    sudo patch serialposix.py < /[path of patch file]pyserial.patch
which would probably look like:

    sudo patch serialposix.py < /[path of this repo]/Linux\ Custom\ Baud\ Hack/Python2/pyserial.patch
if you've simply cloned this repo instead of downloading just the patch file.
#### serialposix_old.py
This is our backup of the `serialposix.py` file.
#### serialposix_new.py
This is what your `serialposix.py` file should look like after applying the patch.
### Python3
If you receive the error `TypeError: 'str' does not support the buffer interface`, traced back to within pySerial, apply this patch (more info: [link](http://stackoverflow.com/questions/5471158/typeerror-str-does-not-support-the-buffer-interface)).  The second patch in the patch file addresses this issue (the first is the baud rate hack).
#### pyserial.patch
Created with `diff -u serialposix_old.py serialposix_new.py > pyserial.patch`.

To use:

    cd /usr/lib/python3/dist-packages/serial
    sudo patch serialposix.py < /[path of patch file on your machine]/pyserial.patch
#### serialposix_old.py
This is our backup of the `serialposix.py` file.
#### serialposix_new.py
This is what your `serialposix.py` file should look like after applying the patch.


# Linux list_ports Hack
Fixes list_ports in Python 3.  Do not apply this patch to pySerial if you are running Python 2.x.
### Python3
If you receive the error `TypeError: can't use a string pattern on a bytes-like object`, traced back to within pySerial, apply this patch (more info: [link](http://stackoverflow.com/questions/5184483/python-typeerror-on-regex)).
#### listports.patch
Created with `diff -u list_ports_posix_old.py list_ports_posix_new.py > listports.patch`.

To use:

    cd /usr/lib/python3/dist-packages/serial
    sudo patch tools/list_ports_posix.py < /[path of patch file on your machine]/listports.patch
#### list_ports_posix_old.py
This is our backup of the `list_ports_posix.py` file.
#### list_ports_posix_new.py
This is what your `list_ports_posix.py` file should look like after applying the patch.


# Seeeduino Mega Extra Pins Hack ([pre-1.5](http://arduino.cc/blog/2012/10/22/arduino-1-5-support-for-the-due-and-other-processors-easier-library-installation-simplified-board-menu-etc/))
Enables pin control for digital 70-85 on Seeed Studio's Mega.
Ubuntu has `patch` and `diff` built-in, but if you're using a different OS, we recommend manually replacing the files.
Note that the first patch is optional, but we recommend it as it allows easily switching back to the unmodified Arduino IDE code.
### paths.txt
Contains the location of the files we're modifying, on both Windows and Ubuntu.  Note that your installation may be in a different location.  You will need to find it before applying the patches.
### seeedmega_menuctl.patch
Add "Seeeduino Mega" board type to Arduino IDE menu.
Created with `diff -u boards_old.txt boards_new.txt > seeedmega_menuctl.patch`.

To use:

    cd /usr/share/arduino/hardware/arduino/
    sudo patch boards.txt < /[path of patch file on your machine]/seeedmega_menuctl.patch

#### boards_old.txt
This is our backup of the `boards.txt` file.
#### boards_new.txt
This is what your `boards.txt` file should look like after applying the patch.
### seeedmega_pindef.patch
Add new board type defining the Seeeduino Mega's pin definitions.
Created with `diff -u mega/pins_arduino.h seeed/pins_arduino.h > seeedmega_pindef.patch`.

To use:

    cd /usr/share/arduino/hardware/arduino/variants/
    sudo cp -R mega seeed
    sudo patch seeed/pins_arduino.h < /[path of patch file on your machine]/seeedmega_pindef.patch

#### mega/pins_arduino.h
This is our backup of the `mega/pins_arduino.h` file.
#### seeed/pins_arduino.h
This is what your new `seeed/pins_arduino.h` file should look like after applying the patch.
### extraPinsControl/extraPinsControl.ino
Example Seeeduino Mega code demonstrating use of extra pins, with some low-level stuff.
