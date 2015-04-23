#!/usr/bin/env python

# comms.py - serial thread
# 
# Copyright (C) 2015 Michael Searing
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


from tools import bits2mask, PYTHON_SERIES, raw_input
if PYTHON_SERIES == 2: from Queue import Empty as QueueEmpty
elif PYTHON_SERIES == 3: from queue import Empty as QueueEmpty
import sys
from threading import Thread, Event # allow serial checking to happen on top of tkinter interface things
# from multiprocessing import Process, Event # allow serial checking to happen at the same time as tkinter interface things
from serial import Serial
from serial.serialutil import SerialException
from serial.tools import list_ports # get computer's port info
from time import time, sleep
from struct import unpack # parse incoming serial data

TALK_TO_XBEE = False

# Patience constants
SER_READ_TIMEOUT = 1 # time to wait for data from Arduino before connection considered lost [s]
MAX_RX_TRIES = 8 # max number of times to try to tell LIDAR to start before giving up # 1s/try

# Packet constants (shared with Arduino)
ENC_FLAG = '\xFE' # encoder data flag (&thorn)
SCN_FLAG = '\xFF' # scan data flag (&yuml)
BUF_LEN = 20 # points per transmit packet []
PKT_SIZE = 4 # length of scan data packet [bytes]
ENC_SIZE = 8 # length of encoder data packet [bytes]
DFAC = 0.5; # distance resolution factor [1/mm]
AFAC = 8.0; # angle resolution factor [1/deg]
XBEE_BAUD = 125000 # maximum baud rate allowed by Arduino and XBee [hz] # 250k=0x3D090, 125k=0x1E848


class SerialThread(Thread):
  # init defines objects and prompts user for port information if necessary
  # connectToPort attempts to establish serial port connection
  # talkToXBee allows direct communication with the XBee if optional flag is set
  # waitForResponse attempts to establish contact with the Arduino
  # stop ends serial communication with the Arduino
  # writeCmd sends data to the Arduino
  # getACK returns whether we have received an ACK from the Arduino
  # resetACK resets boolean indicating that Arduino has received a command
  # run is the main loop, which handles all serial communication

  def __init__(self, laser, statusQueue, RXQueue, TXQueue):
    super(SerialThread, self).__init__() # nicer way to initialize base class (only works with new-style classes)
    self.statusQueue = statusQueue
    self.RXQueue = RXQueue
    self.TXQueue = TXQueue
    self.distMin = laser.DIST_MIN
    self.distMax = laser.DIST_MAX

    self._stop = Event() # create flag
    self.gotACK = False

    self.connectToPort() # initialize serial connection with XBee
    if TALK_TO_XBEE: self.talkToXBee() # optional (see function)
    self.waitForResponse() # start LIDAR and make sure Arduino is sending stuff back to us

    self.ser.timeout = SER_READ_TIMEOUT

  def connectToPort(self):
    # first select a port to use
    portList = sorted([port[0] for port in list_ports.comports() if 'USB' in port[0] or 'COM' in port[0]])
    if not portList: # list empty
      sys.exit("Check your COM ports for connected devices and compatible drivers.")
    elif len(portList) == 1: # if there's only one device connected, use it
      portName = portList[0]
    else: # query user for which port to use
      print("Available COM ports:")
      print(portList)
      portRequested = raw_input("Please select a port number [%s]: " % ', '.join([str(el[-1]) for el in portList]))
      portName = (portList[0][0:-1] + portRequested) if portRequested is not "" else portList[0]

    # then try to connect to it
    try:
      self.ser = Serial(portName, baudrate=XBEE_BAUD)
    except SerialException:
      sys.exit("Invalid port.")
    else:
      sleep(1) # give time to connect to serial port
      print("Successfully connected to: %s" % portName)

  def talkToXBee(self): # allow direct interaction with XBee (Arduino code needs modification)
    if raw_input("Configure this (Arduino's) XBee before mapping? [y/N]: ").lower() == 'y':
      while True:
        inputStr = raw_input("Enter XBee command: ")
        sendStr = inputStr if (inputStr == "+++" or inputStr == '') else inputStr + '\x0D'
        self.ser.write(sendStr)
        tstart = time()
        while time() < tstart + 1: # give XBee 1 sec to respond
          if self.ser.inWaiting(): print(self.ser.read())
        if inputStr.lower() == "atcn": # we've told the XBee to exit command mode, so we should, too...
          self.ser.write('q') # tells Arduino to stop talking to XBee (shouldn't affect computer XBee...)
          break
      sys.exit("Please restart XBee and then re-run this code.") # XBee has to power cycle for change to take effect

  def waitForResponse(self):
    tryCount = 0
    while True:
      self.ser.write('l') # tell robot to start lidar
      self.ser.flushInput() # wipe the receiving buffer
      print("Command sent to robot... waiting for response... check robot power...")
      sleep(1) # give time for Arduino to send something

      if self.ser.inWaiting(): break # until we see something...

      tryCount += 1
      if tryCount >= MAX_RX_TRIES: sys.exit("No response received from Arduino.") # only try for MAX_RX_TRIES seconds before giving up

    print("Data received... live data processing commencing...")

  def stop(self, try1=True):
    if try1:
      self._stop.set() # set stop flag to True
      sleep(0.2) # give serial reading loop time to finish current point before flushInput()
      # prevents: SerialException: device reports readiness to read but returned no data (device disconnected?)
    self.ser.write('o') # tell robot to turn lidar off
    self.ser.flushInput() # empty input serial buffer
    sleep(0.5) # give time to see if data is still coming in
    if self.ser.inWaiting(): self.stop(try1=False)

  def writeCmd(self, outBytes):
    self.ser.write(outBytes)

  def getACK(self):
    return self.gotACK

  def resetACK(self):
    self.gotACK = False

  def run(self):
    MASK1 = bits2mask(range(0,4)) # 0b00001111
    MASK2 = bits2mask(range(4,8)) # 0b11110000

    lagged, missed, total, scans = 0,0,0,0
    tstart = time()
    while not self._stop.isSet(): # pulls and processes all incoming and outgoing serial data
      # relay commands from root to Arduino
      try:
        self.writeCmd(self.TXQueue.get_nowait())
      except QueueEmpty:
        pass

      # check if data in buffer is too old to be useful (300 packets)
      if self.ser.inWaiting() > 300*PKT_SIZE:
        lagged += self.ser.inWaiting()/PKT_SIZE
        self.ser.flushInput()

      # pull serial data from computer buffer (XBee)
      # in order sent (bytes comma-separated):  dist[0:7], dist[8:12] ang[0:3], ang[4:12]
      pointLine = self.ser.read(PKT_SIZE) # distance and angle (blocking)
      if len(pointLine) < PKT_SIZE: # timeout occurs
        self.statusQueue.put("ser.read() timeout. Send 'l' iff LIDAR stopped.")
        continue # try again

      if time() > tstart + 1.0: # report status of serial thread to root every second
        tstart += 1.0
        self.statusQueue.put("{:4} lagged, {:2} errors in {:4} points, {:2} scans.".format(lagged,missed,total,scans))
        lagged, missed, total, scans = 0,0,0,0

      # check for command ACK
      if pointLine == ENC_FLAG*PKT_SIZE:
        self.gotACK = True # ACK from Arduino
        continue # move to the next point

      # check for encoder data packet
      if pointLine[0:2] == ENC_FLAG*2:
        pointLine += self.ser.read(ENC_SIZE-PKT_SIZE) # read more bytes to complete longer packet
        self.RXQueue.put(unpack('<2hH',pointLine[2:])) # little-endian 2 signed shorts, 1 unsigned short
        scans += 1
        continue # move to the next point

      # check for lidar data packet
      if pointLine[-1] == SCN_FLAG:
        byte1, byte2, byte3 = unpack('<3B',pointLine[0:-1]) # little-endian 3 bytes
        distCurr = (byte1 | (byte2 & MASK1) << 8)/DFAC # 12 least-significant (sent first) bytes12 bits
        angleCurr = (byte3 << 4 | (byte2 & MASK2) >> 4)/AFAC # 4 most-significant (sent last) bytes2 bits, 8 byte1 bits
        if self.distMin < distCurr < self.distMax and 0 <= angleCurr <= 360: # data matches what was transmitted
          self.RXQueue.put((distCurr, angleCurr))
          total += 1
        else: # invalid point received (communication error)
          while self.ser.read(1) != SCN_FLAG: pass # delete current packet up to and including SCN_FLAG byte
          missed += 1
        continue # move to the next point

      while self.ser.read() != SCN_FLAG: # clear bytes out of the input buffer if pointLine didn't match anything
        pass
