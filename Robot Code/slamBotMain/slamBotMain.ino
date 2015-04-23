/*
slamBotMain.ino - rover code for Aerospace Robots SLAMbot project

Copyright (C) 2015 Michael Searing

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// This sketch code is based on the RPLIDAR driver library provided by RoboPeak
#include <Encoder.h>
#include <RPLidar.h>

// Macros
#define float2int(x) ((int) (0.5 + x))
#define sgn(x) ((0 < x) - (x < 0))

// Output pins
const int RPLIDAR_MOTOR = 10; // PWM pin sets RPLIDAR motor speed via MOTOCTRL signal
const int LEFT_ENABLE = 6, LEFT_PHASE = 7, RIGHT_ENABLE = 9, RIGHT_PHASE = 4;
const int MOTOR_MODE = 8; // Motor Controller Mode: HIGH = Phase-Enable, LOW = In-In
const int HEART_LED = 13;
const int outPins[] = {RPLIDAR_MOTOR, LEFT_ENABLE, LEFT_PHASE,
  RIGHT_ENABLE, RIGHT_PHASE, MOTOR_MODE, HEART_LED};

// Interrupt pins
const int LEFT_ENCODER_1 = 3, LEFT_ENCODER_2 = 2;
const int RIGHT_ENCODER_1 = 18, RIGHT_ENCODER_2 = 19;

// Other Constants (in C++, static const is redundant, ie same as const; extern const is opposite):
// Packet constants (shared with base station)
const unsigned char ENC_FLAG = '\xFE'; // encoder data flag (&thorn)
const unsigned char SCN_FLAG = '\xFF'; // scan data flag (&yuml)
const unsigned short BUF_LEN = 20; // points per transmit packet
const unsigned short PKT_SIZE = 4; // bytes per point
const unsigned short BUF_SIZE = PKT_SIZE * BUF_LEN; // bytes per transmit packet
const float DFAC = 0.5; // distance resolution factor [1/mm]
const unsigned short AFAC = 8; // angle resolution factor [1/deg]
#define XBEE_BAUD 125000 // maximum baud rate allowed by Arduino and XBee [hz]
const unsigned short DIST_MIN = 100*DFAC; // minimum distance, scaled to TX value
const unsigned short DIST_MAX = 6000*DFAC; // maximum distance, scaled to TX value
const unsigned short ANG_MIN = 0*AFAC; // minimum scan angle, scaled to TX value
const unsigned short ANG_MAX = 360*AFAC; // maximum scan angle, scaled to TX value

// Motor constants
const unsigned short MINSPEED = 150; // minimum functional motor speed
const unsigned short MAXSPEED = 250; // maximum allowable motor speed
const unsigned short TARG = 360; // 360 points per revolution

// Protocol constants
const unsigned short MASK1 = B11111111; // 0000000011111111
const unsigned short MASK2 = B00001111 << 8; // 0000111100000000
const unsigned short MASK3 = B00001111; // 0000000000001111
const unsigned short MASK4 = B11111111 << 4; // 0000111111110000

// User-set constants
bool debugXBee = false; // Should we let the computer talk directly to the XBee?

// Scan data initialization
unsigned short dist;
unsigned short ang;
bool startBit;
// byte qual;

// Counter initialization
unsigned long long nextBeat;
unsigned long long lidarFixed;
int beatDuration = 50; // Heartbeat timing loop (ms)
unsigned short ind = 0; // counter of number of scans in current revolution
// unsigned long long curTime;
// unsigned long long endTime;
short leftWheelAbs = 0;
short rightWheelAbs = 0;
int leftWheelTemp = 0;
int rightWheelTemp = 0;
int driveError = 0; // positive error = left wheel gone too far
int bufferIndex = 0; // where in the software buffer are we?
byte softBuffer[BUF_SIZE] = {}; // store points to send in BUF_LEN-packet bursts

// State variable initialization
bool updatingDriving1 = false; // Should we use encoders to verify drive goal?
bool updatingDriving2 = false; // Should we use encoders to verify drive goal again?
bool sleeping = false; // Do nothing except check for wake up command
bool runLIDAR = false; // Do stuff with LIDAR?
bool heartState = false; // Current state of heartbeat LED
bool phaseMotorDriving = true; // True: Phase-Enable; False: In-In (for motor driver)
bool fixing = false; // Are we currently trying to fix the LIDAR?

// Control value initialization
unsigned char motorspeed = 237; // approx motor speed for 360 readings per revolution
int straightness = 1; // 'P' constant for drive correction
int turnDist = 250; // encoder ticks for each wheel when turning (~45deg)
int straightDist = 350; // encoder ticks for each wheel when going straight (~0.2m)
int AMPLITUDE = 159;
int lGoal = 0;
int rGoal = 0;
int goalNext = 0; // second step in a compound command is always straight ahead
int candidate;
char inChar;
char inCmd[10] = {};
unsigned short goal = 0;

// Other object initialization
RPLidar lidar;
Encoder leftEncoder(LEFT_ENCODER_1, LEFT_ENCODER_2);
Encoder rightEncoder(RIGHT_ENCODER_1, RIGHT_ENCODER_2);

// Note that Serial1 is not used because the interrupt pins are needed for the left encoder
HardwareSerial & pcSer = Serial3;
HardwareSerial & lidarSer = Serial2;
HardwareSerial & xbeeSer = Serial;


// ---------------------Main Methods--------------------- //
// Note that Arduino both defines main() and makes function prototypes for us


void setup() {
  if(debugXBee) { pcSer.begin(XBEE_BAUD); } // if connected to computer, assume talking to XBee // 'if' broken
  lidar.begin(lidarSer); // bind RPLIDAR driver to arduino Serial2
  xbeeSer.begin(XBEE_BAUD); // initialize communication with xBee
  xbeeSer.setTimeout(5);

  for (int i=0; i<sizeof(outPins)/sizeof(i); i++) {
    pinMode(outPins[i],OUTPUT);
  }

  digitalWrite(MOTOR_MODE, phaseMotorDriving);
  nextBeat = millis() + 1000; // beat heart for fist time 1 second after setup

  zeroEncodersAbs();
}

void loop() { // 16us
  if(debugXBee and pcSer.available()>0) { checkPCInput(); }
  if(not debugXBee and xbeeSer.available()>0) { checkXBeeInput(); } // 24us idle - 60us with input
  if(millis()>nextBeat) {
    beatHeart(); nextBeat += beatDuration; // 16us
    readEncoders();
    if(updatingDriving1 or updatingDriving2) { updateDriving(); } // 64us
    // digitalWrite(HEART_LED, fixing);
    if(fixing && millis()>lidarFixed) { fixing = false; }
  }

  if(!fixing) {
    if(runLIDAR) {
      if(IS_OK(lidar.waitPoint())) { // 90us, waits for new data point (500us call-to-call in theory...)
        pullScanData(); // 36 us
        checkScanRate(); // 12 us // ensure we're getting TARG scans per revolution
        writeScanData(); // 72 us // send data to computer over serial0
      } else { fixLIDAR(); } // turn LIDAR on and make sure it's running fine
    } else { analogWrite(RPLIDAR_MOTOR, 0); } // turn motor off if not using LIDAR
  }
}

// curTime = micros();
// endTime = micros();if(endTime-curTime>8){Serial.println(); Serial.println(endTime-curTime);}


// ---------------------Sub Methods---------------------- //


bool beatHeart() {
  heartState = !heartState;
  digitalWrite(HEART_LED, heartState);
}

// LIDAR Functions
void pullScanData() {
  dist = (unsigned short) (DFAC*lidar.getCurrentPoint().distance); // Q13.-1 // distance [2mm]
  ang = (unsigned short) (AFAC*lidar.getCurrentPoint().angle); // Q9.3 // angle [0.125deg]
  startBit = lidar.getCurrentPoint().startBit; // new scan?
}
void writeScanData() {
  if(dist > DIST_MIN and dist < DIST_MAX and ang < ANG_MAX) { // only send real data
    softBuffer[bufferIndex + 0] = dist & MASK1; // least significant dist bits
    softBuffer[bufferIndex + 1] = (dist & MASK2) >> 8 | (ang & MASK3) << 4; // most significant dist bits, least significant ang bits
    softBuffer[bufferIndex + 2] = (ang & MASK4) >> 4; // most significant ang bits
    softBuffer[bufferIndex + 3] = SCN_FLAG; // end of point
    bufferIndex += PKT_SIZE;
  }
  if(startBit) { // new scan, so send queued data and encoder data (encoder data signals new scan)
    for(int i=0; i<bufferIndex; i++) { xbeeSer.write(softBuffer[i]); } // send all data to XBee for transmitting
    bufferIndex = 0;
    unsigned short currTime = millis(); // get time for odometry info
    xbeeSer.write(ENC_FLAG); xbeeSer.write(ENC_FLAG); // begin encoder info
    xbeeSer.write(leftWheelAbs & MASK1); xbeeSer.write(leftWheelAbs >> 8); // left wheel
    xbeeSer.write(rightWheelAbs & MASK1); xbeeSer.write(rightWheelAbs >> 8); // right wheel
    xbeeSer.write(currTime & MASK1); xbeeSer.write(currTime >> 8); // time
  } else if(bufferIndex == BUF_SIZE) { // we just filled the buffer
    for(int i=0; i<bufferIndex; i++) { xbeeSer.write(softBuffer[i]); } // send all data to XBee for transmitting
    bufferIndex = 0; // start writing to beginning of softBuffer
  }
}
void checkScanRate() {
  if(startBit) {
    // maintain TARG readings per revolution to minimize SLAM error
    if(ind > TARG + 1 and motorspeed < MAXSPEED) {analogWrite(RPLIDAR_MOTOR, ++motorspeed);} // too many readings
    else if(ind < TARG - 1 and motorspeed > MINSPEED) {analogWrite(RPLIDAR_MOTOR, --motorspeed);} // too few readings
    ind = 0; // reset counter
  }
  ind++;
}
void fixLIDAR() {
  analogWrite(RPLIDAR_MOTOR, 0); // stop the rplidar motor to indicate fault
  rplidar_response_device_info_t info; // prepare to receive device status
  if (IS_OK(lidar.getDeviceInfo(info, 100))) { // RPLIDAR detected
    lidar.startScan(); // attempt to restart scanning if LIDAR is healthy
    analogWrite(RPLIDAR_MOTOR, 237); // turn motor on
    fixing = true; // don't try to scan yet
    lidarFixed = millis() + 1000; // give motor time to stabilize
  }
}

// Phase/Enable Motor Driving Functions
void driveLeft(const int & lspeed) {
  digitalWrite(LEFT_PHASE, lspeed<0); // HIGH (reverse) if speed<0; LOW (fwd) if speed>=0
  analogWrite(LEFT_ENABLE, abs(lspeed)>255?255:abs(lspeed)); // Writes speed to pin
}
void driveRight(const int & rspeed) {
  digitalWrite(RIGHT_PHASE, rspeed<0); // HIGH (reverse) if speed<0; LOW (fwd) if speed>=0
  analogWrite(RIGHT_ENABLE, abs(rspeed)>255?255:abs(rspeed)); // Writes speed to pin
}
void updateDriving() { // keep relative wheel distance traveled as close to goal as possible
  if(abs(leftWheelTemp) >= abs(lGoal)-4 or abs(rightWheelTemp) >= abs(rGoal)-4) { // completed drive command
    zeroMotors(); // stop motors
    if(updatingDriving2) { // is there a second command?
      zeroEncoders(); lGoal = goalNext; rGoal = goalNext; // only matters if just finished first command
      updatingDriving2 = updatingDriving1; // only matters if just finished second command
    }
    updatingDriving1 = false; // done with first command
  } else { // not yet completed drive command
    driveError = straightness * (sgn(lGoal)*leftWheelTemp - sgn(rGoal)*rightWheelTemp);
    driveLeft(sgn(lGoal)*(AMPLITUDE - driveError));
    driveRight(sgn(rGoal)*(AMPLITUDE + driveError));
  }
}
void zeroMotors() {
  driveLeft(0);
  driveRight(0);
}

// Encoder Reading and Resetting
void readEncoders() {
  long left = leftEncoder.read();
  long right = rightEncoder.read();
  leftWheelTemp += left - leftWheelAbs;
  rightWheelTemp += right - rightWheelAbs;
  leftWheelAbs = left;
  rightWheelAbs = right;
}
void zeroEncoders() {
  leftWheelTemp = 0;
  rightWheelTemp = 0;
}
void zeroEncodersAbs() {
  leftEncoder.write(0);
  rightEncoder.write(0);
  leftWheelAbs = 0;
  rightWheelAbs = 0;
  zeroEncoders();
}

// Keyboard Control
void checkXBeeInput() {
  inChar = xbeeSer.read();
  if(sleeping and inChar != 'x') { return; } // sleeping only responds to command to wake up

  goal = 0;
  if(inChar == 'h') {
    xbeeSer.println("Hello!"); return;
  } else if(inChar == 'c') {
    zeroEncoders(); // reset the encoders in preparation for a new driving command
    updatingDriving1 = true; updatingDriving2 = true; // tell main loop to monitor driving status
    goal = xbeeSer.parseInt(); xbeeSer.read(); // get angle and clear separator
    goalNext = xbeeSer.parseInt(); xbeeSer.read(); // get distance and clear terminator
    for(int i=0; i<PKT_SIZE; i++) { xbeeSer.write(ENC_FLAG); } // command received, send ACK
  } else if(inChar == 'w' or inChar == 'a' or inChar == 's' or inChar == 'd') {
    zeroEncoders(); // reset the encoders in preparation for a new driving command
    updatingDriving1 = true; // tell main loop to monitor driving status
    goal = xbeeSer.parseInt(); xbeeSer.read(); // get associated value and clear terminator
    for(int i=0; i<PKT_SIZE; i++) { xbeeSer.write(ENC_FLAG); } // command received, send ACK
  } else if(inChar == 'W' or inChar == 'A' or inChar == 'S' or inChar == 'D') {
    zeroEncoders(); // reset the encoders in preparation for a new driving command
    updatingDriving1 = true; // tell main loop to monitor driving status
  } else if(updatingDriving1 or updatingDriving2) { // currently driving, not requested to continue driving
    zeroMotors(); // stop motors
    updatingDriving1 = false; updatingDriving2 = false; // stop monitoring of driving status
  }

  if(inChar == 'x') {runLIDAR = false; sleeping = !sleeping; beatDuration = sleeping?1000:50;

  } else if(inChar == 'l') { runLIDAR = true; bufferIndex = 0; zeroEncodersAbs();
  } else if(inChar == 'o') { runLIDAR = false;

  } else if(inChar == 'c') { lGoal = goal; rGoal = -goal; // goal is to the right (can be negative -> left)
  } else if(inChar == 'w') { lGoal = goal; rGoal = goal;
  } else if(inChar == 'a') { lGoal = -goal; rGoal = goal;
  } else if(inChar == 's') { lGoal = -goal; rGoal = -goal;
  } else if(inChar == 'd') { lGoal = goal; rGoal = -goal;
  } else if(inChar == 'W') { lGoal = straightDist; rGoal = straightDist;
  } else if(inChar == 'A') { lGoal = -turnDist; rGoal = turnDist;
  } else if(inChar == 'S') { lGoal = -straightDist; rGoal = -straightDist;
  } else if(inChar == 'D') { lGoal = turnDist; rGoal = -turnDist;

  } else if(inChar == 'v') {
    candidate = xbeeSer.parseFloat();
    xbeeSer.read(); // clear terminator
    if( candidate>=0 && candidate<=255 ) { AMPLITUDE = candidate; }
    for(int i=0; i<PKT_SIZE; i++) { xbeeSer.write(ENC_FLAG); } // command received, send ACK
  }

  while(xbeeSer.available()>0) {xbeeSer.read();}; // clear all other incoming data
}

void checkPCInput() { // allow computer to talk directly to XBee (Arduino asks as relay)
  if(pcSer.peek() == 'q') { debugXBee = false; pcSer.end(); return; } // non-blocking (yay)
  while (pcSer.available()) { xbeeSer.write(pcSer.read()); delay(5); }
  while (xbeeSer.available()) { pcSer.write(xbeeSer.read()); delay(5); }
}