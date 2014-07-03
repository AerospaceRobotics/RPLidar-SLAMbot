/*  This file is the file for upload to a MEGA-compatible microcontroller
  for use with the Aerospace Robotics SLAM Rover project. 
  Do whatever you want with this code as long as it doesn't kill people.
  No liability et cetera.
  http://www.aerospacerobotics.com/             June-August 2014
                      Michael Searing & Bill Warner

*/

// This sketch code is based on the RPLIDAR driver library provided by RoboPeak
#include <Encoder.h>
#include <RPLidar.h>

// Output pin Declaration:
const int RPLIDAR_MOTOR = 10; // PWM pin sets RPLIDAR motor speed via MOTOCTRL signal
const int LEFT_ENABLE = 6, LEFT_PHASE = 7, RIGHT_ENABLE = 9, RIGHT_PHASE = 4;
const int MOTOR_MODE = 8; // Motor Controller Mode: HIGH = Phase-Enable, LOW = In-In
const int HEART_LED = 13;
const int outPins[] = {RPLIDAR_MOTOR, LEFT_ENABLE, LEFT_PHASE,
  RIGHT_ENABLE, RIGHT_PHASE, MOTOR_MODE, HEART_LED};

// Input Pin Declaration:
const int LEFT_ENCODER_1 = 3, LEFT_ENCODER_2 = 2;
const int RIGHT_ENCODER_1 = 18, RIGHT_ENCODER_2 = 19;

// Other Constants (in C++, static const is redundant, ie same as const; extern const is opposite):
#define NEWLINE '\xFE' // response indicator (&thorn)
#define ENDLINE '\xFF' // end line terminator (&yuml)
const float dFac = 0.5; // distance resolution factor [1/mm]
const unsigned short aFac = 8; // angle resolution factor [1/deg]
const unsigned short DIST_MIN = 100*dFac; // minimum distance, factored
const unsigned short DIST_MAX = 6000*dFac; // maximum distance, factored

const unsigned short PKT_SIZE = 4; // bytes per point
const unsigned short BUF_LEN = 40; // points per transmit packet
const unsigned short BUF_SIZE = PKT_SIZE * BUF_LEN; // bytes per transmit packet

const unsigned short MINSPEED = 150; // minimum functional motor speed
const unsigned short MAXSPEED = 250; // maximum allowable motor speed
const unsigned short TARG = 360; // 360 points per revolution
const unsigned short MASK1 = B11111111; // 0000000011111111
const unsigned short MASK2 = B00001111 << 8; // 0000111100000000
const unsigned short MASK3 = B00001111; // 0000000000001111
const unsigned short MASK4 = B11111111 << 4; // 0000111111110000

// Scan data:
unsigned short dist;
unsigned short ang;
bool startBit;
// byte qual;

// Counters:
unsigned long long nextBeat;
int beatDuration = 50; // Heartbeat timing loop (ms)
unsigned short ind = 0; // counter of number of scans in current revolution
// unsigned long long curTime;
// unsigned long long endTime;
long leftWheelPos;
long rightWheelPos;
int driveError = 0; // positive error = left wheel gone too far
int bufferIndex = 0; // where in the software buffer are we?
byte softBuffer[BUF_SIZE] = {}; // store points to send in BUF_LEN-packet bursts

// State Variables:
bool updatingDriving = false; // Use encoders to verify drive goal?
bool sleeping = false; // Do nothing except check for wake up command
bool runLIDAR = false; // Do stuff with LIDAR?
bool heartState = false; // Current state of heartbeat LED
bool phaseMotorDriving = true; // True: Phase-Enable; False: In-In (for motor driver)
bool pc = false; // Are we connected to a computer?
bool cmdMode = false; // Are we in command mode with the XBee?

// Control Values:
unsigned char motorspeed = 237; // approx motor speed for 360 readings per revolution
int straightness = 1; // 'P' constant for drive correction
int turnDist = 150; // encoder ticks for each wheel when turning (~30deg)
int straightDist = 400; // encoder ticks for each wheel when going straight (~0.2m)
int AMPLITUDE = 159;
int lGoal = 0;
int rGoal = 0;
int candidate;
char inChar;
char inCmd[10] = {};
unsigned short goal = 0;

// Create objects
RPLidar lidar;
Encoder leftEncoder(LEFT_ENCODER_1, LEFT_ENCODER_2);
Encoder rightEncoder(RIGHT_ENCODER_1, RIGHT_ENCODER_2);

// Note that Serial1 is not used because the interrupt pins are needed for the left encoder
HardwareSerial & pcSer = Serial;
HardwareSerial & lidarSer = Serial2;
HardwareSerial & xbeeSer = Serial3;


// ---------------------Main Methods--------------------- //


void setup() {
  // if(pcSer) {pcSer.begin(250000); cmdMode = true; } // if connected to computer, assume talking to XBee
  lidar.begin(lidarSer); // bind RPLIDAR driver to arduino Serial2
  xbeeSer.begin(250000); // initialize communication with xBee
  xbeeSer.setTimeout(5); // used by parseInt()

  for (int i=0; i<sizeof(outPins)/sizeof(i); i++) {
    pinMode(outPins[i],OUTPUT);
  }

  digitalWrite(MOTOR_MODE, phaseMotorDriving);
  nextBeat = millis() + 1000; // beat heart for fist time 1 second after setup
}

void loop() { // 16us
  // if(cmdMode and pcSer.available()>0) { checkPCInput(); }
  if(not cmdMode and xbeeSer.available()>0) { checkXBeeInput(); } // 24us idle - 60us with input
  if(millis()>nextBeat) {
    beatHeart(heartState); nextBeat += beatDuration; // 16us
    if(updatingDriving) { readEncoders(); updateDriving(); } // 64us
  }

  if(runLIDAR) {
    if(IS_OK(lidar.waitPoint())) { // 90us, waits for new data point (500us call-to-call in theory...)
      pullScanData(dist, ang, startBit); // 36 us
      checkScanRate(ind, motorspeed, startBit); // 12 us // ensure we're getting TARG scans per revolution
      writeScanData(dist, ang); // 72 us // send data to computer over serial0
    } else { fixLIDAR(); } // turn LIDAR on and make sure it's running fine
  } else { analogWrite(RPLIDAR_MOTOR, 0); } // turn motor off if not using LIDAR
}

// curTime = micros();
// endTime = micros();if(endTime-curTime>8){Serial.println(); Serial.println(endTime-curTime);}


// ---------------------Sub Methods---------------------- //


int sgn(int val) { // for int type, find sign (int -1,0,1)
  return (int(0) < val) - (val < int(0));
}

bool beatHeart(bool & heartState) {
  heartState = !heartState;
  digitalWrite(HEART_LED, heartState);
}

// LIDAR Functions
void pullScanData(unsigned short & dist, unsigned short & ang, bool & startBit) {
  dist = (unsigned short) (dFac*lidar.getCurrentPoint().distance); // Q13.-1 distance [2mm]
  ang = (unsigned short) (aFac*lidar.getCurrentPoint().angle + 0.5f); // Q9.3 round(8*angle) [0.125deg]
  startBit = lidar.getCurrentPoint().startBit; // new scan?
}
void writeScanData(const unsigned short & dist, const unsigned short & ang) { // little-endian
  if(dist > DIST_MIN and dist < DIST_MAX and ang <= 360*aFac) {
    softBuffer[bufferIndex + 0] = dist & MASK1; // least significant dist bits
    softBuffer[bufferIndex + 1] = (dist & MASK2) >> 8 | (ang & MASK3) << 4; // most significant dist bits, least significant ang bits
    softBuffer[bufferIndex + 2] = (ang & MASK4) >> 4; // most significant ang bits
    softBuffer[bufferIndex + 3] = ENDLINE; // end of point
    if(bufferIndex == BUF_SIZE - PKT_SIZE) { // we just filled the buffer
      for(int i=0; i<BUF_SIZE; i++) {
        xbeeSer.write(softBuffer[i]); // send all data to XBee for transmitting
      }
      // xbeeSer.write(softBuffer, BUF_SIZE); // send all data to XBee for transmitting
      bufferIndex = 0; // start writing to beginning of softBuffer
    } else {
      bufferIndex += PKT_SIZE;
    }
  }
}
void checkScanRate(unsigned short & ind, unsigned char & motorspeed, const bool & startBit) {
  if(startBit) {
    // maintain 360 readings per revolution to minimize SLAM error
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
    delay(1000); // give motor time to stabilize
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
  if(abs(leftWheelPos) > abs(lGoal) or abs(rightWheelPos) > abs(rGoal)) {zeroMotors(); updatingDriving = false; return;}
  driveError = straightness * (sgn(lGoal)*leftWheelPos - sgn(rGoal)*rightWheelPos);
  driveLeft(sgn(lGoal)*(AMPLITUDE - driveError));
  driveRight(sgn(rGoal)*(AMPLITUDE + driveError));
}
void zeroMotors() {
  driveLeft(0);
  driveRight(0);
}

// Encoder Reading and Resetting
void readEncoders() {
  leftWheelPos = leftEncoder.read();
  rightWheelPos = rightEncoder.read();
}
void zeroEncoders() {
  leftEncoder.write(0);
  rightEncoder.write(0);
}

// Keyboard Control
void checkXBeeInput() {
  inChar = xbeeSer.read();
  if(sleeping && inChar != 'x') { return; } // sleeping only responds to command to wake up

  goal = 0;
  if(inChar == 'w' or inChar == 'a' or inChar == 's' or inChar == 'd') {
    zeroEncoders(); // reset the encoders in preparation for a new driving command
    updatingDriving = true; // tell main loop to monitor driving status
    goal = xbeeSer.parseInt(); // get associated value // returns 0 if no int found after 2ms
    for(int i=0; i<5; i++) { xbeeSer.write(NEWLINE); } // command received
  } else if(inChar == 'W' or inChar == 'A' or inChar == 'S' or inChar == 'D') {
    zeroEncoders(); // reset the encoders in preparation for a new driving command
    updatingDriving = true; // tell main loop to monitor driving status
  } else if(updatingDriving) { // if currently driving, but not requested to continue driving
    zeroMotors(); // stop motors
    updatingDriving = false; // stop monitoring of driving status
  }

  if(inChar == 'x') {runLIDAR = false; sleeping = !sleeping; beatDuration = sleeping?1000:50;

  } else if(inChar == 'l') { runLIDAR = true;
  } else if(inChar == 'o') { runLIDAR = false;

  } else if(inChar == 'w') { lGoal = goal; rGoal = goal;
  } else if(inChar == 'a') { lGoal = -goal; rGoal = goal;
  } else if(inChar == 's') { lGoal = -goal; rGoal = -goal;
  } else if(inChar == 'd') { lGoal = goal; rGoal = -goal;
  } else if(inChar == 'W') { lGoal = straightDist; rGoal = straightDist;
  } else if(inChar == 'A') { lGoal = -turnDist; rGoal = turnDist;
  } else if(inChar == 'S') { lGoal = -straightDist; rGoal = -straightDist;
  } else if(inChar == 'D') { lGoal = turnDist; rGoal = -turnDist;

  } else if(inChar == 'v') {
    candidate = xbeeSer.parseInt();
    if( candidate>=0 && candidate<=255 ) { AMPLITUDE = candidate; }
    for(int i=0; i<5; i++) { xbeeSer.write(NEWLINE); } // command received
  }

  while(xbeeSer.available()>0) {xbeeSer.read();}; // clear all other incoming data
}

// void checkPCInput() { // relay all serial data between computer and XBee
//   if(pcSer.peek() == 'q') { cmdMode = false; return; } // non-blocking (yay)
//   while (pcSer.available()) { xbeeSer.write(pcSer.read()); delay(5); }
//   while (xbeeSer.available()) { pcSer.write(xbeeSer.read()); delay(5); }
// }