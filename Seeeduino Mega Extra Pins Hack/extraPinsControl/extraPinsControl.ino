// make sure to select Tools > Board > Seeeduino Mega, or this code won't work


unsigned long long nextBeat; // time in millis of next heart beat
int beatDuration = 1000; // 1/2 heart beat period [ms]
bool heartState = false; // initial heart LED state

bool outputs = false; // set pins 70-85 as outputs?

HardwareSerial & pcSer = Serial; // "rename" serial port to be more readable

// ---------------------Main Methods---------------------
void setup(){
  pcSer.begin(9600);
  delay(500);
  pcSer.println("Hello, World!");

  pinMode(13, OUTPUT);
  for (int i=70; i<86; i++) {
    pinMode(i,INPUT); // calling pins by number
  }


  // we can use internal stuff to find out whether a pin is set to input or output:

  // directly from register (super fast):
  uint8_t state_quickAnDirty = DDRH & _BV(2); // _BV(n) returns a byte with a 1 bit at n // _BV(2) = 0x04 = 0b00000010

  // human-readable form that Arduino does (still really fast because macros everywhere):
  uint8_t pin = DPH2; // can use labels on board as well as 70-85 numbering // precede label with "D" for digital
  uint8_t port = digitalPinToPort(pin); // which register
  uint8_t bit = digitalPinToBitMask(pin); // where in register
  uint8_t state = *portModeRegister(port) & bit; // check specific bit in specific register

  pcSer.print("Pin 70 is set to: ");
  if (state==INPUT) {
    pcSer.println("INPUT");
  } else if (state==OUTPUT) {
    pcSer.println("OUTPUT");
  }


  nextBeat = millis() + 1000; // start heart beating 1 second after program starts
}

void loop(){
  if (millis()>nextBeat) {
    beatHeart(heartState); nextBeat += beatDuration;
  }
}

// ---------------------Sub Methods---------------------
bool beatHeart(bool & heartState) {
  heartState = !heartState;
  digitalWrite(13, heartState);

  for (int i=70; i<86; i++) {
    if (outputs) {
      digitalWrite(i, heartState); // if outputs, make pins beat along with heart
    } else {
      if (digitalRead(i) == HIGH) { pcSer.println(i); } // if inputs, print HIGH pins to computer
    }
  }
}