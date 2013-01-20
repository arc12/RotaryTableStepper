
/*
-------------------
Rotary Table Driver
-------------------
For use with ArcEuroTrade 4.2A driver (160-020-00101)

Terminology:
  "step" refers to a stepper motor unit of movement
  "pulse" to the signal to move a step
  "station" refers to the unit of movement of the rotary table
  
Notes:
  1. When reverse motion occurs there will be backlash compensation by over-reversing and then advancing
  2. Similarly on zero-setting, there will be a back-and-forth action to set the table ready for forward motion
  3. Number of stations determined at time of zero-setting only.
  4. 400 pulses per stepper motor rev and a table ratio of 90 gives about 0.01 degree max table position error
*/

//uncomment the following line to emit debug info to Serial
//#define DEBUG

/* --------------
  Driver Settings
  & Table Spec
  --------------- */
// MUST have internal jumpers set to: J1: falling edge of motor pulse; J2: single clock mode (step+direction)
//pulses per stepper motor rev
const long pulses360 = 400;
// geared reduction in rotary table from input to table rotation
const long tableRatio = 90;
// number of degrees table must move backwards then forwards when compensating for backlash
const long backlashDegs = 2;

/* ----------------
   Pin Assignments
   ---------------- */
// >> Number of stations is indicated by BCD thumbwheels x 3. These are multiplexed.
// Thumbwheel common pins (driven by Arduino)
const int pinStationsCommH = 9;//hundreds
const int pinStationsCommT = 10;//tens
const int pinStationsCommU = 11;//units
// Thumbwheel data pins (read by Arduino)
const int pinStations3 = 5; //BCD 2^3
const int pinStations2 = 6; //BCD 2^2
const int pinStations1 = 7; //BCD 2^1
const int pinStations0 = 8; //BCD 2^0
// >> Operation is by (momentary) push buttons
const int pinSetZero = 4;// reset the counter so current position is "zero degrees".
const int pinStep1 = 2;//advance by 1 step
const int pinGoZero = 3;//return to the zero degrees position, going forwards or backwards.
// >> pulse and direction signals to the driver unit
const int pinPUL = A0; //PUL signal to driver (active low)
const int pinDIR = A1; //DIR signal to driver (active low)

// >> Minimal signalling to user by LED
const int pinReady = 13;

/* -------
   Timings
   ------- */
// Time (in ms) after sensing a signal on pinStep1/pinGoZero/pinSetZero during which it will be ignored.
// This is for debouncing. If pulses being sent pinStep1 is ignored.
const long buttonDelay_ms = 1000;
// minimum delay between step pulses in micro-seconds. Determines max rotation speed alongside pulses360
// NB1 there is a max speed for the motor; delay is necessary just for it to turn properly even with no load
// NB2 the timer has a resulution of 4uS
const long stepDelay_us = 2000; //2000us is probably OK for 400 steps/rev

/* -----------
    Variables
   ----------- */
long numStations = 0; //set by BCD thumbwheels
//need both station and step counters to avoid accumulating rounding error
long currentStation = 0;
long currentStep =0;
long lastUserAction = 0;//millis() reading on last user putton press. Used with buttonDelay_ms
long backlashSteps=0;//number of steps for backlash correction back-and-forth motion

void setup(){
  #ifdef DEBUG
  Serial.begin(9600);
  Serial.println("Starting Debug Monitor");
  
  #endif
  
  //pin directions
  pinMode(pinStationsCommH, INPUT);//hi-Z except when MUX output
  pinMode(pinStationsCommT, INPUT);
  pinMode(pinStationsCommU, INPUT);
  pinMode(pinStations3 , INPUT);//will be using active high and external pull-downs because the thumbwheels contain diodes!
  pinMode(pinStations2 , INPUT);
  pinMode(pinStations1 , INPUT);
  pinMode(pinStations0 , INPUT);
  pinMode(pinSetZero , INPUT_PULLUP);
  pinMode(pinStep1 , INPUT_PULLUP);
  pinMode(pinGoZero , INPUT_PULLUP);
  pinMode(pinPUL , OUTPUT);
  pinMode(pinDIR , OUTPUT);
  pinMode(pinReady , OUTPUT);

  //these two are active low so initialise to HIGH
  digitalWrite(pinPUL, HIGH);
  digitalWrite(pinDIR, HIGH);
  
  //derived quantities
  backlashSteps = tableRatio * pulses360 * backlashDegs / 360;
  
  //read No. stations and init
  setZero();
}

// loop reads push buttons and triggers actions
void loop(){ 
  //allow for debounce (or twitchy fingers)
  if(millis() - lastUserAction > buttonDelay_ms){
    //active low pushbutton signals
    if(digitalRead(pinSetZero) == LOW){
      lastUserAction = millis();
      setZero();
      #ifdef DEBUG
      debugLog();
      #endif
    }else if(digitalRead(pinStep1) == LOW){
      lastUserAction = millis();
      currentStation++;
      int steps = currentStation * pulses360 * tableRatio / numStations  - currentStep;
      moveSteps(steps, false);
      currentStep +=steps;
      //check for full circle and keep counters in limits if so
      if(currentStation == numStations){
        currentStation = 0;
        currentStep = 0;
      }
      #ifdef DEBUG
      debugLog();
      #endif
    }else if(digitalRead(pinGoZero) ==  LOW){
      lastUserAction = millis();
      //if more than half-way around, go forwards to zero otherwise go back
      if(currentStation*2 >= numStations){
        moveSteps(pulses360 * tableRatio - currentStep, false);
      }else{
        moveSteps(currentStep, true); 
        backlashShuffle();  //we went backwards! 
      }
      //keep counters in limits 
      currentStation = 0;
      currentStep = 0;
      #ifdef DEBUG
      debugLog();
      #endif
    }
  }
}

void debugLog(){
    Serial.print("Microseconds=");
    Serial.print(micros());
    Serial.print("  Current Station=");
    Serial.print(currentStation);      
    Serial.print("  Current Step=");
    Serial.println(currentStep);
}
//read the number of stations, zero counters and do backlash compensation
void setZero(){
  //compose the number of stations from BCD. Note pullups are used and signalling is active low.
  numStations = readBCD(pinStationsCommH)*100 + readBCD(pinStationsCommT)*10 + readBCD(pinStationsCommU);
  //zero station and step counters and backlash correct
  currentStation = 0;
  currentStep = 0;
  backlashShuffle();
  //signal ready
  digitalWrite(pinReady, HIGH);
  #ifdef DEBUG
  Serial.print("Stations=");
  Serial.println(numStations,DEC);
  #endif
}

void backlashShuffle(){
  //backlash correction
  moveSteps(backlashSteps, true);
  moveSteps(backlashSteps, false);
}

//read the BCD value for the identified HTU digit (muxPin = pinStationsCommH etc)
int readBCD(int muxPin){
  //make the mux pin an output (normal state is high imp input - ie no pullup)
  pinMode(muxPin, OUTPUT);
  //using active high because the thumbwheels contain diodes!
  digitalWrite(muxPin, HIGH);  
  int val= digitalRead(pinStations0) + (digitalRead(pinStations1)<<1) + (digitalRead(pinStations2)<<2) + (digitalRead(pinStations3)<<3);
  //revert to hi-Z
  pinMode(muxPin, INPUT);
  return val;
}

//send pulses to move steps steps forwards or backwards (if back=true)
// delay inserted to ensure stepDelay_us between step pulses
void moveSteps(int steps, bool back){
  //signal busy = no ready
  digitalWrite(pinReady, LOW);
  
  //direction
  if(back){
    digitalWrite(pinDIR, LOW);
  }else{
    digitalWrite(pinDIR, HIGH);
  }
  unsigned long t=0;
  int pulse=0;
  //send pulses. Pulses must be >500ns but this should easily be exceeded with the following code and a normal 16MHz clock
  //logic analyser shows 6us between LOW and HIGH writes propagating to hardware
  while(pulse<steps){
    t=micros();
    digitalWrite(pinPUL, LOW);
    digitalWrite(pinPUL, HIGH);
    pulse++;
    // delay if needed
    while(micros()-t < stepDelay_us);
  }
  
  //signal ready again
  digitalWrite(pinReady, HIGH);
}
